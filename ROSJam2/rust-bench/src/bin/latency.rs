use std::{collections::VecDeque, io::Cursor, sync::mpsc::{Receiver, Sender}, thread::sleep, time::{Duration, Instant, SystemTime}};

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{decode, encode, estimate_size};

fn sender(interface:Box<dyn SerialPort>){

}

fn receiver(interface:Box<dyn SerialPort>, instants:Receiver<Instant>, notifier:Sender<bool>){

}


fn main() {
    let port = serialport::new("/dev/ttyACM1", 115200)
		.timeout(Duration::from_secs(1))
        .exclusive(false);
    if let Ok(mut interface) = port.open(){
		// let interface2 = interface.try_clone().unwrap();
		// let instant_channel = mpsc::channel::<Instant>();
		// let received_updates = mpsc::channel::<bool>();
		// thread::spawn(move ||{
		// 	receiver(interface2);
		// });
        let mut buffer = [0u8;10000];
		let mut data:VecDeque<u8> = VecDeque::new();
		let mut send = true;
		let mut times:VecDeque<f64> = VecDeque::new();
		let mut initial = true;
		let init_time = Instant::now();
        while true {
			if initial {
                let buf = [0u8];
                if let Ok(()) = interface.write_all(&buf[..]) {
                    initial = false;
                }
            }
			let start = Instant::now();
			let sys_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();
            let final_str_data = sys_time.to_string();
            // let final_str_data = String::from("Hello uart0 12345678901234567890123456789 ");
            let send_data = rmp_serde::to_vec(&final_str_data).unwrap();
            let map_value = Value::Map(vec![(Value::String("topic".into()),Value::String("diag0".into())), ((Value::String("data".into()), Value::Binary(send_data)))]);
            let mut msg_pack_packet = Vec::new();
            rmpv::encode::write_value(&mut msg_pack_packet, &map_value);
            msg_pack_packet.push(b'\n');
            let cobs_encoded_frame = encode(msg_pack_packet, 0);
			println!("Encode time: {}", start.elapsed().as_nanos());
            let write_start = Instant::now();
            match interface.write_all(&cobs_encoded_frame){
                Ok(()) => {
				},
                Err(err) => {
                    println!("Failed to write {}", err);
                    return;
                }
            }
			println!("Write time {}", write_start.elapsed().as_nanos());
			interface.flush();
			send = false;
			// sleep(Duration::from_millis(1000));
			while !send {
				if let Ok(read) = interface.read(&mut buffer){
					for element in &buffer[..read] {
						data.push_back(*element);
					}
					let decode_start = Instant::now();
					if let Ok((mut decoded, read)) = decode(data.make_contiguous(), 0){
						data.drain(0..read);
						decoded.remove(decoded.len()-1);
						let mut stream = Cursor::new(decoded);
						if let Value::Map(map) = rmpv::decode::read_value(&mut stream).unwrap(){
							let mut topic_str:String = String::new();
							if let Value::String(topic) = &map[0].1 {
								topic_str = topic.clone().into_str().expect("no topic");
							}
							// println!("{}", topic_str);
							if topic_str == "diag0"{
								if let Value::Binary(decoded_data) = &map[1].1 {
									let mut data_stream = Cursor::new(decoded_data);
									if let Value::String(data_string) = rmpv::decode::read_value(&mut data_stream).unwrap(){
										
										let elapsed = start.elapsed().as_nanos();
										let current_sys_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();
										let rust_str_data_string = data_string.into_str().expect("No string");
										let num:u128 = match rust_str_data_string.parse() {
											Ok(n) => n,
											Err(_) => continue
										};
										print!("\x1B[2J");
										print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
										println!("ROSJam latency:");
										println!("{}", init_time.elapsed().as_nanos());
										println!("Decode time: {}", decode_start.elapsed().as_nanos());
										println!("rtt: {}ns", elapsed);
										println!("rtt (systime sent on packet): {}ns", current_sys_time-num);
										if !times.is_empty() {
											let total:f64 = times.iter().sum();
											let count = times.len() as f64;
											let avg_rtt = total/count;
											println!("Avg rtt ({} samples): {}ns", times.len() , avg_rtt);
										}
										times.push_back(elapsed as f64);
										if times.len() > 2000 {
											times.pop_front();
										}
										send = true;
									}
								}
							} 		
						}
					}
				}
			}
        }
    } else {
        println!("Could not open");
    }
    
}
