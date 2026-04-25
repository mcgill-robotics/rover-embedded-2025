use std::{collections::VecDeque, io::Cursor, thread::sleep, time::{Duration, Instant, SystemTime}};

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{decode, encode, estimate_size};

fn main() {
    let port = serialport::new("/dev/ttyACM1", 115200)
		.timeout(Duration::from_secs(1))
        .exclusive(false);
    if let Ok(mut interface) = port.open(){
        let mut buffer = [0u8;10000];
		let mut data:VecDeque<u8> = VecDeque::new();
		let mut counter = 0;
		let mut send = true;
		let mut times:VecDeque<f64> = VecDeque::new();
		let mut initial = true;
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
            let send_data = rmp_serde::to_vec(&final_str_data).unwrap();
            let map_value = Value::Map(vec![(Value::String("topic".into()),Value::String("uart0".into())), ((Value::String("data".into()), Value::Binary(send_data)))]);
            let mut msg_pack_packet = Vec::new();
            rmpv::encode::write_value(&mut msg_pack_packet, &map_value);
            msg_pack_packet.push(b'\n');
            let cobs_encoded_frame = encode(msg_pack_packet, 0);
            
            match interface.write_all(&cobs_encoded_frame){
                Ok(()) => {                    
                    counter+=1;
                    
                },
                Err(err) => {
                    println!("Failed to write {}", err);
                    return;
                }
            }
			send = false;

			while !send {
				if let Ok(read) = interface.read(&mut buffer){
					for element in &buffer[..read] {
						data.push_back(*element);
					}
					if let Ok((decoded, read)) = decode(data.make_contiguous(), 0){
						data.drain(0..read);
						let mut stream = Cursor::new(decoded);
						if let Value::Map(map) = rmpv::decode::read_value(&mut stream).unwrap(){
							let mut topic_str:String = String::new();
							if let Value::String(topic) = &map[0].1 {
								topic_str = topic.clone().into_str().expect("no topic");
							}
							if topic_str == "diag0"{
								let elapsed = start.elapsed().as_nanos();
								if let Value::Binary(decoded_data) = &map[1].1 {
									let mut data_stream = Cursor::new(decoded_data);
									if let Value::String(data_string) = rmpv::decode::read_value(&mut data_stream).unwrap(){
										let current_sys_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();
										let rust_str_data_string = data_string.into_str().expect("No string");
										let num:u128 = rust_str_data_string.parse().unwrap();
										print!("\x1B[2J");
										print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
										println!("ROSJam latency:");
										println!("rtt: {}ns", elapsed);
										println!("rtt (systime sent on packet): {}ns", current_sys_time-num);
										if !times.is_empty() {
											let total:f64 = times.iter().sum();
											let count = times.len() as f64;
											let avg_rtt = total/count;
											println!("Avg rtt (200 samples): {}ns", avg_rtt);
										}
										times.push_back(elapsed as f64);
										if times.len() > 200 {
											times.pop_front();
										}
										send = true;
										sleep(Duration::from_millis(100));
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
