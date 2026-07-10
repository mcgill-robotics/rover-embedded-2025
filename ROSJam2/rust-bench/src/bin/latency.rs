use std::{collections::VecDeque, io::Cursor, sync::mpsc::{Receiver, Sender}, thread::sleep, time::{Duration, Instant, SystemTime}};

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{self, decode, encode, estimate_size};

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
        let mut buffer = [0u8;1024];
		let mut data:VecDeque<u8> = VecDeque::new();
		let mut send = true;
		let mut times:VecDeque<f64> = VecDeque::new();
		let mut initial = true;
		let init_time = Instant::now();
		let mut first_frame = true;
		let mut counter = 0;
		let mut mismatched_count = 0;
		let mut throughput_period_start = Instant::now();
		let mut period_data_read = 0;
		let mut period_data_send = 0;
		let mut throughput:f64 = 0.0;
		let mut send_throughput = 0.0;
		let mut counter2 = 0;
        while true {
			if initial {
                let buf = [0u8];
                if let Ok(()) = interface.write_all(&buf[..]) {
                    initial = false;
                }
            }
			let start = Instant::now();
			let sys_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();
            let final_str_data = counter2.to_string();
            // let final_str_data = String::from("Hello uart0 12345678901234567890123456789 ");
            let send_data = rmp_serde::to_vec(&final_str_data).unwrap();
            let map_value = Value::Map(vec![(Value::String("topic".into()),Value::String("diag0".into())), ((Value::String("data".into()), Value::Binary(send_data)))]);
            let mut msg_pack_packet = Vec::new();
            rmpv::encode::write_value(&mut msg_pack_packet, &map_value);
            msg_pack_packet.push(b'\n');
            let cobs_encoded_frame = encode(msg_pack_packet, 0);
			println!("Encode time: {}", start.elapsed().as_nanos());
            let write_start = Instant::now();
			// println!("{}, {:?}", cobs_encoded_frame.len(), cobs_encoded_frame);
            match interface.write_all(&cobs_encoded_frame){
                Ok(()) => {
					period_data_send+=cobs_encoded_frame.len();
				},
                Err(err) => {
                    println!("Failed to write {}", err);
                    return;
                }
            }
			println!("Write time {}", write_start.elapsed().as_nanos());
			// println!("throughput: {} kb/s", throughput);
			// println!("send throughput: {} kb/s", send_throughput);
			interface.flush();
			send = false;
			// sleep(Duration::from_millis(1000));
			while !send {
				// break;
				if let Ok(read) = interface.read(&mut buffer){
					// println!("found data {}, {:?}", read, &buffer[..read]);
					
					period_data_read+=read;
					for element in &buffer[..read] {
						data.push_back(*element);
					}
					// println!("total  data {:?}", data);
					let decode_start = Instant::now();
					if let Ok((mut decoded, read)) = decode(data.make_contiguous(), 0){
						// println!("dec: {:?}", decoded);
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
										// println!("{}", rust_str_data_string);
										// println!("{}", sys_time);
										let num:u128 = match rust_str_data_string.parse() {
											Ok(n) => n,
											Err(_) => panic!("bad number")
										};
										print!("\x1B[2J");
										print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
										println!("ROSJam latency:");
										println!("Decode time: {}", decode_start.elapsed().as_nanos());
										println!("rtt: {}ns", elapsed);
										println!("id: {}, {}", counter2, num);
										// println!("rtt (systime sent on packet): {}ns", current_sys_time-num);
										println!("throughput: {} kb/s", throughput);
										println!("send throughput: {} kb/s", send_throughput);
										if !times.is_empty() {
											let total:f64 = times.iter().sum();
											let count = times.len() as f64;
											let avg_rtt = total/count;
											println!("Avg rtt ({} samples): {}ns", times.len() , avg_rtt);
										}
										println!("Mismatched on other messages {}", mismatched_count);
										times.push_back(elapsed as f64);
										if times.len() > 2000 {
											times.pop_front();
										}
										counter2+=1;
										send = true;
									}
								}
							} else {
								if let Value::Binary(decoded_data) = &map[1].1 {
									let mut data_stream = Cursor::new(decoded_data);
									if let Value::String(data_string) = rmpv::decode::read_value(&mut data_stream).unwrap(){
										let rust_str_data_string = data_string.into_str().expect("No string");
										if let Some(num_str) = rust_str_data_string.split(" ").last() {
											let num:i32 = num_str.parse().unwrap();
											if topic_str != "diag0"{
												if first_frame {
													first_frame = false
												} else {
													if counter+1 != num {
														mismatched_count +=1;
													} 
												}
												counter = num;
												// println!("Topic: {}", topic_str);
												// println!("Data: {}", rust_str_data_string);
												// println!("Counter: {}", num);
											}
										}
									}
								}
							} 		
						}
					}
				}
			}
			let elapsed = throughput_period_start.elapsed();
			if elapsed.as_secs() >= 1 {
				throughput = (period_data_read as f64/elapsed.as_millis() as f64)*1000.0/1024.0;
				send_throughput = (period_data_send as f64/elapsed.as_millis() as f64)*1000.0/1024.0;
				throughput_period_start = Instant::now();
				period_data_read = 0;
				period_data_send = 0;
			}
        }
    } else {
        println!("Could not open");
    }
    
}
