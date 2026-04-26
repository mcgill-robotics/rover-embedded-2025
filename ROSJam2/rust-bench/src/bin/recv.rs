use std::{collections::VecDeque, io::Cursor, sync::mpsc::{self, Receiver, Sender}, thread, time::{Duration, Instant}};

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{decode, encode, estimate_size};



fn main() {
    let port = serialport::new("/dev/ttyACM1", 115200)
        .timeout(Duration::from_secs(0))
        .exclusive(false);
    if let Ok(mut interface) = port.open(){
        let mut buffer = [0u8;1024];
		let mut data:VecDeque<u8> = VecDeque::new();
		let mut first_frame = true;
		let mut first_frame_diag = true;
		let mut counter = 0;
		let mut diag_counter =0;
		let mut mismatched_count = 0;
		let mut diag_mismatched_count = 0;
        while true {
			// let toRead = interface.bytes_to_read().unwrap();
			// if toRead != 0 {
			// 	println!("Available: {}", toRead);
			// }
			
            if let Ok(read) = interface.read(&mut buffer){
				for element in &buffer[..read] {
					data.push_back(*element);
				}
				if let Ok((mut decoded, read)) = decode(data.make_contiguous(), 0){
					data.drain(0..read);
					decoded.remove(decoded.len()-1);
					let mut stream = Cursor::new(decoded);
					if let Value::Map(map) = rmpv::decode::read_value(&mut stream).unwrap(){
						let mut topic_str:String = String::new();
						if let Value::String(topic) = &map[0].1 {
							topic_str = topic.clone().into_str().expect("no topic");
						}
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
												println!("mismatched");
												mismatched_count +=1;
											} 
										}
										counter = num;
										// println!("Topic: {}", topic_str);
										// println!("Data: {}", rust_str_data_string);
										// println!("Counter: {}", num);
									} else {
										if first_frame_diag {
											first_frame_diag = false
										} else {
											if diag_counter+1 != num {
												println!("mismatched");
												diag_mismatched_count +=1;
											} 
										}
										diag_counter = num;
										print!("\x1B[2J");
										print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
										println!("Topic: {}", topic_str);
										println!("Data: {}", rust_str_data_string);
										println!("Mismatches: {}", mismatched_count);
										println!("Diag0 Mismatches: {}", diag_mismatched_count);
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
