use std::{collections::VecDeque, io::Cursor, time::Duration};

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{decode, encode, estimate_size};

fn main() {
    let port = serialport::new("/dev/ttyACM1", 115200)
        .timeout(Duration::from_secs(0))
        .exclusive(false);
    if let Ok(mut interface) = port.open(){
        let mut buffer = [0u8;10000];
		let mut data:VecDeque<u8> = VecDeque::new();
		let mut first_frame = true;
		let mut counter = 0;
        while true {
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
											} 
										}
										counter = num;
										println!("Topic: {}", topic_str);
										println!("Data: {}", rust_str_data_string);
										println!("Counter: {}", num);
									} else {
										
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
