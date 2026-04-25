use std::time::Duration;

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use cobs_rs::cobs::{decode, encode, estimate_size};

fn main() {
    let mut counter = 0;
    let mut initial = true;
    let port = serialport::new("/dev/ttyACM1", 115200)
        .timeout(Duration::from_secs(1))
        .exclusive(false);
    if let Ok(mut interface) = port.open(){
        let mut buffer = [0u8;10000];
        while true {
            if initial {
                let buf = [0u8];
                if let Ok(()) = interface.write_all(&buf[..]) {
                    initial = false;
                }
            }
            let strData = String::from("Hello uart0 12345678901234567890123456789 ");
            let finalStrData = strData+&counter.to_string();
            let data = rmp_serde::to_vec(&finalStrData).unwrap();
            
            let map_value = Value::Map(vec![(Value::String("topic".into()),Value::String("uart0".into())), ((Value::String("data".into()), Value::Binary(data)))]);
            let mut msg_pack_packet = Vec::new();
            rmpv::encode::write_value(&mut msg_pack_packet, &map_value);
            msg_pack_packet.push(b'\n');
            
            
            let cobs_encoded_frame = encode(msg_pack_packet, 0);
            
            match interface.write_all(&cobs_encoded_frame){
                Ok(()) => {
                    
                    println!("Sent packet {}", counter);
                    
                    
                    counter+=1;
                    
                },
                Err(err) => {
                    println!("Failed to write {}", err);
                    return;
                }
            }
            
        }
    } else {
        println!("Could not open");
    }
}
