mod cobs;
use std::time::Duration;

use rmpv::Value;
use serde::Serialize;
use serialport::{SerialPort, SerialPortBuilder};

use crate::cobs::{decode, encode, estimate_size};

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
            // if let Ok(available) = interface.bytes_to_read() && available > 0{
            //     match interface.read(&mut buffer[..]) {
            //         Ok(read_bytes) => {
            //             println!("Read: {}", read_bytes);
            //         },
            //         Err(err) => {
            //             return;
            //         }
            //     } 
                 
            // }
            let strData = String::from("Hello uart0 ");
            let finalStrData = strData+&counter.to_string();
            let data = rmp_serde::to_vec(&finalStrData).unwrap();
            // println!("{:?}", data);
            // println!("data field len: {}", &data.len());
            let map_value = Value::Map(vec![(Value::String("topic".into()),Value::String("uart0".into())), ((Value::String("data".into()), Value::Binary(data)))]);
            let mut msg_pack_packet = Vec::new();
            rmpv::encode::write_value(&mut msg_pack_packet, &map_value);
            msg_pack_packet.push(b'\n');
            let msgpack_len =  msg_pack_packet.len();
            // println!("MsgPack len: {}", msgpack_len);
            // println!("{:?}", msg_pack_packet);
            let cobs_encoded_frame = encode(msg_pack_packet, 0);
            // println!("{} {}", cobs_encoded_frame.len(), estimate_size(msgpack_len));
            match interface.write_all(&cobs_encoded_frame){
                Ok(()) => {
                    // if counter % 1000 == 0{
                    println!("Sent packet {}", counter);
                    // }
                    
                    counter+=1;
                    // println!("Wrote {}", written_bytes);
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
    // let mut v1 = vec![0,1,2,3,4,5];
    // let v2 = vec![6,7];
    // v1[0..2].copy_from_slice(&v2);
    // println!("{:?}", v1);
}
