use std::{collections::VecDeque, error::{self, Error}, fmt, iter::TakeWhile, vec};

const MAX_CHUNK_SIZE:u8 = 255;
#[derive(Debug)]
enum DecodeError{
	NotEnoughBytes,
	NoDelimiterfound
} 
impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "COBS decoding error")
    }
}

impl error::Error for DecodeError{}

pub fn estimate_size(size:usize) -> usize{
	let max_size_usize:usize = MAX_CHUNK_SIZE.into();
	return 1+(size/(max_size_usize-1)+1)+size
}

pub fn encode(input_buf:Vec<u8>, delim:u8) -> Vec<u8> {
	let mut output:Vec<u8> = Vec::with_capacity(estimate_size(input_buf.len()));
	let mut last_replaced = 0;
	let mut chunk_size = 0;
	// append temp data for header
	output.push(0);
	for (index, current_byte) in input_buf.iter().enumerate(){
		let us_chunk_size:usize = chunk_size.into();
		let output_write_head = output.len();
		if chunk_size+1 == MAX_CHUNK_SIZE {
			output.reserve(us_chunk_size);
			output.resize(output_write_head+us_chunk_size, 0);
			let write_start = index-us_chunk_size;
			output[output_write_head..(output_write_head+us_chunk_size)].copy_from_slice(&input_buf[write_start..index]);
			
			output[last_replaced] = chunk_size+1;
			last_replaced = output_write_head+us_chunk_size;

			output.push(0);
			chunk_size = 0;
		}
		if *current_byte == delim {
			output.reserve(us_chunk_size);
			output.resize(output_write_head+us_chunk_size, 0);
			let write_start = index-us_chunk_size;
			output[output_write_head..output_write_head+us_chunk_size].copy_from_slice(&input_buf[write_start..index]);
			
			output[last_replaced] = chunk_size+1;
			last_replaced = output_write_head+us_chunk_size;

			output.push(0);
			chunk_size = 0;
		} else {
			chunk_size+=1;
		}
	}
	let output_write_head = output.len();
	let us_chunk_size:usize = chunk_size.into();
	output.reserve(us_chunk_size);
	output.resize(output_write_head+us_chunk_size, 0);
	let index = input_buf.len();
	let write_start = index-us_chunk_size;
	
	output[output_write_head..(output_write_head+us_chunk_size)].copy_from_slice(&input_buf[write_start..index]);
	output[last_replaced] = chunk_size+1;
	output.push(0);
	
	return output;

}

pub fn decode(input_buf:&[u8], delim:u8) -> Result<(Vec<u8>, usize), Box<dyn Error>> {
	let mut input_index = 0;
	let mut output: Vec<u8> = Vec::new();
	let mut delim_count = 0;
	let mut chunk_size = 0;
	while input_index < input_buf.len() {
		let current_byte = input_buf[input_index];
		if current_byte == delim {
			delim_count+=1;
			input_index+=1;
			chunk_size = MAX_CHUNK_SIZE-1;
			if delim_count == 1 {
				continue;
			}
		}
		if delim_count == 0 {
			input_index+=1;
			continue;
		} else if delim_count>1 {
			break;
		}

		let current_value = input_buf[input_index];
		input_index+=1;
		if current_value == 0{
			continue;
		}
		if chunk_size==MAX_CHUNK_SIZE-1 {
			chunk_size = current_value-1;
		} else {
			chunk_size = current_value-1;
			output.push(delim);
		}

		
		let current_output_index = output.len();
		output.reserve(chunk_size.into());
		let chunk_size_usize:usize = chunk_size.into();
		output.resize(current_output_index+chunk_size_usize, 0);
		output[current_output_index..current_output_index+chunk_size_usize].copy_from_slice(&input_buf[input_index..input_index+chunk_size_usize]);
		input_index+=chunk_size_usize;
		
	}
	if delim_count == 0 {
		return Err(Box::new(DecodeError::NoDelimiterfound));
	} else if delim_count == 1 {
		return Err(Box::new(DecodeError::NotEnoughBytes));
	}
	return Ok((output, input_index-1));
}