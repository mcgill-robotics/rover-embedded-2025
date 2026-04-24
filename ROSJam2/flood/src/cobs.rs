use std::vec;

const MAX_CHUNK_SIZE:u8 = 255;

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
	// println!("{}, {}, {}, {}, {}, {}, {:?}", output_write_head, us_chunk_size, index, write_start, output.capacity(), output.len(), output);
	output[output_write_head..(output_write_head+us_chunk_size)].copy_from_slice(&input_buf[write_start..index]);
	output[last_replaced] = chunk_size+1;
	output.push(0);
	// println!("{}, {}, {}, {}, {}, {}, {:?}", output_write_head, us_chunk_size, index, write_start, output.capacity(), output.len(), output);
	return output;

}

pub fn decode(input_buf:Vec<u8>, delim:u8) -> (Vec<u8>, usize) {
	while 
	return (vec![], 0);
}