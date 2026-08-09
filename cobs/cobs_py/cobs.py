def encode(input_buf:bytes, delim:int):
	output = bytearray()
	non_stuffed_count = 0
	last_replaced = 1
	output.append(delim)
	output.append(0)
	for byte in input_buf:
		if non_stuffed_count == 254:
			output[last_replaced] = non_stuffed_count+1
			non_stuffed_count = 0
			last_replaced = len(output)
			output.append(0)
		if byte == delim:
			output[last_replaced] = non_stuffed_count+1
			non_stuffed_count = 0
			last_replaced = len(output)
			output.append(0)
		else:
			output.append(byte)
			non_stuffed_count+=1
	
	output[last_replaced] = non_stuffed_count+1
	output.append(delim)
	return output

def decode(input_buf:bytearray, delim:int):
	"""
		will consume bytes if successful
		returns decoded frame and bytes read
		error code for bytes read:
		>0: no error
		-1 needs more data
		-2 all data to throw
	"""
	output = bytearray()
	delim_count = 0
	next_delim_replacement_pos = 255
	chunk_size = 0
	read = 0
	for i, byte in enumerate(input_buf):
		read = i
		if byte == delim:
			delim_count+=1
			chunk_size = 255
			next_delim_replacement_pos = 255
			if delim_count == 1:
				continue
		
		if delim_count == 0:
			continue
		elif delim_count>1:
			break
		
		if chunk_size == next_delim_replacement_pos:
			if next_delim_replacement_pos != 255:
				output.append(delim)
			next_delim_replacement_pos = byte
			chunk_size = 1
			continue
		output.append(byte)
		chunk_size+=1
	if delim_count == 0:
		return output, -2
	elif delim_count == 1:
		return output, -1
	del input_buf[:read+1]
	return output, read+1
	

if __name__ == "__main__":
	input_buf = "ab123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890bcdefghaasdfasfaf"
	delim = 'a'.encode('ascii')[0]
	print(input_buf)
	enc_buf = bytearray()
	frames = 0
	for i in range(5):
		enc = encode(input_buf.encode(), delim)
		enc_buf.extend(enc)
		print(f"Wrote: {len(enc)}")
	
	
	while enc_buf:
		print(len(enc_buf))
		print(enc_buf)
		dec, read = decode(enc_buf, delim)
		print(f"read: {read}")
		print(f"Wrote: {len(dec)}")
		if read < 0:
			break
		dec = dec.decode()
		print(dec)
		print(input_buf==dec)
		frames+=1
	
	print(f"Read {frames} frames")