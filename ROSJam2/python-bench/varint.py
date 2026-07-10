DATA_MASK = 0x7F
CONTINUE_BITS = 0x80

def encode(num:int) -> bytearray:
	output = bytearray()
	while True:
		if (num & ~DATA_MASK) == 0:
			output.append(num & DATA_MASK)
			break
		output.append((num&DATA_MASK)|CONTINUE_BITS)
		num >>= 7
	return output

def decode(buf:bytearray) -> int:
	output = 0
	for index, byte in enumerate(buf):
		output |= (byte&DATA_MASK) << index*7
		if (byte & CONTINUE_BITS) == 0:
			break
		if index*7>= 32:
			raise ValueError("VarInt too long")
	return output

if __name__ == "__main__":
	num = 255
	data = encode(num)
	for byte in data:
		print(byte)
	decoded = decode(data)
	print(num==decoded)