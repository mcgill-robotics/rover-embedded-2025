#include <string.h>
#include <stdio.h>
#include "cobs.h"
#include "varint.h"

uint8_t encode_buffer[10000];
uint8_t decode_buffer[10000];

int float_to_string(float number, int precision, char* buf, int buf_len){
    int len = snprintf(NULL, 0, "%.*f", precision, number);
	if (len +1 > buf_len){
		return -1;
	}
    snprintf(buf, len + 1, "%.*f", number);
}

void test_cobs(){
		char buf[1000];
	float_to_string(12.33454565, 2, buf, 1000);
	printf("%s\n", buf);
	char* test1 = "ab123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890bcdefghaasdfasfaf";
	// char* test1 = "abcdefghaasdfasfaf";
	uint8_t* write_head = encode_buffer;
	*write_head = 'a';
	write_head++;
	printf("Og len: %d\n", strlen(test1));
	cobs_reader_t reader;
	cobs_setup_stream_reader(&reader);
	for (int i=0;i<5;i++){
		int slen = strlen(test1);
		printf("estimate: %d \n", cobs_estimate_encoded_size(slen));
		int len = cobs_encode(&reader, test1, slen, write_head, cobs_estimate_encoded_size(slen), 'a');
		printf("Wrote %d \n", len);
		printf("delim %c\n", *(write_head+len-1));
		write_head+=len;
		// printf("a is %hhx\n", 'a');
		// for (int i=0; i<len; i++){
		// 	printf("%hhx, %c\n", encode_buffer[i], encode_buffer[i]);
		// }
	}
	
	// int len = encode(test1, strlen(test1), encode_buffer, 1000, 'a');
	int remaining = 10000;
	int remaining_decode = 10000;
	write_head = decode_buffer;
	uint8_t* read_head = encode_buffer;
	int read = 0;
	printf("all: %s\n", encode_buffer);
	while (remaining>0){
		int written;
		int read_bytes;
		printf("frame %d: %p, %p\n", read, write_head, read_head);
		int len2 = cobs_stream_decode_buf(read_head, remaining, write_head, remaining_decode-1, 'a', &written, &read_bytes);
		printf("Read: %d\n", len2);
		if (len2 == -3 || len2 == -2){
			break;
		}
		write_head[written] = '\0';
		printf("%.*s\n", len2, encode_buffer);
		printf("Wrote: %d\n", written);	
		printf("Result: %d\n", strcmp(test1, write_head));
		printf("start: %p\n", write_head);
		printf("%s\n", write_head);
		remaining-=len2;
		remaining_decode-=written;
		write_head+=written;
		read_head+=len2;
		read++;
		printf("n:%s\n", read_head);
		if (read == 6){
			break;
		}
	}
	printf("Found %d frames\n", read);
	
	// memcpy(encode_buffer, test1, strlen(test1));
	// printf("a is %hhx\n", 'a');
	// for (int i=0; i<len; i++){
	// 	printf("%hhx\n", encode_buffer[i]);
		
	// }
}

void test_varint(){
	uint32_t i = 1243234234;
	uint8_t buf[5];
	int bytes_used = encode_varint(i, buf);
	printf("Used %d\n", bytes_used);
	for (int i =0; i< 5;i++){
		printf("%d\n", buf[i]);
	}
	uint32_t num;
	varint_decode_state_t state = decode_varint(buf, &num);
	switch (state)
	{
	case VARINT_OK:
		printf("ok\n");
		break;
	case VARINT_TOO_BIG:
		printf("too big\n");
		break;
	}
	printf("%d\n", num);
	if (i==num){
		printf("decoded correctly\n");
	}

}

int main(){
	// test_varint();
	test_cobs()
}