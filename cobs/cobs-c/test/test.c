#include <string.h>
#include <stdio.h>
#include "cobs.h"

uint8_t encode_buffer[10000];
uint8_t decode_buffer[10000];


int main(){
	char* test1 = "ab123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890bcdefghaasdfasfaf";
	// char* test1 = "abcdefghaasdfasfaf";
	uint8_t* write_head = encode_buffer;
	printf("Og len: %d\n", strlen(test1));
	for (int i=0;i<5;i++){
		int slen = strlen(test1);
		printf("estimate: %d \n", estimate_encoded_size(slen));
		int len = encode(test1, slen, write_head, 1000, 'a');
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
		
		printf("frame %d: %p, %p\n", read, write_head, read_head);
		int len2 = decode(read_head, remaining, write_head, remaining_decode-1, 'a', &written);
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