#include "cobs.h"
#include "default_rosjam_config.h"
#include "rosjam.h"
#include "rosjam_internal.h"
#include "buffers.h"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "json_serde/deserialization.h"
#include "json_serde/serialization.h"
#include "stm32g4xx_hal_def.h"
#include "tusb_config.h"
#include "tusb.h"
#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mpack.h"
// #include "cobs.h"

ActiveEndpoints currentEndpoints;
cobs_reader_t cobs_reader;
int mid_cobs_frame = 0;

int drop_dangling = 0;

double string_to_float(char* string){
	return atof(string);
}

int float_to_string(double number, int precision, char* buf, int buf_len){
    int len = snprintf(NULL, 0, "%.*lf", precision, number);
	if (len +1 > buf_len){
		return -1;
	}
    snprintf(buf, len + 1, "%.*lf", precision, number);
	return len+1;
}

int int_to_string(int number, char* buf, int buf_len){
    int len = snprintf(NULL, 0, "%d", number);
	if (len +1 > buf_len){
		return -1;
	}
    snprintf(buf, len + 1, "%d", number);
	return len+1;
}

void setup_simple(){
	tusb_rhport_init_t dev_init = {
    	.role = TUSB_ROLE_DEVICE,
    	.speed = TUSB_SPEED_AUTO
  	};
  	tusb_init(BOARD_TUD_RHPORT, &dev_init);
}

void print_to_usb(char* message){
	send_msg_raw(message,strlen(message));
}

void send_msg_raw(char *message, int message_len){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		tud_cdc_n_write(USB_CDC_ITF, message, message_len);
		tud_cdc_n_write_flush(USB_CDC_ITF);
	}
}

int has_data(){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		int available = tud_cdc_n_available(USB_CDC_ITF);
		return available > 0;
	}
	return 0;
}

char read_char(){
	if (has_data()){
		return tud_cdc_n_read_char(USB_CDC_ITF);
	}
	return -1;
}


void process_simple(){
	tud_cdc_n_write_flush(USB_CDC_ITF); //make sure small messages get immediately sent
	tud_task();
}


uint8_t temp_data_buf[ENDPOINT_BUF_LEN];
uint8_t temp_frame_buf[ENDPOINT_BUF_LEN];
uint8_t global_rx_buf[ENDPOINT_BUF_LEN];

void reset_cobs_reader(){
	cobs_setup_stream_reader(&cobs_reader);
	mid_cobs_frame = 0;
}

void init_rosjam_usb(){
	currentEndpoints.global_rx_buffer.buf = global_rx_buf;
	currentEndpoints.global_rx_buffer.capacity = ENDPOINT_BUF_LEN;
	currentEndpoints.global_rx_buffer.read_offset = 0;
	currentEndpoints.global_rx_buffer.size = 0;
	currentEndpoints.nextTxEndpoint = 0;
	currentEndpoints.size = 0;
	currentEndpoints.first_message = 1;
	currentEndpoints.hasPending = 0;
	currentEndpoints.diag = NULL;
	tusb_rhport_init_t dev_init = {
    	.role = TUSB_ROLE_DEVICE,
    	.speed = TUSB_SPEED_AUTO
  	};
  	tusb_init(BOARD_TUD_RHPORT, &dev_init);
	reset_cobs_reader();
}

/**
	Set diag channel for loopback use
*/
void set_diag_endpoint(RosjamEndpoint* diag){
	currentEndpoints.diag = diag;
}

void init_interface(RosjamEndpoint* endpoint, const char* topic, uint8_t* buffer, int buffer_size){
	endpoint->topic = topic;
	(endpoint->tx_buf).capacity = buffer_size;
	(endpoint->tx_buf).size = 0;
	(endpoint->tx_buf).read_offset = 0;
	(endpoint->tx_buf).buf = buffer;
}

/**
	Registers the interface for use. Returns 1 on success, 0 on failure if not enough endpoints were declared at compile time
*/
int register_interface(RosjamEndpoint* endpoint){
	if (currentEndpoints.size<ENDPOINT_COUNT){
		// Add to array
		currentEndpoints.endpoints[currentEndpoints.size] = endpoint;
		currentEndpoints.size++;
		return 1;
	}
	return 0;
}


/**

Convenience function to send a string as data

*/
void send_msg(RosjamEndpoint* endpoint, char* message){
	mpack_writer_t writer;
	mpack_writer_init(&writer, (char*) temp_data_buf, ENDPOINT_BUF_LEN);
	mpack_write_cstr(&writer, message);
	int written = mpack_writer_buffer_used(&writer);
    if (mpack_writer_destroy(&writer) != mpack_ok) {
        fprintf(stderr, "An error occurred encoding the data!\n");
        return;
    }
	send_data(endpoint, (uint8_t*) temp_data_buf, written);
}

/**

Call this to prepare a message to be sent on an endpoint

*/
void send_data(RosjamEndpoint* endpoint, uint8_t* data, int data_len){
	currentEndpoints.hasPending = 1;
	Buffer* buffer = &(endpoint->tx_buf);
	int json_size = serialize(temp_frame_buf, ENDPOINT_BUF_LEN, endpoint->topic, (uint8_t*) data, data_len);
	int cobs_size = cobs_estimate_encoded_size(json_size);
	uint8_t* write_head;
	buffer_alloc_state_t alloc_state = get_tagged_write_space(buffer, cobs_size, &write_head);
	if (write_head != NULL && alloc_state != BUFFER_TOO_SMALL){
		cobs_encode(temp_frame_buf, json_size, write_head, cobs_size, 0);
	}
}

/**

Call in a loop to schedule sending messages to tinyusb

*/
int send_next_messages(){
	if (!tud_cdc_n_ready(USB_CDC_ITF)){
		return 0;
	}

	if (!currentEndpoints.hasPending){
		return 0;
	}
	// tud_cdc_n_write_clear(USB_CDC_ITF);
	int can_send = tud_cdc_n_write_available(USB_CDC_ITF);
	if (currentEndpoints.first_message && can_send>1){
		char buf[1];
		buf[0] = 0;
		tud_cdc_n_write(USB_CDC_ITF, buf, 1);
		currentEndpoints.first_message = 0;
		can_send-=1;
		// return 0;
	} else if (currentEndpoints.first_message){
		return 0;
	}

	
	int str_size;
	uint8_t* start;
	int empty_count = 0;
	// Find next non empty endpoint in round robin fashion
	int first_endpoint_to_try = currentEndpoints.nextTxEndpoint;
	int next_endpoint_idx = first_endpoint_to_try;
	Buffer* buf = &(currentEndpoints.endpoints[next_endpoint_idx] -> tx_buf);
	int total_sent = 0;
	int msg_bytes_actual = get_first_message(buf, &str_size, &start);
	int counter = 0;
	while (empty_count<currentEndpoints.size){
		if (msg_bytes_actual == 0){
			empty_count++;

		} else {
			empty_count = 0;
			
			if (str_size>CFG_TUD_CDC_TX_BUFSIZE){
				tud_cdc_n_write_flush(USB_CDC_ITF); // flush buffer
				int bytes_sent = 0;
				int bytes_to_send = str_size;
				while (bytes_to_send>0){
					int sent = tud_cdc_n_write(USB_CDC_ITF, start+bytes_sent, bytes_to_send);
					tud_cdc_n_write_flush(USB_CDC_ITF);
					bytes_sent += sent;
					bytes_to_send-=bytes_sent;
				}
				mark_read(buf, msg_bytes_actual);
				currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size;
				counter+=1;
				return total_sent;
			} else if (str_size>can_send){
				// Message larger than available (wait till flushed and space becomes available)
				tud_cdc_n_write_clear(USB_CDC_ITF);
				break;
			}
			if (currentEndpoints.endpoints[next_endpoint_idx]==currentEndpoints.diag){
					// led_state=!led_state;
					printf("Sent message from diag\n");
			}
			total_sent+=str_size;
			// send as many complete messages as possible
			tud_cdc_n_write(USB_CDC_ITF, start, str_size);
			can_send-=str_size;
			mark_read(buf, msg_bytes_actual);		
			counter+=1;	
			
		}

		// move on to next interface
		currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size; // try next
		next_endpoint_idx = currentEndpoints.nextTxEndpoint;
		buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);
		msg_bytes_actual = get_first_message(buf, &str_size, &start);
	}
	printf("Sent %d\n", counter);
	currentEndpoints.hasPending = 0;
	return total_sent;
}

__weak void receivedFromUSB(RosjamEndpoint *endpoint, char *message, int message_len){
	if (currentEndpoints.diag != NULL){
		char message_string[512];
		mpack_reader_t reader;
		mpack_reader_init_data(&reader, message, message_len);
		mpack_expect_cstr(&reader, message_string, 512);
		if (mpack_reader_destroy(&reader) != mpack_ok) {
			fprintf(stderr, "An error occurred decoding the data!\n");
			return;
		}
		// if (endpoint == currentEndpoints.diag){
		// 	led_state = !led_state;
		// }
		if (strcmp(endpoint->topic, "diag0")==0){
			led_state = !led_state;
			printf("%s\n",message_string);
			send_msg(currentEndpoints.diag, message_string);
		}
	}
}

void check_rx_next(){
	if (!tud_cdc_n_ready(USB_CDC_ITF)){
		return;
	}
	int available = tud_cdc_n_available(USB_CDC_ITF);
	if (available == 0){
		return;
	}
	// uint8_t temp[100];
	// int read_data_count = tud_cdc_n_read(USB_CDC_ITF, temp, 100);
	int output_size_estimate = cobs_estimate_decoded_size(available)+1;
	uint8_t* write_head;
	Buffer* buffer = &currentEndpoints.global_rx_buffer;
	buffer_alloc_state_t alloc_state = get_write_space(buffer, output_size_estimate, &write_head);
	if (alloc_state == BUFFER_TOO_SMALL){
		available = buffer->capacity-buffer->size;
		alloc_state = get_write_space(buffer, available, &write_head);
	} else if (alloc_state == BUFFER_CLEARED){
		if (mid_cobs_frame){
			reset_cobs_reader();
		}
	} else {
		printf("hello");
	}
	while (true) {
		int written_bytes;
		int read_bytes;
		// memset(write_head, 0, output_size_estimate);
		cobs_result_t decode_result =  cobs_stream_decode(&cobs_reader, &tud_cdc_n_read, USB_CDC_ITF, available, write_head, output_size_estimate, 0, &written_bytes, &read_bytes);
		if (decode_result != COBS_DONE){
			reclaim_allocated(buffer, output_size_estimate-written_bytes);
		}
		 if (decode_result == COBS_INCOMPLETE_FRAME){
			mid_cobs_frame = 1;
			break;
		} else if (decode_result == COBS_NO_FRAME){
			mid_cobs_frame = 0;	
			break;
		} else if (decode_result==COBS_OUTPUT_FULL){
			break;
		}
		
		if (decode_result == COBS_DONE){
			mid_cobs_frame = 0;
			// decode data here
			int data_size;
			
			RosjamEndpoint* endpoint = deserialize(&currentEndpoints, (char*)buffer->buf+buffer->read_offset,  cobs_reader.current_frame_size, (char*)temp_data_buf, ENDPOINT_BUF_LEN, &data_size);
			if (endpoint != NULL){
				// call callback for user to process
				receivedFromUSB(endpoint, (char*)temp_data_buf, data_size);
			}
			mark_read(buffer, cobs_reader.current_frame_size);
			cobs_consume_message(&cobs_reader);

		}
		available-=read_bytes;
		output_size_estimate-=written_bytes;
		write_head+=written_bytes;
	}
}


void check_rx(){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		int available = tud_cdc_n_available(USB_CDC_ITF);
		if (available == 0){
			return;
		}
		
		
		//determine how many bytes to read in this operation
		int to_read = available;
		Buffer* buffer = &currentEndpoints.global_rx_buffer;
		if (available > buffer->capacity){
			to_read = buffer -> capacity;
		}
		// read data into rx buffer
		uint8_t* write_head;
		buffer_alloc_state_t alloc_state = get_write_space(buffer, to_read, &write_head);
		if (write_head != NULL && alloc_state != BUFFER_TOO_SMALL){
			tud_cdc_n_read(USB_CDC_ITF, write_head, to_read);
		}
		do {
			int written_bytes;
			int in_buffer = buffer->size;
			memset(temp_frame_buf, 0, ENDPOINT_BUF_LEN); // investigate why this is needed
			int read_bytes = cobs_decode(buffer->buf+buffer->read_offset, in_buffer, temp_frame_buf, ENDPOINT_BUF_LEN, 0, &written_bytes);
			if (read_bytes == -2){
				// not enough data so throw all away
				mark_read(buffer, in_buffer);
				// skip 
				return;
			} else if (read_bytes == -1){
				// message too big for output buffer
				mark_read(buffer, ENDPOINT_BUF_LEN); // 2048 is size of cobs_decode_buf because it was filled
				continue;
			} else if (read_bytes == -3) {
				break;
			}

			
			int data_size;
			RosjamEndpoint* endpoint = deserialize(&currentEndpoints, (char*) temp_frame_buf,  written_bytes-1, (char*)temp_data_buf, ENDPOINT_BUF_LEN, &data_size);
			if (endpoint != NULL){
				// call callback for user to process
				receivedFromUSB(endpoint, (char*)temp_data_buf, data_size);
			}
			mark_read(buffer, read_bytes);
			
		} while (buffer->size>0);
	}
}

void process(){
	check_rx();
	// check_rx_next();
	send_next_messages();
	tud_cdc_n_write_flush(USB_CDC_ITF); //make sure small messages get immediately sent
	tud_task();
}