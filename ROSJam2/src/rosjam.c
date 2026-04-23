#include "cobs.h"
#include "default_rosjam_config.h"
#include "rosjam.h"
#include "rosjam_internal.h"
#include "buffers.h"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "json_serde/deserialization.h"
#include "json_serde/serialization.h"
#include "tusb_config.h"
#include "tusb.h"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
// #include "cobs.h"

ActiveEndpoints currentEndpoints;

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

void setup(){
	currentEndpoints.global_rx_buffer.capacity = ENDPOINT_BUF_LEN*ENDPOINT_COUNT;
	currentEndpoints.global_rx_buffer.read_offset = 0;
	currentEndpoints.global_rx_buffer.size = 0;
	currentEndpoints.nextTxEndpoint = 0;
	currentEndpoints.size = 0;

	tusb_rhport_init_t dev_init = {
    	.role = TUSB_ROLE_DEVICE,
    	.speed = TUSB_SPEED_AUTO
  	};
  	tusb_init(BOARD_TUD_RHPORT, &dev_init);
}


int add_interface(RosjamEndpoint* endpoint, const char* topic){
	if (currentEndpoints.size<ENDPOINT_COUNT){
		// initialize endpoint fields
		//tx buffer
		// (endpoint->rx_buf).capacity = ENDPOINT_BUF_LEN;
		// (endpoint->rx_buf).size = 0;
		// (endpoint->rx_buf).read_offset = 0;
		//rx buffer
		(endpoint->tx_buf).capacity = ENDPOINT_BUF_LEN;
		(endpoint->tx_buf).size = 0;
		(endpoint->tx_buf).read_offset = 0;

		endpoint->topic = topic;

		// Add to array
		currentEndpoints.endpoints[currentEndpoints.size] = endpoint;
		currentEndpoints.size++;
		return 1;
	}
	return 0;
}


/**

Call this to prepare a message to be sent on an endpoint

*/
void send_msg(RosjamEndpoint* endpoint, char* message){
	uint8_t temp_buf[2048];
	Buffer* buffer = &(endpoint -> tx_buf);
	printf("Serialization start\n");
	int json_size = serialize(temp_buf, 512, endpoint->topic, (uint8_t*) message); // serialize into tx buffer directly
	int cobs_size = estimate_encoded_size(json_size);
	uint8_t* write_head = get_write_space(buffer, cobs_size);
	printf("Sizes: %d, %d\n", json_size, cobs_size);
	if (write_head != NULL){
		int enc_status = encode(temp_buf, json_size, write_head, cobs_size, 0);
		printf("Wrote: %d\n", enc_status);
	}
}

/**

Call in a loop to schedule sending messages to tinyusb

*/
void send_next_messages(){
	if (!tud_cdc_n_ready(USB_CDC_ITF)){
		return;
	}
	// Find next non empty endpoint in round robin fashion
	// int first_endpoint_to_try = currentEndpoints.nextTxEndpoint;
	// int next_endpoint_idx = first_endpoint_to_try;
	// Buffer* buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);

	// while (buf->size == 0){
	// 	next_endpoint_idx=(next_endpoint_idx+1)%currentEndpoints.size;
	// 	// Came back to first tried endpoint
	// 	if (next_endpoint_idx==first_endpoint_to_try){
			
	// 		return;
	// 	}
	// 	buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);
	// }
	// // update to current endpoint if needs multiple tries
	// currentEndpoints.nextTxEndpoint = next_endpoint_idx;

	// compute how many message can be sent and size in bytes
	// or find size of message if larger than available
	// int sent_count = 0;
	int can_send = tud_cdc_n_write_available(USB_CDC_ITF);
	// printf("Can send %d\n", can_send);
	// int bytes_to_send = 0;
	// int i = 0;
	// while ((i<can_send || sent_count==0)&&buf->read_offset+i<buf->size){
	// 	char current_char = (buf->buf+buf->read_offset)[i];
	// 	if (current_char == '\n'){
	// 		bytes_to_send = i+1; //+1 because i starts at 0
	// 		if (i<can_send){
	// 			sent_count++;
	// 		} else {
	// 			sent_count = 1;
	// 		}
	// 	}
	// 	i++;
	// }

	
	int str_size;
	uint8_t* start;
	int empty_count = 0;
	// Find next non empty endpoint in round robin fashion
	int first_endpoint_to_try = currentEndpoints.nextTxEndpoint;
	int next_endpoint_idx = first_endpoint_to_try;
	Buffer* buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);

	int msg_bytes_actual = get_first_message(buf, &str_size, &start);

	while (empty_count<currentEndpoints.size){
		if (msg_bytes_actual == 0){
			// printf("Empty send %d, %d\n", currentEndpoints.nextTxEndpoint, empty_count);
			empty_count++;

		} else {
			empty_count = 0;

			if (str_size>CFG_TUD_CDC_TX_BUFSIZE){
				tud_cdc_n_write_flush(USB_CDC_ITF); // flush buffer
				int bytes_sent = 0;
				int bytes_to_send = str_size;
				while (bytes_to_send>0){
					int sent = tud_cdc_n_write(USB_CDC_ITF, start, bytes_to_send);
					tud_cdc_n_write_flush(USB_CDC_ITF);
					bytes_sent += sent;
					bytes_to_send-=bytes_sent;
				}
				mark_read(buf, msg_bytes_actual);
				currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size;
				return;
			} else if (str_size>can_send){
				// Message larger than available (wait till flushed and space becomes available)
				return;
			}
			
			// send as many complete messages as possible
			tud_cdc_n_write(USB_CDC_ITF, start, str_size);
			can_send-=str_size;
			mark_read(buf, msg_bytes_actual);			
		}

		// move on to next interface
		currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size; // try next
		next_endpoint_idx = currentEndpoints.nextTxEndpoint;
		buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);
		msg_bytes_actual = get_first_message(buf, &str_size, &start);
	}
	// if (sent_count == 0){
	// 	// if message too big for one transfer try immediately sending it
	// 	if (bytes_to_send>CFG_TUD_CDC_TX_BUFSIZE){
	// 		tud_cdc_n_write_flush(USB_CDC_ITF); // flush buffer
	// 		int bytes_sent = 0;
	// 		while (bytes_to_send>0){
	// 			int sent = tud_cdc_n_write(USB_CDC_ITF, buf->buf+buf->read_offset+bytes_sent, bytes_to_send);
	// 			tud_cdc_n_write_flush(USB_CDC_ITF);
	// 			bytes_sent += sent;
	// 			bytes_to_send-=bytes_sent;
	// 		}
	// 		mark_read(buf, bytes_sent);
	// 	} else {
	// 		// Message larger than available (wait till flushed and space becomes available)
	// 		return;
	// 	}
	// } else {
	// 	// send as many complete messages as possible
	// 	tud_cdc_n_write(USB_CDC_ITF, buf->buf, bytes_to_send);
	// 	mark_read(buf, bytes_to_send);
	// }
	// // message sent so we can move to next available interface
	// currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size;
}

__weak void receivedFromUSB(RosjamEndpoint *endpoint, char *message){
	printf("Received: topic: %s, msg: %s\n", endpoint->topic, message);
	send_msg(endpoint, message);
	printf("Echo message\n");
}


void check_rx(){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		int available = tud_cdc_n_available(USB_CDC_ITF);
		if (available == 0){
			// printf("Not available\n");
			return;
		}
		printf("Checking rx\n");
		//determine how many bytes to read in this operation
		int to_read = available;
		RosjamRxBuffer* buffer = &currentEndpoints.global_rx_buffer;
		if (available > buffer->capacity){
			to_read = buffer -> capacity;
		}
		printf("To read: %d\n", to_read);
		// read data into rx buffer
		uint8_t* write_head = get_write_space_global(buffer, to_read);
		printf("Writehead: %p", write_head);
		if (write_head != NULL){
			
			int read = tud_cdc_n_read(USB_CDC_ITF, write_head, to_read);
			printf("Reading %d\n", read);
		}
		
		//{"topic":"uart0","message":"hello"}

		// find json packet boundary
		do {
			// uint8_t* read_head = buffer->buf+buffer->read_offset;
			// int in_buffer = buffer->size-buffer->read_offset;
			// printf("read_head: %p\n", read_head);
			// // find newline
			// int newline_pos = 0;
			// for (int i=0;i<in_buffer;i++){
			// 	if (read_head[i]=='\n'){
			// 		newline_pos = i;
			// 		break;
			// 	}
			// }
			// char* newline_pos = strchr((char*)read_head, '\n');
			uint8_t cobs_decode_buf[2048];
			int written_bytes;
			int in_buffer = buffer->size-buffer->read_offset;
			int read_bytes = decode(buffer->buf+buffer->read_offset, in_buffer, cobs_decode_buf, 2048, 0, &written_bytes);
			if (read_bytes == -2){
				// printf("Dropping\n");
				// drop_dangling = 0;
				mark_read_global(buffer, in_buffer);
				// skip 
				return;
			}
			if (read_bytes > 0) {
			// if (newline_pos != 0) {
				// if (drop_dangling){
				// 	printf("Dropping\n");
				// 	drop_dangling = 0;
				// 	mark_read_global(buffer, newline_pos+1);
				// 	// parse rest on next invocation
				// 	continue;
				// }
				// read_head[newline_pos] = '\0'; //terminate string for deserialize
				// decode cobs
				uint8_t msg_pack_decode_buf[2048];
				RosjamEndpoint* endpoint = deserialize(&currentEndpoints, (char*) cobs_decode_buf, (char*)msg_pack_decode_buf, 2048);
				if (endpoint != NULL){
					// call callback for user to process
					receivedFromUSB(endpoint, (char*)msg_pack_decode_buf);
				}
				mark_read_global(buffer, read_bytes);
				// mark_read_global(buffer, newline_pos+1);
				// printf("Marked %d as read\n", newline_pos+1);
			} else {
				if (buffer->size == buffer->capacity){
					printf("Full\n");
					//buffer full and message cannot fit
					// buffer->size = 0; // clear buffer
					// buffer->read_offset = 0;
					// on future reads, discard message until done
					drop_dangling = 1;
					return;
				} else {
					printf("No new line\n");
					break;
				}
			}
		} while (buffer->read_offset<buffer->size);
	}
}

void process(){
	
	check_rx();
	send_next_messages();
	tud_cdc_n_write_flush(USB_CDC_ITF); //make sure small messages get immediately sent
	tud_task();
}