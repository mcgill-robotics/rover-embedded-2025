#include "rosjam.h"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "json_serde/deserialization.h"
#include "json_serde/serialization.h"
#include "tusb_config.h"
#include "tusb.h"
#include "stdio.h"

ActiveEndpoints currentEndpoints;

int drop_dangling = 0;

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

uint32_t get_size_with_pad(uint32_t size){
	// ensure alignment
	return size+(4-(size%4))%4;
	
}

int get_first_message(Buffer* buf, int* str_size, uint8_t** start){
	if (buf->read_offset < buf->size){
		int offset = buf ->read_offset;
		uint8_t* message_start = buf->buf+offset;
		*str_size = ((uint32_t*) message_start)[0];
		*start = message_start+4;
		int size_with_padding = get_size_with_pad(*str_size);
		return size_with_padding+4;
	}
	return 0;
	
}

void mark_read(Buffer* buf, int read){
	(buf ->read_offset)+=read;
}

void mark_read_global(RosjamRxBuffer* buf, int read){
	(buf ->read_offset)+=read;
}

uint8_t* get_write_space(Buffer* buf, uint32_t size){
	uint32_t metadata_size = 4; //+4 for metadata about string size
	uint32_t size_with_padding = get_size_with_pad(size);
	uint32_t to_reserve = size_with_padding+metadata_size;
	
	if (size > buf->capacity){
		return NULL;
	}
	int available = buf->capacity - buf->size;
	uint8_t* write_position;
	if (available < to_reserve){
		// drop everything currently in buffer
		buf-> read_offset = 0;
		buf -> size = to_reserve;
		write_position = buf -> buf;
	} else {
		write_position = buf -> buf+buf -> size;
		buf -> size += to_reserve;
	}
	((uint32_t*) write_position)[0] = size;
	write_position = write_position+metadata_size;
	return write_position;
}


uint8_t* get_write_space_global(RosjamRxBuffer* buf, int size){
	if (size > buf->capacity){
		return NULL;
	}
	int available = buf->capacity - buf->size;
	if (available < size){
		// drop everything currently in buffer
		buf-> read_offset = 0;
		buf -> size = size;
		return buf -> buf;
	} else {
		uint8_t* write_position = buf -> buf+buf -> size;
		buf -> size += size;
		return write_position;
	}
}

int add_interface(RosjamEndpoint* endpoint, const char* topic){
	if (currentEndpoints.size<ENDPOINT_COUNT){
		// initialize endpoint fields
		//tx buffer
		(endpoint->rx_buf).capacity = ENDPOINT_BUF_LEN;
		(endpoint->rx_buf).size = 0;
		(endpoint->rx_buf).read_offset = 0;
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
	Buffer* buffer = &(endpoint -> tx_buf);
	serialize(buffer, endpoint->topic, (uint8_t*) message); // serialize into tx buffer directly
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
			// message sent so we can move to next available interface
			currentEndpoints.nextTxEndpoint = (next_endpoint_idx+1)%currentEndpoints.size;
			next_endpoint_idx = currentEndpoints.nextTxEndpoint;
			buf = &(currentEndpoints.endpoints[next_endpoint_idx]->tx_buf);
			msg_bytes_actual = get_first_message(buf, &str_size, &start);
			
		}
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
	printf("Received: topic: %d, msg: %d\n", endpoint->topic, message);
}


void check_rx(){
	if (tud_cdc_n_ready(USB_CDC_ITF)){
		int available = tud_cdc_n_available(USB_CDC_ITF);
		if (available == 0){
			return;
		}

		//determine how many bytes to read in this operation
		int to_read = available;
		RosjamRxBuffer* buffer = &currentEndpoints.global_rx_buffer;
		if (available > buffer->capacity){
			to_read = buffer -> capacity;
		}

		// read data into rx buffer
		uint8_t* write_head = get_write_space_global(buffer, to_read+1);
		if (write_head != NULL){
			tud_cdc_n_read(USB_CDC_ITF, write_head, to_read);
		}
		*(write_head+to_read) = '\0';

		// find json packet boundary
		uint8_t* read_head = buffer->buf+buffer->read_offset;
		char* newline_pos = strchr((char*)read_head, '\n');

		if (newline_pos != NULL) {
			if (drop_dangling){
				drop_dangling = 0;
				mark_read_global(buffer, (int)(newline_pos-(char*)read_head));
				// parse rest on next invocation
				return;
			}
			*newline_pos = '\0'; //terminate string for deserialize
			DeserializationResult result = deserialize(&currentEndpoints, (char*) read_head);
			if (result.endpoint != NULL){
				// call callback for user to process
				receivedFromUSB(result.endpoint, result.message);
			}
		} else {
			if (buffer->size == buffer->capacity){
				//buffer full and message cannot fit
				buffer->size = 0; // clear buffer
				buffer->read_offset = 0;
				// on future reads, discard message until done
				drop_dangling = 1;
			}
		}

	}
}

void process(){
	tud_task();
	check_rx();
	send_next_messages();
}