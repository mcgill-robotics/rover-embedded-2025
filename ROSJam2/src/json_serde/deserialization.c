
#include "buffers.h"
#include "mpack.h"
#include "rosjam.h"
#include "deserialization.h"


// extern "C" {
	enum key_names       {
		KEY_TOPIC, 
		KEY_DATA,
		KEY_COUNT
	};
	const char* keys[] = {"topic"  , "data"  };

	RosjamEndpoint* deserialize(ActiveEndpoints* endpoints, char *json, int json_buf_len, char* message_buf, int msg_buf_len, int* read_message_len) {

		
		mpack_reader_t reader;
		bool found[KEY_COUNT] = {0};
		uint8_t topic[255];
		mpack_reader_init_data(&reader, json, json_buf_len);
		mpack_expect_map_max(&reader, KEY_COUNT);
		mpack_expect_cstr_match(&reader, "topic");
		mpack_expect_cstr(&reader, (char*) topic, 255);
		mpack_expect_cstr_match(&reader, "data");
		*read_message_len = mpack_expect_bin_buf(&reader, message_buf, msg_buf_len);
		// size_t i = mpack_expect_map_max(&reader, KEY_COUNT); // critical check!
		// for (; i > 0 && mpack_reader_error(&reader) == mpack_ok; --i) { // critical check!
		// 	switch (mpack_expect_key_cstr(&reader, keys, found, KEY_COUNT)) {
		// 		case KEY_TOPIC: mpack_expect_cstr(&reader, (char*) topic, 255); break;
		// 		case KEY_DATA:  *read_message_len = mpack_expect_bin_buf(&reader, message_buf, msg_buf_len); break;
		// 		default: mpack_discard(&reader); break;
		// 	}
		// }
		
		// compact is not optional
		// if (!found[KEY_TOPIC]){
		// 	mpack_reader_flag_error(&reader, mpack_error_data);
		// 	return NULL;
		// }
			
		// if (!found[KEY_DATA]){
		// 	mpack_reader_flag_error(&reader, mpack_error_data);
		// 	return NULL;
		// }
		mpack_done_map(&reader);
		if (mpack_reader_destroy(&reader) != mpack_ok) {
       		fprintf(stderr, "An error occurred decoding the data!\n");
        	return NULL;
   		}
		RosjamEndpoint* endpoint = NULL;
		for (int i = 0; i<endpoints->size; i++){
			RosjamEndpoint* endpoint_to_try = (endpoints->endpoints)[i];
			if (strcmp((char*)topic,endpoint_to_try->topic)==0){
				endpoint = endpoint_to_try;
				break;
			}
		}
		return endpoint;
		

    	// JsonDocument doc;
		// DeserializationResult result = {
		// 	.endpoint = NULL,
		// 	.message = NULL
		// };
		// deserializeMsgPack(doc, json);
		// // Match endpoint
		// const char *topic = doc["topic"];
		// RosjamEndpoint* endpoint = NULL;
		// for (int i = 0; i<endpoints->size; i++){
		// 	RosjamEndpoint* endpoint_to_try = (endpoints->endpoints)[i];
		// 	if (strcmp(topic,endpoint_to_try->topic)==0){
		// 		endpoint = endpoint_to_try;
		// 		break;
		// 	}
		// }
		// if (endpoint == NULL){
		// 	return endpoint;
		// }
		// // Copy message into buffer
		// const char *msg = doc["message"];
		// int message_len = strlen(msg)+1;
		// if (message_len > msg_buf_len){
		// 	return NULL;
		// }
		// // uint8_t* write_head = get_write_space(&(endpoint->rx_buf), message_len);

		// // if (write_head != NULL){
		// memcpy(message_buf, msg, message_len);
		// 	// result.message = (char*) write_head;
		// 	// result.endpoint = endpoint;
		// // }
		// return endpoint;
	}
// }