#include "ArduinoJson.h"
#include "ArduinoJson/Document/JsonDocument.hpp"
#include "ArduinoJson/Json/JsonDeserializer.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include "ArduinoJson/MsgPack/MsgPackDeserializer.hpp"
#include "buffers.h"
#include "rosjam.h"
#include <cstddef>
#include "deserialization.h"


extern "C" {


	RosjamEndpoint* deserialize(ActiveEndpoints* endpoints, char *json, char* message_buf, int msg_buf_len) {
    	JsonDocument doc;
		DeserializationResult result = {
			.endpoint = NULL,
			.message = NULL
		};
		deserializeMsgPack(doc, json);
		// Match endpoint
		const char *topic = doc["topic"];
		RosjamEndpoint* endpoint = NULL;
		for (int i = 0; i<endpoints->size; i++){
			RosjamEndpoint* endpoint_to_try = (endpoints->endpoints)[i];
			if (strcmp(topic,endpoint_to_try->topic)==0){
				endpoint = endpoint_to_try;
				break;
			}
		}
		if (endpoint == NULL){
			return endpoint;
		}
		// Copy message into buffer
		const char *msg = doc["message"];
		int message_len = strlen(msg)+1;
		if (message_len > msg_buf_len){
			return NULL;
		}
		// uint8_t* write_head = get_write_space(&(endpoint->rx_buf), message_len);

		// if (write_head != NULL){
		memcpy(message_buf, msg, message_len);
			// result.message = (char*) write_head;
			// result.endpoint = endpoint;
		// }
		return endpoint;
	}
}