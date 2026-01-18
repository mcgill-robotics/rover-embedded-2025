
#include "ArduinoJson.h"
#include "ArduinoJson/Document/JsonDocument.hpp"
#include "ArduinoJson/Json/JsonDeserializer.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include <cstring>
#include "deserialization.h"

extern "C" {
	void deserialize(char *topic_buf, size_t topic_len, uint8_t *msg_buf, size_t msg_len, char *json) {
		JsonDocument doc;
		deserializeJson(doc, json);
		const char *topic = doc["topic"];
		memcpy(topic_buf, topic, topic_len);

		const char *msg = doc["message"];
		memcpy(msg_buf, msg, msg_len);
	}
}