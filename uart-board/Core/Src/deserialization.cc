
#include "ArduinoJson.h"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include "deserialization.h"

extern "C" {
	void deserialize(char* output, int maxLen, char* sensor, long time, double latitude, double longitude){

		JsonDocument doc;
		

		doc["sensor"] = sensor;
		doc["time"] = maxLen;
		doc["data"].add(latitude);
		doc["data"].add(longitude);
		serializeJson(doc, output, maxLen);
	}
}