
#include "ArduinoJson.h"
#include "serialization.h"

extern "C" {
	void serialize(char* json){

		JsonDocument doc;
		deserializeJson(doc, json);

		const char* sensor = doc["sensor"];
		long time          = doc["time"];
		double latitude    = doc["data"][0];
		double longitude   = doc["data"][1];
	}
}