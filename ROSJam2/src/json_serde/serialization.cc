#include "serialization.h"
#include "ArduinoJson.h"
#include "ArduinoJson/Document/JsonDocument.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include "class/cdc/cdc_device.h"
#include "rosjam.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdint.h>

extern "C" {
    
size_t serialize(Buffer* buffer, const char *topic, uint8_t *msg) {
    JsonDocument doc;
    doc["topic"] = topic;
    doc["message"] = msg;
    int json_size = measureJson(doc);
    if (json_size == 0){
        return 0;
    }
    // update buffer size with json size
    uint8_t* write_head = get_write_space(buffer, json_size+1); // +1 and newline
    if (write_head != NULL){
        int size = serializeJson(doc, (void*) write_head, json_size)+2;
        *(write_head+json_size) = '\n';
        // *(write_head+json_size+1) = '\0';
        return size;
    } else {
        return 0;
    }
}

}