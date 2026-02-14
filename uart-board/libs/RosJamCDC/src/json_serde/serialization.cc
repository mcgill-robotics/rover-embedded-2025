#include "main.h"
#include "serialization.h"
#include "ArduinoJson.h"
#include "ArduinoJson/Document/JsonDocument.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include "class/cdc/cdc_device.h"
#include <stdint.h>

extern "C" {
size_t serialize(uint8_t *output, size_t output_len, const char *topic,
               uint8_t *msg) {
  JsonDocument doc;
  doc["topic"] = topic;
  doc["message"] = msg;
  return serializeJson(doc, (void *)output, output_len);
}

void send_msg(const char *topic, uint8_t *msg) {
  uint8_t json[JSON_BUF_LEN];
  uint8_t* json_ptr = json;
  size_t json_size = serialize(json, JSON_BUF_LEN, topic, msg);

  uint32_t to_write = json_size;
  while (to_write > 0) {
      uint32_t written = tud_cdc_n_write(0, json_ptr, to_write);
      to_write -= written;
      json_ptr += written;
  }
  
  tud_cdc_n_write(0, "\n", 1);
  tud_cdc_n_write_flush(0);
}
}