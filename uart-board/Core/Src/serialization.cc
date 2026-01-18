
#include "serialization.h"
#include "ArduinoJson.h"
#include "ArduinoJson/Document/JsonDocument.hpp"
#include "ArduinoJson/Json/JsonSerializer.hpp"
#include "class/cdc/cdc_device.h"

extern "C" {
void serialize(uint8_t *output, size_t output_len, const char *topic,
               uint8_t *msg, size_t msg_len) {
  JsonDocument doc;
  doc["topic"] = topic;
  doc["message"] = msg;
  serializeJson(doc, (void *)output, output_len);
}

void send_msg(const char *topic, uint8_t *msg, size_t msg_len) {
  uint8_t json[128] = {0};
  serialize(json, 128, topic, msg, msg_len);

  tud_cdc_n_write(0, json, 128);
  // flush msg
  memset(msg, 0, msg_len);
}
}