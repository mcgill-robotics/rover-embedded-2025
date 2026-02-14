#ifndef SERIALIZATION2_H
#define SERIALIZATION2_H

#ifdef __cplusplus
extern "C" {
#endif

void deserialize(char *topic_buf, size_t topic_len, uint8_t *msg_buf, size_t msg_len, char *json);

#ifdef __cplusplus
}
#endif

#endif