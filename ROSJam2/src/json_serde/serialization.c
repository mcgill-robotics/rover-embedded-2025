#include "serialization.h"
#include "class/cdc/cdc_device.h"
#include "buffers.h"
#include "mpack.h"

// class StaticAllocator:public ArduinoJson::Allocator {
//     char* buffer;
//     int offset;
//     public:
//         StaticAllocator(char* buffer_to_use){
//             buffer = buffer_to_use;
//             offset = 0;
//         }

//         void* allocate(size_t size) override {
//             char* to_allocate = buffer+offset;
//             offset+=size;
//             return to_allocate;
//     }

//     void deallocate(void* pointer) override {
//             ;
//     }

//     void* reallocate(void* ptr, size_t new_size) override {
//         offset+=new_size;
//         return ptr;
//     }
// };

// extern "C" {
    
size_t serialize(uint8_t* temp_buf, int buf_len, const char *topic, uint8_t *data, int data_len) {
    // char buildBuffer[2048];
    // char* head = "{\"topic\":\"";
    // memcpy(buildBuffer, head, strlen(head)+1);
    // strcat(buildBuffer, topic);
    // strcat(buildBuffer, "\",\"message\":\"");
    // strcat(buildBuffer, (char*) msg);
    // strcat(buildBuffer, "\"}");
    // size_t len = strlen(buildBuffer);
    // // update buffer size with json size
    // uint8_t* write_head = get_write_space(buffer, len+1); // +1 and newline
    // if (write_head != NULL){
    //     // int size = serializeJson(doc, (void*) write_head, json_size)+2;
    //     memcpy(write_head, buildBuffer, len);
    //     *(write_head+len) = '\n';
    //     // *(write_head+json_size+1) = '\0';
    //     return len+1;
    // } else {
    //     return 0;
    // }
    // return 0;

    // with arduino json
    // StaticAllocator alloc(buildBuffer);
    // JsonDocument doc(&alloc);

    // old
    // JsonDocument doc;
    // doc["topic"] = topic;
    // doc["message"] = msg;
    // int json_size = measureMsgPack(doc);
    // if (json_size == 0){
    //     return 0;
    // }
    // if (buf_len >= json_size){
    //     int size = serializeMsgPack(doc, (void*) temp_buf, json_size)+1;
    //     *(temp_buf+json_size) = '\n';
    //     // *(write_head+json_size+1) = '\0';
    //     return size;
    // } else {
    //     return 0;
    // }

    mpack_writer_t writer;
    mpack_writer_init(&writer, (char*) temp_buf, buf_len);
    mpack_start_map(&writer, 2);
    mpack_write_cstr(&writer, "topic");
    mpack_write_cstr(&writer, topic);
    mpack_write_cstr(&writer, "data");
    mpack_write_bin(&writer, (char*) data, data_len);
    mpack_finish_map(&writer);
    int written = mpack_writer_buffer_used(&writer);
    if (mpack_writer_destroy(&writer) != mpack_ok) {
        fprintf(stderr, "An error occurred encoding the data!\n");
        return 0;
    }
    if (written>buf_len){
        return 0;
    } else {
        *(temp_buf+written) = '\n';
    }
    return written+1;
}

// }