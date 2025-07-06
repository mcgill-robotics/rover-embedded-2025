#include "brushed_comms.h"
#include <string.h>

// Internal buffer for communication
static uint8_t comms_buffer[256];

// Initialize communication
void BrushedComms_Init(void) {
    // Initialize communication hardware (e.g., USB, UART)
    // Placeholder for hardware initialization code
}

// Process incoming and outgoing data
void BrushedComms_Process(void) {
    // Placeholder for processing incoming and outgoing data
    // This could involve reading from a buffer, parsing commands, etc.
}

// Send feedback to PC
void BrushedComms_SendFeedback(const FeedbackData* feedback) {
    if (feedback == NULL) {
        return;
    }

    // Serialize feedback data into the communication buffer
    memcpy(comms_buffer, feedback->motor_position, sizeof(feedback->motor_position));
    memcpy(comms_buffer + sizeof(feedback->motor_position), feedback->motor_current, sizeof(feedback->motor_current));
    comms_buffer[sizeof(feedback->motor_position) + sizeof(feedback->motor_current)] = feedback->error_code;

    // Send the serialized data over the communication interface
    // Placeholder for sending data (e.g., USB, UART)
}

// Handle setpoint data from PC
void BrushedComms_HandleSetpoint(const uint8_t* data, uint16_t length) {
    if (data == NULL || length == 0) {
        return;
    }

    // Process setpoint data
    // Placeholder for handling setpoint data
}

// Handle commands from PC
void BrushedComms_HandleCommand(const uint8_t* data, uint16_t length) {
    if (data == NULL || length == 0) {
        return;
    }

    // Process command data
    // Placeholder for handling command data
}

// Report error to PC
void BrushedComms_ReportError(uint8_t error_code) {
    // Serialize error code into the communication buffer
    comms_buffer[0] = CMD_ERROR;
    comms_buffer[1] = error_code;

    // Send the serialized error code over the communication interface
    // Placeholder for sending data (e.g., USB, UART)
}

uint8_t BrushedComms_CalculateCRC(uint8_t* data, uint16_t length){
    uint8_t crc = 0x00;
    while (length--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}