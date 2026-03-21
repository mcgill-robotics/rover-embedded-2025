/*
 * CAN_processing.c
 *
 * CAN FD message parsing, encoding, response transmission, and data extraction.
 * Motor control logic has been removed -- command handlers are stubbed out for now.
 *
 * The CAN ID bit-field layout and all encoding/decoding logic is preserved from the
 * original implementation. Header fields still use classic-CAN-style 11-bit IDs;
 * migration to an FD-native ID scheme will happen in a later step.
 *
 * Made by: James Di Sciullo
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "CAN_processing_v2.h"

/* Enable uart_debug_print() if available, otherwise silently discard */
#ifdef USE_UART_DEBUG
  #include "uart_debugging.h"
#else
  #define uart_debug_print(...)  ((void)0)
#endif

/*
 * CAN ID Bit-field Masks & Shifts (11-bit standard ID)
 *
 * Bit layout (MSB to LSB):
 *   [10]    Sender        (0 = Master, 1 = Slave/ESC)
 *   [9]     Action        (0 = Run,    1 = Read)
 *   [8]     MotorConfig   (0 = Multi,  1 = Single)
 *   [7]     MotorType     (0 = Drive,  1 = Steering)
 *   [6:4]   Specification (RunSpec or ReadSpec depending on Action)
 *   [3:0]   Device ID
 */
#define SENDER_DEVICE_MASK              (0x0400U)
#define SENDER_DEVICE_SHIFT             (10U)
#define NACTION_READ_DEVICE_MASK        (0x0200U)
#define NACTION_READ_ID_DEVICE_SHIFT    (9U)
#define NMULTI_SINGLE_DEVICE_MASK       (0x0100U)
#define NMULTI_SINGLE_SHIFT             (8U)
#define NDRIVE_STEERING_DEVICE_MASK     (0x0080U)
#define NDRIVE_STEERING_SHIFT           (7U)
#define MSG_SPECIFICATION_DEVICE_MASK   (0x0070U)
#define MSG_SPECIFICATION_SHIFT         (4U)
#define ID_DEVICE_MASK                  (0x000FU)

/* CAN ID field extractors */

static uint8_t get_CAN_transmitter(uint16_t CAN_ID) {
    return (CAN_ID & SENDER_DEVICE_MASK) >> SENDER_DEVICE_SHIFT;
}

static uint8_t get_CAN_action(uint16_t CAN_ID) {
    return (CAN_ID & NACTION_READ_DEVICE_MASK) >> NACTION_READ_ID_DEVICE_SHIFT;
}

static uint8_t get_CAN_motor_mov_type(uint16_t CAN_ID) {
    return (CAN_ID & NMULTI_SINGLE_DEVICE_MASK) >> NMULTI_SINGLE_SHIFT;
}

static uint8_t get_CAN_motor_type(uint16_t CAN_ID) {
    return (CAN_ID & NDRIVE_STEERING_DEVICE_MASK) >> NDRIVE_STEERING_SHIFT;
}

static uint8_t get_CAN_SPEC(uint16_t CAN_ID) {
    return (CAN_ID & MSG_SPECIFICATION_DEVICE_MASK) >> MSG_SPECIFICATION_SHIFT;
}

static int get_CAN_device_ID(uint16_t CAN_ID) {
    return (CAN_ID & ID_DEVICE_MASK);
}

/* Main CAN message parser */

void CAN_Parse_MSG(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    uart_debug_print("Parsing the ID...\r\n");

    ParsedCANID CANMessage;
    uint16_t msg_ID = rxHeader->Identifier & 0x07FFU;

    /* ESCs do not process messages from other ESCs */
    CANMessage.messageSender = (Transmitter)get_CAN_transmitter(msg_ID);
    if (CANMessage.messageSender == SLAVE) {
        return;
    }

    CANMessage.commandType = (Action)get_CAN_action(msg_ID);
    if (CANMessage.commandType == ACTION_RUN) {
        uart_debug_print("Run Command Detected\r\n");
        CANMessage.runSpec = (RunSpec)get_CAN_SPEC(msg_ID);
    } else {
        uart_debug_print("Read Command Detected\r\n");
        CANMessage.readSpec = (ReadSpec)get_CAN_SPEC(msg_ID);
    }

    CANMessage.motorType = (MotorType)get_CAN_motor_type(msg_ID);
    CANMessage.motorConfig = (MotorConfig)get_CAN_motor_mov_type(msg_ID);

    if (CANMessage.motorConfig == SINGLE_MOTOR) {
        CANMessage.motorID = (MotorID)get_CAN_device_ID(msg_ID);
        if ((int)CANMessage.motorID != ESC_ID) {
            uart_debug_print("Not my ID\r\n");
            return;
        }
        uart_debug_print("Processing Single Command\r\n");
        Process_Single_ESC_Command(&CANMessage, rxData);
    } else {
        uart_debug_print("Processing Multiple Command\r\n");
        Process_Multiple_ESC_Command(&CANMessage, rxData);
    }
}

/* Single-motor command dispatch */

void Process_Single_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData)
{
    float information = SingleExtractFloatFromCAN(rxData);

    if (CANMessageID->commandType == ACTION_RUN) {
        Handle_Run_Command(CANMessageID, rxData, information);
    } else {
        Handle_Read_Command(CANMessageID, rxData);
    }
}

/* Multiple-motor command dispatch */

void Process_Multiple_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData)
{
    uart_debug_print("Running Multiple Motors...\r\n");

    float information = extract_multiple_positions_arm(rxData);

    if (CANMessageID->commandType == ACTION_RUN) {
        Handle_Run_Command(CANMessageID, rxData, information);
    } else {
        Handle_Read_Command(CANMessageID, rxData);
    }
}

/* Stub command handlers -- replace with real logic later */

void Handle_Run_Command(ParsedCANID *id, uint8_t *rxData, float info)
{
    (void)rxData;

    uart_debug_print("Handle_Run_Command: runSpec=%d, info=%d\r\n",
                     (int)id->runSpec, (int)info);

    switch (id->runSpec) {
    case RUN_STOP:
        uart_debug_print("  -> STOP\r\n");
        break;
    case RUN_ACKNOWLEDGE_FAULTS:
        uart_debug_print("  -> ACK FAULTS\r\n");
        break;
    case RUN_SPEED:
        uart_debug_print("  -> SPEED: %d RPM\r\n", (int)info);
        break;
    case RUN_POSITION_INCREMENT:
        uart_debug_print("  -> POS INCREMENT: %d deg\r\n", (int)info);
        break;
    case RUN_POSITION:
        uart_debug_print("  -> POSITION: %d deg\r\n", (int)info);
        break;
    case RUN_CALIBRATION:
        uart_debug_print("  -> CALIBRATION\r\n");
        break;
    case RUN_FOLLOW_POSITION:
        uart_debug_print("  -> FOLLOW POS: %d deg\r\n", (int)info);
        break;
    default:
        uart_debug_print("  -> UNKNOWN runSpec\r\n");
        break;
    }
}

void Handle_Read_Command(ParsedCANID *id, uint8_t *rxData)
{
    (void)rxData;

    float dummy_response = 0.0f;

    uart_debug_print("Handle_Read_Command: readSpec=%d\r\n", (int)id->readSpec);

    switch (id->readSpec) {
    case READ_CALIBRATION:
        uart_debug_print("  -> READ_CALIBRATION\r\n");
        dummy_response = 0.0f;
        break;
    case READ_POSITION:
        uart_debug_print("  -> READ_POSITION\r\n");
        dummy_response = 0.0f;
        break;
    case VOLTAGE:
        uart_debug_print("  -> VOLTAGE\r\n");
        dummy_response = 12.0f;
        break;
    case CURRENT:
        uart_debug_print("  -> CURRENT\r\n");
        dummy_response = 1.5f;
        break;
    case GET_ALL_FAULTS:
        uart_debug_print("  -> GET_ALL_FAULTS\r\n");
        dummy_response = 0.0f;
        break;
    case GET_CURRENT_STATE:
        uart_debug_print("  -> GET_CURRENT_STATE\r\n");
        dummy_response = 0.0f;
        break;
    case GET_TEMPERATURE:
        uart_debug_print("  -> GET_TEMPERATURE\r\n");
        dummy_response = 25.0f;
        break;
    case GET_PING:
        uart_debug_print("  -> GET_PING\r\n");
        dummy_response = 69.0f;
        break;
    default:
        uart_debug_print("  -> UNKNOWN readSpec\r\n");
        break;
    }

    sendCANResponse(id, dummy_response);
}

/*
 * Sends a CAN FD response back to the master. The ID mirrors the received
 * message but with the sender bit set to 1 (SLAVE), so no other ESC processes it.
 */
void sendCANResponse(ParsedCANID *CANMessageID, float information)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint16_t txID = 0;
    uint8_t txData[64] = {0}; /* full FD payload, unused bytes stay zero */

    uint8_t spec = (CANMessageID->commandType == ACTION_READ)
                   ? (uint8_t)(CANMessageID->readSpec)
                   : (uint8_t)(CANMessageID->runSpec);

    /* Build the 11-bit response ID */
    txID |= (1U & 0x01U) << SENDER_DEVICE_SHIFT;
    txID |= ((uint16_t)CANMessageID->commandType & 0x01U) << NACTION_READ_ID_DEVICE_SHIFT;
    txID |= ((uint16_t)CANMessageID->motorConfig & 0x01U) << NMULTI_SINGLE_SHIFT;
    txID |= ((uint16_t)CANMessageID->motorType   & 0x01U) << NDRIVE_STEERING_SHIFT;
    txID |= ((uint16_t)spec                       & 0x07U) << MSG_SPECIFICATION_SHIFT;
    txID |= ((uint16_t)CANMessageID->motorID     & 0x0FU);

    /* Pack float into bytes 0-3; bytes 4-63 remain zero */
    memcpy(txData, &information, sizeof(float));

    txHeader.Identifier          = txID;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8; /* bump to _12/_16/etc when payload grows */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_ON;      /* data phase at 2 Mbps */
    txHeader.FDFormat            = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    uart_debug_print("CAN FD Response Sent!\r\n");

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK) {
        Error_Handler();
    }
}

/* Data extraction utilities */

/* Extract a 32-bit float from bytes 0-3 of a CAN data buffer */
float SingleExtractFloatFromCAN(uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(float));
    return value;
}

/* Extract a signed 16-bit speed for this ESC from a multi-motor payload.
 * Each ESC occupies 2 bytes at offset (ESC_ID * 2). */
int16_t extract_multiple_speeds(const uint8_t *rxData)
{
    uint16_t offset = (uint16_t)(ESC_ID * 2);
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
}

/* Convert IEEE-754 half-precision (16-bit) float to 32-bit float */
float half_to_float(uint16_t half)
{
    uint32_t sign = (half >> 15) & 0x00000001U;
    uint32_t exp  = (half >> 10) & 0x0000001FU;
    uint32_t mant = half & 0x000003FFU;
    uint32_t f;

    if (exp == 0) {
        if (mant == 0) {
            f = sign << 31;
        } else {
            while ((mant & 0x00000400U) == 0) {
                mant <<= 1;
                exp  -= 1;
            }
            exp  += 1;
            mant &= ~0x00000400U;
            f = (sign << 31) | ((exp + 112) << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        f = (sign << 31) | 0x7F800000U | (mant << 13);
    } else {
        f = (sign << 31) | ((exp + 112) << 23) | (mant << 13);
    }

    float result;
    memcpy(&result, &f, sizeof(result));
    return result;
}

/* Extract this ESC's half-float position from a multi-motor arm payload.
 * ESC_ID 8/9/10 maps to index 0/1/2, each occupying 2 bytes. */
float extract_multiple_positions_arm(const uint8_t *rxData)
{
    int index = ESC_ID - 8;
    if (index < 0 || index > 2) {
        return 0.0f;
    }

    uint16_t raw_half = (uint16_t)(rxData[index * 2 + 1] << 8) | rxData[index * 2];
    return half_to_float(raw_half);
}
