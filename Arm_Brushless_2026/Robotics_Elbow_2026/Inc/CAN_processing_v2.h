/*
 * CAN_processing_v2.h
 *
 *  Created on: Mar 20, 2026
 *      Author: james
 */

#ifndef APPLICATION_USER_CAN_PROCESSING_V2_H_
#define APPLICATION_USER_CAN_PROCESSING_V2_H_
/*
 * CAN_processing.h
 *
 * Header for CAN FD message parsing, encoding, and command dispatch.
 * Motor control logic is NOT included here — this file is CAN-protocol only.
 *
 * Made by: James Di Sciullo
 */
/*
 * CAN_processing.h
 *
 * Header for CAN FD message parsing, encoding, and command dispatch.
 * Motor control logic is NOT included here -- this file is CAN-protocol only.
 *
 * Made by: James Di Sciullo
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "stm32g4xx_hal.h"

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

/* External variables (defined elsewhere, e.g. main.c) */
extern int ESC_ID;
extern FDCAN_HandleTypeDef hfdcan1;

/* CAN ID enumeration types */

typedef enum {
    MASTER              = 0,
    SLAVE               = 1
} Transmitter;

typedef enum {
    ACTION_RUN          = 0,
    ACTION_READ         = 1
} Action;

typedef enum {
    MULTIPLE_MOTORS     = 0,
    SINGLE_MOTOR        = 1
} MotorConfig;

typedef enum {
    DRIVE_MOTOR         = 0,
    STEERING_MOTOR      = 1
} MotorType;

typedef enum {
    RUN_STOP                = 0,
    RUN_ACKNOWLEDGE_FAULTS  = 1,
    RUN_SPEED               = 2,
    RUN_POSITION_INCREMENT  = 3,
    RUN_POSITION            = 4,
    RUN_CALIBRATION         = 5,
    RUN_FOLLOW_POSITION     = 6
} RunSpec;

typedef enum {
    READ_CALIBRATION    = 0,
    READ_POSITION       = 1,
    VOLTAGE             = 2,
    CURRENT             = 3,
    GET_ALL_FAULTS      = 4,
    GET_CURRENT_STATE   = 5,
    GET_TEMPERATURE     = 6,
    GET_PING            = 7
} ReadSpec;

typedef enum {
    WAIST    = 8,
    SHOULDER = 9,
    ELBOW    = 10
} MotorID;

/* Parsed CAN ID struct */

typedef struct {
    Transmitter     messageSender;
    MotorType       motorType;
    MotorConfig     motorConfig;
    Action          commandType;
    ReadSpec        readSpec;
    RunSpec         runSpec;
    MotorID         motorID;
} ParsedCANID;

/* CAN message parsing & dispatch */
void CAN_Parse_MSG(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void Process_Single_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData);
void Process_Multiple_ESC_Command(ParsedCANID *CANMessageID, uint8_t *rxData);

/* CAN response (TX) */
void sendCANResponse(ParsedCANID *CANMessageID, float information);

/* Data extraction helpers */
float    SingleExtractFloatFromCAN(uint8_t *data);
int16_t  extract_multiple_speeds(const uint8_t *rxData);
float    extract_multiple_positions_arm(const uint8_t *rxData);
float    half_to_float(uint16_t half);

/* Stub command handlers (to be implemented later) */
void Handle_Run_Command(ParsedCANID *id, uint8_t *rxData, float info);
void Handle_Read_Command(ParsedCANID *id, uint8_t *rxData);


#endif /* APPLICATION_USER_CAN_PROCESSING_V2_H_ */
