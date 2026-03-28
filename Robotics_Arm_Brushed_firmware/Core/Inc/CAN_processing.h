#include <stdint.h>
#include <stdbool.h>
//#include "mc_interface.h"
#include <string.h>
#include <math.h>
#include "main.h"
#include "motorControl.h"

// Defines
#ifndef CAN_processing_H
#define CAN_processing_H


#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif



// Defining variables
extern int s_previousDirection;
//extern Motor * all_motors_list[NB_MOTORS]; // in order gripper, roll, pitch


extern int STEERING_ID;
extern FDCAN_HandleTypeDef hfdcan2;

extern Motor * all_motors_list[NB_MOTORS];

// Enumeration classes for different possible CAN Message cases
typedef enum {
    MASTER				= 0,
    SLAVE				= 1
} Transmitter;

typedef enum {
    ACTION_RUN			= 0,
    ACTION_READ			= 1
} Action;

typedef enum {
    MULTIPLE_MOTORS		= 0,
    SINGLE_MOTOR		= 1
} MotorConfig;

typedef enum {
    DRIVE_MOTOR			= 0,
    STEERING_MOTOR		= 1,
	ARM_BRUSHED_MOTOR	= 2
} MotorType;

typedef enum {
    RUN_STOP               	= 0,
	RUN_ACKNOWLEDGE_FAULTS 	= 1,
    RUN_SPEED              	= 2,
    RUN_POSITION           	= 3
} RunSpec;

typedef enum {
    READ_SPEED             	 = 0,
    READ_POSITION          	 = 1,
    VOLTAGE           		 = 2,
    CURRENT           	 	 = 3,
    GET_ALL_FAULTS    		 = 4,
    GET_CURRENT_STATE   	 = 5,
	GET_TEMPERATURE	  		 = 6,
	GET_PING				 = 7
} ReadSpec;

typedef enum {
	GRIPPER_ID= 0,
	PITCH_ID = 1,
	ROLL_ID = 2
} MotorID;

// Struct for a CAN message ID
typedef struct {
	Transmitter		messageSender;
	MotorType		motorType;
	MotorConfig		motorConfig;
    Action			commandType;
	ReadSpec 		readSpec;
	RunSpec			runSpec;
	MotorID			motorID;
} ParsedCANID;



//CAN Interaction prototypes
#include "motorControl.h"
void CAN_Parse_MSG (FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void Process_Multiple_Motor_Command (ParsedCANID *parsedMessageID, uint8_t *rxData);
void Process_Single_Motor_Command (Motor * motor, ParsedCANID *CANMessageID, uint8_t *rxData);
void sendCANResponse(ParsedCANID *CANMessageID, float information);
float SingleExtractFloatFromCAN(uint8_t *data);

// Motor control prototypes
void ControlSingleMotor(Motor * motor, float newSpeed); //TODO FIX
void runSingleMotorV2(float newSpeed); //TODO remove V part
void IdleSingleMotor(float newSpeed);
void StartSingleMotor(float newSpeed);

bool checkReversing(float speedCmd);
//bool safeStopMotor(float currentSpeedRpm, MCI_State_t motorState);
//helper function prototypes

float speedCheck (float targetSpeed);
float clippingCheck(float currentSpeedSetpoint);
int16_t extract_multiple_speeds(const uint8_t *rxData);
uint16_t computeRampTimeMs(float currentSpeedRpm, float targetSpeedRpm, bool isStartup);

#endif // CAN_processing_H
