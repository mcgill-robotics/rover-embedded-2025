#include <stdint.h>
#include <stdbool.h>
//#include "mc_interface.h"
#include <string.h>
#include <math.h>
#include "main.h"

// Defines
#ifndef CAN_processing_H
#define CAN_processing_H
#endif // CAN_processing_H

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

// Defining variables
extern int s_previousDirection;



extern int STEERING_ID;
extern CAN_HandleTypeDef hcan2;


// Enumaeration classes for different possible CAN Message casess
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
    STEERING_MOTOR		= 1
} MotorType;

typedef enum {
    RUN_STOP               	= 0,
	RUN_ACKNOWLEDGE_FAULTS 	= 1,
    RUN_SPEED              	= 2,
    RUN_POSITION           	= 3,
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
	RF_DRIVE = 0,
	LF_DRIVE = 1,
	LB_DRIVE = 2,
	RB_DRIVE = 3,
	RF_STEER = 4,
	RB_STEER = 5,
	LB_STEER = 6,
	LF_STEER = 7
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

//Watchdog for start condition!
typedef struct {
    uint32_t    firstTick;
    uint8_t     attempts;
} StartWatchdog;



//CAN Interaction prototypes
void CAN_Parse_MSG (CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void Process_Multiple_ESC_Command (ParsedCANID *parsedMessageID, uint8_t *rxData);
void Process_Single_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData);
void sendCANResponse(ParsedCANID *CANMessageID, float information);
float SingleExtractFloatFromCAN(uint8_t *data);

// Motor control prototypes
void ControlSingleMotor(float newSpeed);
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


