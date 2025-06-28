/*
 * This file contains all of the logic which is used to properly receive and send CAN commands on the ESC. The way each message is decoded, and the breakdown
 * off all the bits in each message can be found in the Rover Drive System Documentation.
 *
 * Made by: James Di Sciullo
 */
#include <stdint.h>
#include "CAN_processing.h"
//#include "uart_debugging.h"
#include <stdbool.h>
#include "pid.h"
#include "motor.h"
#include "encoder.h"


// Define the bit masks to retrieve the relevent portions of data from the CAN id
#define SENDER_DEVICE_MASK		  		(0x0400U) // 0b010000000000
#define SENDER_DEVICE_SHIFT 		    (10U)
#define NACTION_READ_DEVICE_MASK  		(0x0200U) // 0b001000000000
#define NACTION_READ_ID_DEVICE_SHIFT    (9U)
#define NMULTI_SINGLE_DEVICE_MASK		(0x0100)	// 0b000100000000
#define NMULTI_SINGLE_SHIFT				(8U)
#define NDRIVE_STEETING_DEVICE_MASK		(0x0080) // 0b000010000000
#define NDRIVE_STEERING_SHIFT 			(7U)
#define MSG_SPECIFICATION_DEVICE_MASK	(0x0070) // 0b000001110000
#define MSG_SPECIFICATION_SHIFT			(4U)
#define ID_DEVICE_MASK					(0x000f)// 0b000000001111


//Ramp up "torque" parameters
#define RAMP_MIN_MS_RUN      100   // small tweaks while already running
#define RAMP_MIN_MS_STARTUP  200   // zero → set-point, or after stop

//Startup Watchdog parameters
#define START_WD_WINDOW_MS   5000   // length of rolling window
#define START_WD_THRESHOLD   3     // # kicks that trigger a fault

// For print statements
//extern UART_HandleTypeDef huart2;

// Variable declarations
static float g_lastCommandedSpeed = 0; // Previous speed setpoint given to the esc
int s_previousDirection = 0 ; // 0 means idle, 1 means forward, -1 means backward
int DELTA_SPEED_THRESH = 200; // Threshold to clip differing speed commands
const float SPEED_ZERO_THR = 50.0f; // Threshold to consider "almost zero" / unreliable speed feedback point
const float MAX_SPEED_THR = 3200.0f;
const int WAIT_AFTER_STOP = 250; // amountin ms motor will wait after it has issued a stopMotor command
const int SAFE_STOP_SPEED_THRESHOLD = 400;

//Motor parameters --> Get these from the profiled motor!!
static float g_maxTorque   = 0.30f;    // [N·m]  conservative value
const float g_startupTorque = 0.15f;   // typical open-loop pull-in
static float g_inertia     = 0.00001242f; // [kg·m^2]
static float g_speedThresh = 50.0f;    // threshold below which we treat speed as zero

static StartWatchdog s_startWd = { .firstTick = 0, .attempts = 0 };





//The next functions here are to retrieve the correct pattern of bits from the received CAN message
uint8_t get_CAN_transmitter (uint16_t CAN_ID) {
	return (CAN_ID & SENDER_DEVICE_MASK) >> SENDER_DEVICE_SHIFT;
}
uint8_t get_CAN_action (uint16_t CAN_ID) {
	return (CAN_ID & NACTION_READ_DEVICE_MASK) >> NACTION_READ_ID_DEVICE_SHIFT;
}
uint8_t get_CAN_motor_mov_type (uint16_t CAN_ID) {
	return (CAN_ID & NMULTI_SINGLE_DEVICE_MASK) >> NMULTI_SINGLE_SHIFT;
}
uint8_t get_CAN_motor_type (uint16_t CAN_ID) {
	return (CAN_ID & NDRIVE_STEETING_DEVICE_MASK) >> NDRIVE_STEERING_SHIFT;
}
uint8_t get_CAN_SPEC (uint16_t CAN_ID) {
	return (CAN_ID & MSG_SPECIFICATION_DEVICE_MASK) >> MSG_SPECIFICATION_SHIFT;
}
int get_CAN_device_ID (uint16_t CAN_ID) {
	return (CAN_ID & ID_DEVICE_MASK);
}

/*
 * This function includes the main control flow for CAN commands.
 */
void CAN_Parse_MSG (CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData){

	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	uart_debug_print("Parsing the ID...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	float information = SingleExtractFloatFromCAN(rxData);

	ParsedCANID CANMessage; //initialize a struct for the received message
	uint16_t msg_ID = rxHeader->StdId & 0x07ff;//rxHeader->Identifier & 0x07ff ; // We only care about the first 11 bits here

	// First check to see who is transmitting --> ESCS cannot command other ECSs
	CANMessage.messageSender = (Transmitter) get_CAN_transmitter(msg_ID);
	if (CANMessage.messageSender == SLAVE){
		return;
	}

	//Check to make sure the command is directed toward the ESC
	CANMessage.motorType = (MotorType) get_CAN_motor_type(msg_ID);
	if (CANMessage.motorType == DRIVE_MOTOR){
		return;
	}

	// Check the type of action to determine which field of the struct needs to be filled in
	CANMessage.commandType = (Action) get_CAN_action(msg_ID);
	if (CANMessage.commandType == ACTION_RUN){


		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Run Command Detected\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

			CANMessage.runSpec = (RunSpec) get_CAN_SPEC(msg_ID);
	}
	else{
		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Read Command Detected\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		CANMessage.readSpec = (ReadSpec) get_CAN_SPEC(msg_ID);
	}

	// Determine if the command effects a single, or mulitple motors, and call the appropriate helper functions
	CANMessage.motorConfig = (MotorConfig) get_CAN_motor_mov_type(msg_ID);
	if (CANMessage.motorConfig == SINGLE_MOTOR){
		CANMessage.motorID = (MotorID) get_CAN_device_ID(msg_ID);
		if (CANMessage.motorID == STEERING_ID){
			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			uart_debug_print("Processing Single Command\r\n");
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			Process_Single_Steering_Motor_Command(&CANMessage, rxData);
		}
		else {
			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Not My IDr\n");

			//////////////////////////////////////////////////////////////////////////////////////////////////////
			return;
		}
	}
	else{
		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Processing Multiple Commands\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		Process_Multiple_Steering_Motor_Command(&CANMessage, rxData);
	}

}


void Process_Single_Steering_Motor_Command (ParsedCANID *CANMessageID, uint8_t *rxData){

	//single information will always come as a float (signed 4 bytes)
	float information = SingleExtractFloatFromCAN(rxData);


	if (CANMessageID->commandType == ACTION_RUN){
		switch(CANMessageID->runSpec){

		case (RUN_STOP):
				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				uart_debug_print("Motor Stopped \r\n");
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				stop_motor();
				break;

		case (RUN_ACKNOWLEDGE_FAULTS):
				break;

		case (RUN_SPEED):
				break;
		case (RUN_POSITION):
//				uart_debug_print("In case RUN_SPEED\r\n");
				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				uart_debug_print("Setpoint %d RPM\r\n", (int)information);
//				uart_debug_print("Previous Direction %d\r\n", (int)s_previousDirection);
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				ControlSingleMotor(information);
				break;
		default:
			break;
		}
		//otherwise ignore whatever was received :)
	}

	else{
		// We are now in a read command
		switch(CANMessageID->readSpec){

		case(READ_POSITION):
				float currentPosition = count_to_angle(get_counts());
				sendCANResponse(CANMessageID, currentPosition);
				break;
		case(VOLTAGE):
				break;
		case(CURRENT):
				break;
		case(GET_ALL_FAULTS):
				break;
		case(GET_CURRENT_STATE):
				break;
		case(GET_TEMPERATURE):
				// --> need to poll the gpio, check the datasheet later for whichever pin this is
				break;
		case(GET_PING):
				float feedback = 69;// ;)
				sendCANResponse(CANMessageID, feedback);
				break;
		default:
			break;
		}

	}
}

void Process_Multiple_Steering_Motor_Command (ParsedCANID *CANMessageID, uint8_t *rxData){

	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Running Multiple Motors...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	switch(CANMessageID->runSpec){

	case (RUN_POSITION):
			int16_t curESCSpeed = extract_multiple_speeds(rxData);

			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Running This Motor\r\n");
//			 uart_debug_print("Setpoint %d RPM\r\n", (int)curESCSpeed);
//			 uart_debug_print("Previous Direction %d\r\n", (int)s_previousDirection);
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			ControlSingleMotor(curESCSpeed);
			break;

	case (RUN_STOP):
			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Stop this motor\r\n");
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			stop_motor();
			break;
	default:
		break;
	}


}


void ControlSingleMotor(float angle){
	setPIDGoalA(angle);
}


void sendCANResponse(ParsedCANID *CANMessageID, float information){

	/*
	 Function used to send back data from ESCs to the master. The ID of the information that is being sent back is identical to the one
	 that was sent to the esc, except the MSB is now 1, indicating that it is the esc which is sending data back to the master. In turn, this also
	 makes sure that no other escs begin processing the message that the current one is trying to send.

	 */

	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
    uint16_t txID = 0;
    uint8_t txData[8];


    uint8_t spec = (CANMessageID->commandType == ACTION_READ)
                   ? (uint8_t)(CANMessageID->readSpec)
                   : (uint8_t)(CANMessageID->runSpec);


    // Configure the ID of the message according to what was received
    txID |= (1 & 0x01) << SENDER_DEVICE_SHIFT;
    // This caluevis now always 1 since the esc is sending data.
    txID |= (CANMessageID->motorType & 0x01) << NDRIVE_STEERING_SHIFT;
    txID |= (CANMessageID->motorConfig & 0x01) << NMULTI_SINGLE_SHIFT;
    txID |= (CANMessageID->commandType & 0x01) << NACTION_READ_ID_DEVICE_SHIFT;
    txID |= (CANMessageID->readSpec & 0x07) << MSG_SPECIFICATION_SHIFT;
    txID |= (CANMessageID->motorID & 0x0f);

    //TONY: I want the txID to be exactly what I specified in main.c
    // Move the information into the data paylaod
    memcpy(txData, &information, sizeof(float)); // data[0] --> data[3] now stores float


	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("CAN Command Sent back!\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

    //format other message peripherals

	 TxHeader.DLC = 8;
	 TxHeader.ExtId = 0;
	 TxHeader.IDE = CAN_ID_STD;
	 TxHeader.RTR = CAN_RTR_DATA;
	 TxHeader.StdId = txID;

	// Send the response

	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, txData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}

// Extracts a float from a CAN data buffer that is expected to be in little-Endian
float SingleExtractFloatFromCAN(uint8_t *data) {
    float value;
    uint8_t reorderedData[4];

    // The data is already big-endian, so we copy directly:
    reorderedData[0] = data[0];
    reorderedData[1] = data[1];
    reorderedData[2] = data[2];
    reorderedData[3] = data[3];

    memcpy(&value, reorderedData, sizeof(float));
    return value;
}


int16_t extract_multiple_speeds(const uint8_t *rxData){
    uint16_t offset = (STEERING_ID-4) * 2;
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
}
