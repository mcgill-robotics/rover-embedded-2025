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

// Tony debugging
int whatsmymotorid = 0;
int whatsmymotortype = 0;
int whatsmyaction = 0;
int whatsmyreadrunspec = 0;
int johnyangle = 0;

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


	ParsedCANID CANMessage; //initialize a struct for the received message
	uint16_t msg_ID = rxHeader->StdId;//rxHeader->Identifier & 0x07ff ; // We only care about the first 11 bits here
	whatsmymotorid = CANMessage.motorID;
	whatsmymotortype = CANMessage.motorType;
	whatsmyaction = CANMessage.commandType;
	whatsmyreadrunspec = CANMessage.runSpec;
	setPIDGoalA(rxData[0]);
	johnyangle = rxData[0];

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
//				safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), MC_GetSTMStateMotor1());
				stop_motor();
				break;

		case (RUN_ACKNOWLEDGE_FAULTS):
//				MC_AcknowledgeFaultMotor1();
				break;

//		case (RUN_SPEED):
//				uart_debug_print("In case RUN_SPEED\r\n");
////				MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				uart_debug_print("Setpoint %d RPM\r\n", (int)information);
//				uart_debug_print("Previous Direction %d\r\n", (int)s_previousDirection);
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				ControlSingleMotor(information);
//				break;
		case (RUN_POSITION):
//				uart_debug_print("In case RUN_SPEED\r\n");
//				MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
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
//				float phaseVoltage = MC_GetPhaseVoltageAmplitudeMotor1();
//				sendCANResponse(CANMessageID, phaseVoltage);
				break;
		case(CURRENT):
//				float phaseCurrent = MC_GetPhaseCurrentAmplitudeMotor1();
//				sendCANResponse(CANMessageID, phaseCurrent);
				break;
		case(GET_ALL_FAULTS):
//				float currentFaults = MC_GetOccurredFaultsMotor1();
//				sendCANResponse(CANMessageID, currentFaults);
				break;
		case(GET_CURRENT_STATE):
//				float currentState = MC_GetSTMStateMotor1();
//				sendCANResponse(CANMessageID, currentState);
				break;
		case(GET_TEMPERATURE):
				//TODO --> need to poll the gpio, check the datasheet later for whichever pin this is
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

	case (RUN_SPEED):

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

//	 	 	safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), MC_GetSTMStateMotor1());
			break;
	default:
		break;
	}


}


void ControlSingleMotor(float angle){
	setPIDGoalA(angle);
//	MCI_State_t motorState =  MC_GetSTMStateMotor1();
//
//	switch (motorState){
//
//	case(IDLE):
//
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Idle\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		IdleSingleMotor(newSpeed);
//		break;
//
//	case(RUN):
//
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Run\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		runSingleMotorV2(newSpeed);
//		break;
//
//	case(START):
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Start\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		StartSingleMotor(newSpeed);
//		break;
//
//	case(SWITCH_OVER):
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Switch over\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		StartSingleMotor(newSpeed);
//		break;
//
//	case(OFFSET_CALIB):
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Offset Calibration\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		StartSingleMotor(newSpeed);
//		break;
//
//
//	case(FAULT_OVER):
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Fault Over \r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		break;
//
//	case(FAULT_NOW):
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor State: Fault Now \r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		break;
//
//	default:
//	    uart_debug_print("Motor state not explicitly handled: %d\r\n", motorState);
//	    break;
//	}
//
//	return;
}


void runSingleMotorV2(float newSpeed){

	/*
	 * The run state indicates that the motor is currently being controlled by the ESC, that is, it is already moving. To leave this state,
	 * either a fault has occured or a stop command has been issued.
	 *
	 * Since the motor has been previously moving, the purpose of this state is to simply update the motor speed with the new setpoint
	 * given in the command that was received. Various checks are performed on the new setpoint in order to properly control the motor:
	 *
	 *	- Check that the setpoint is with motor parameters (less than max rmp, greater than very slow rpm that would provide unreliable data)
	 *	- Check that the new setpoint does not differ by too much from the current operating speed, and adjust it if it is
	 *	- Check that reversing did not occur, that is if the new speed setpoint is now in the opposite direction, the motor will be indicated to stop
	 *		instead of procesing in the negative direction. Note here, that the motor will not be set to the new negative setpoint, rather it will stop
	 *		and is is up to the next command in order to determine the setpoint. This has been done as to avoid any timing errors on the CAN bus, as well
	 *		as to allow the motor to ramp to higher values in the negative direction that is not affected by the clipping function which controls the
	 *		degree of change between past and previous setpoints as previously discussed.
	 *
	 *	Based on these checks, the action taken by the motor is determined. In short, the motor will either:
	 *	1. Move to the new setpoint indicated in the command in the same direction
	 *	2. Stop the motor if a 0 speed input is received, or direction is changed
	 */

	float speedCmd = newSpeed;

	speedCmd = speedCheck(speedCmd); // Checks to make sure the speed is within acceptable limits, corrects otherwise
	speedCmd = clippingCheck(speedCmd); // Checks the degree of variation between the previous two commands, if too great than correct

//	if (checkReversing(speedCmd)){ // If the motor has reversed, then motor is stopped (or fault has occured) so leave process
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		 uart_debug_print("REVERSING DETECTED!\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		// Stop motor in order to bring it back to zero
//		if(safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), RUN)){
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has executed successfully \r\n");
//			 uart_debug_print("Motor is now stopped after direction change \r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
//		else{
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has NOT executed successfully!!!! \r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
//		return;
//	}

	// After this point, it means the setpoint is either 0, or a change of speed in the same direction

//	if (fabsf(speedCmd) < 0.001){
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor received command that is or was considered to be 0\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), RUN);
//		return;
//	}

	uint16_t myRampTime = (uint16_t)computeRampTimeMs(MC_GetMecSpeedReferenceMotor1_F(), speedCmd, false);
	float currentSpeed = MC_GetMecSpeedReferenceMotor1_F();
	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Motor is already moving, change setpoint in same direction\r\n");
//	 uart_debug_print("New Ramp Setpoint %d RPM\r\n", (int)speedCmd);
//	 uart_debug_print("New Ramp time allocated %d ms\r\n", (int)myRampTime);
//	 uart_debug_print("Current direction is %d \r\n", s_previousDirection);
//	 uart_debug_print("Current actual speed is %d\r\n", (int)currentSpeed);
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime);
	s_previousDirection = (speedCmd > 0) ? 1 : -1;
    g_lastCommandedSpeed = speedCmd;
}


void IdleSingleMotor(float newSpeed){
	/*
	 * This idle state indicates that the esc is not currently controlling the motor, this usually means the motor is stopped. In order to
	 * leave the idle state, the motor needs to start, or a fault needs to occur, meaning that IDLE state is only realized on ESC boot (POR)
	 * or after a successful motor stop.
	 *
	 * When it comes to controlling the motor, from the IDLE state there are really only 2 options:
	 *
	 * 1. A zero setpoint (or near 0) was commanded, so nothing will happen to the motor since it is already in the idle state (it will not start)
	 * 2. A non-zero setpoint is received, so the motor will begin its start sequence to ramp to the given speed.
	 */

	float speedCmd = newSpeed;
	speedCmd = speedCheck(speedCmd);


	if (fabsf(speedCmd) < 0.001){
		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor received command that is or was considered to be 0, so nothing happens\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
		return;
	}

	// Watchdog for any experience of multiple start requests at the same time
    uint32_t now = HAL_GetTick();

    if (now - s_startWd.firstTick > START_WD_WINDOW_MS) {
        /* window expired – start a new one */
    	////////////////////////////////////////////////////////////////////////////////////////////////////////
//    	 uart_debug_print("Safe amount of time since last start!\r\n");
//    	 uart_debug_print("Amount of time since last start: %d \r\n", (int) now - s_startWd.firstTick );

    	////////////////////////////////////////////////////////////////////////////////////////////////////////
        s_startWd.firstTick = now; // sets last time start was issued
        s_startWd.attempts  = 0;
    }
	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Amount of start attempts within 1s of eachother: %d\r\n", s_startWd.attempts);
	////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (++s_startWd.attempts >= START_WD_THRESHOLD) {
//        MCI_FaultProcessing(&Mci[0], MC_DP_FAULT, 0);
        s_startWd.attempts = 0;
        return;
    }

    // watchdog check over

	uint16_t myRampTime = (uint16_t)computeRampTimeMs(MC_GetMecSpeedReferenceMotor1_F(), speedCmd, true);
	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Motor will begin to ramp!\r\n");
//	 uart_debug_print("New Ramp Setpoint %d RPM\r\n", (int)speedCmd);
//	 uart_debug_print("New Ramp time allocated %d ms\r\n", (int)myRampTime);
//	 uart_debug_print("Current direction is %d \r\n", s_previousDirection);
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Motor startup sequence
	MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime); // Must set a setpoint before startup --> otherwise unpredictable behavior
	if (!MC_StartMotor1()) {
		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		 uart_debug_print("Start Failed...");
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		return; // start failed
	}

	//Than motor start command has been issued
	s_previousDirection = (speedCmd > 0) ? 1 : -1;
	g_lastCommandedSpeed = speedCmd;

	return;
}

void StartSingleMotor (float newSpeed){

	/*
	 * The Start state is entered when the motor is issued the startMotor() function, and it initializes the starting sequence. The same
	 * logic of the START state is also true for the SWITHC_OVER state, and the OFFSET_CALIB, so this function is used for all three cases
	 *
	 * When in this state, on of three things may occur:
	 * 1. Another setpoint in the same direction is received
	 * 		In this cse, we will simply ignore the setpoint as it would otherwise be buffered in the lower level firmware anyways,
	 * 		so it is more efficient so simply drop the command
	 * 2. A zero (or close to 0) speed is issued, in which case the motor should stop the starting sequence
	 * 3. A command with a different direction is issued, in which the case the motor should stop the starting sequence
	 */

	float speedCmd = newSpeed;
	speedCmd = speedCheck(speedCmd);

	if (fabs(speedCmd) < 0.001){

		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		uart_debug_print("Motor received command that is or was considered to be 0\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		if (safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), START)){
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has executed successfully \r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
//		else{
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has NOT executed successfully!!!! \r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
	}

	else if (checkReversing(speedCmd)){

//		if (safeStopMotor( MC_GetMecSpeedReferenceMotor1_F(), START)){
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has executed successfully \r\n");
//			 uart_debug_print("Motor is now stopped after direction change\r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
//		else{
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("Safe Stop has NOT executed successfully!!!! \r\n");
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//		}
	}
	else{
		return;
	}
}


float speedCheck(float targetSpeed){
	/*
	 * The main purpose of this function is to make sure that the speed demanded falls within the actual range at which the motor
	 * is capable of operating in.
	 */

	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Checking speed...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Deadzone for small speeds
    if (fabsf(targetSpeed) < SPEED_ZERO_THR) {
    	targetSpeed = 0.0f;
    }

    // Check for out of bound setpoint
    if (fabsf(targetSpeed) > MAX_SPEED_THR) {
    	targetSpeed = (targetSpeed > 0) ? MAX_SPEED_THR : -MAX_SPEED_THR;
    }
    return targetSpeed;
}


float clippingCheck(float currentSpeedSetpoint){

	/*
	 * The main purpose of this function is to check if the speed demanded is much greater than the previous speed, and if so, to adjust
	 * the change in speed such that it is not too great and will cause too high of a voltage spike to fault the motor. This logic works
	 * due to how the motor is being operated for our teams rover, and is highly project based. This can also be implemented in software,
	 * however it will be integrated in the firmware for this 2025 Rover.
	 */

	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Checking clipping...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	float delta = currentSpeedSetpoint - g_lastCommandedSpeed;

	if (fabsf(delta) > DELTA_SPEED_THRESH) {
		g_lastCommandedSpeed += (delta > 0.0f ? DELTA_SPEED_THRESH : -DELTA_SPEED_THRESH);
	} else {
		g_lastCommandedSpeed = currentSpeedSetpoint;
	}

	return speedCheck(g_lastCommandedSpeed);

}

//bool safeStopMotor(float currentSpeedRpm, MCI_State_t motorState){
//
//	/*
//	 * This function is designed in order to safely stop the motor. The ST library already containts a stopMotor() function,
//	 * although it can sometimes be unsafe to use depending on how the motor is operating, and as it is a non-blocking function,
//	 * it can be hard to know if it actually executed or not. This functon also updates the previous setpoint, and the direction
//	 * after is has executed
//	 *
//	 * This function will do one of two things based on the current speed of the motor and the State:
//	 * 1. The motor is moving quickly
//	 * 		if the motor is moving quickly when this function is called, then a blocking ramp downwards is created, so that the motor
//	 * 		is guided to a slow stop, so as to avoid any issued when stopping abruptly while the motor is moving quickly. In addition,
//	 * 		due to the nature of the code preventing large jumps in speed setpoints, under normal operation this condition is never
//	 * 		really used since the motor setpoints would otherwise be guided to a slow speed before stopping thanks to the clipping check
//	 * 		function. Thus, condition 1 is really only used for if the user wants to abruptly stop the motor manually instead of letting
//	 * 		it gradually glide to a hault.
//	 * 2. The motor was not previously moving quickly, or is in the startup phases
//	 * 		If this is the case, then the stopMotor() function from ST is called directly, as there are less risks since the motor is already
//	 * 		moving quite slowly.
//	 *
//	 * 	It returns true if the mottor has been successfulyl stoped, and false if an error occured.
//	 */
//
//	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("Performing a safe stop\r\n");
//	////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//	//First check to see if the motor was moving a significant amount before this function was being called
//	if (fabsf(currentSpeedRpm) > SAFE_STOP_SPEED_THRESHOLD && motorState != START && motorState != SWITCH_OVER){
//
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		 uart_debug_print("Motor is being guided to a stop\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		//Slow down through a speed ramp
//		float rampTarget = (currentSpeedRpm > 0) ? 100.0f : -100.0f;
//		float deltaSpeed = currentSpeedRpm - rampTarget;
//
//		int rampDownDivisions = (int)(fabsf(deltaSpeed) / DELTA_SPEED_THRESH);
//		if (rampDownDivisions < 1) rampDownDivisions = 1;  // Ensure at least one division
//
//		float stepSize = deltaSpeed / rampDownDivisions;
//
//		// Slowly ramp down to target before stopping
//		for (int i = 1; i <= rampDownDivisions; i++) {
//			float intermediateTarget = currentSpeedRpm - (stepSize * i);
//			uint16_t rampTime = computeRampTimeMs(MC_GetMecSpeedReferenceMotor1_F(), intermediateTarget, false);
//			MC_ProgramSpeedRampMotor1_F(intermediateTarget, rampTime);
//			HAL_Delay(rampTime);  // Wait for ramp to apply before next one
//		}
//
//		// Final ramp to fixed low-speed value to soften the last deceleration step
//		uint16_t finalRampTime = computeRampTimeMs(MC_GetMecSpeedReferenceMotor1_F(), rampTarget, false);
//		MC_ProgramSpeedRampMotor1_F(rampTarget, finalRampTime);
//
//		// Wait for motor speed to reach near 100 --> Use timeout in case this runs into an issue
//		uint32_t tStart = HAL_GetTick();
//		while (fabsf(MC_GetMecSpeedReferenceMotor1_F()) > 120.0f) {
//			if (HAL_GetTick() - tStart > 1000) break; // Timeout safety
//			HAL_Delay(10);
//		}
//
//	    MC_StopMotor1();
//		//  Wait for motor to become IDLE
//	    tStart = HAL_GetTick();
//		while (1) {
//			HAL_Delay(5); // poll the state until it IDLE
//			MCI_State_t currState = MC_GetSTMStateMotor1();
////			int currentFaults = MC_GetOccurredFaultsMotor1();
////			////////////////////////////////////////////////////////////////////////////////////////////////////////
////			 uart_debug_print("current state is %d\r\n", currState);
////			 uart_debug_print("current fault is %d\r\n", currentFaults);
////			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			if (currState == IDLE) {
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				 uart_debug_print("Block wait \r\n");
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				HAL_Delay(WAIT_AFTER_STOP); // Tune this value for seemless transition
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				 uart_debug_print("Motor is now stopped \r\n");
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				s_previousDirection = 0;
//				g_lastCommandedSpeed = 0;
//				return true;
//			}
//
//		    if (HAL_GetTick() - tStart > 2000) { //use 1 sec for timeout period
//		        uart_debug_print("Motor failed to stop in time!!\r\n");
//			    s_previousDirection = 0;
//			    g_lastCommandedSpeed = 0;
//		        MCI_FaultProcessing(&Mci[0], MC_DP_FAULT, 0);
//		        return false;
//		    }
//		}
//	}
//
//	else{
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//		 uart_debug_print("Motor is moving slowly or is in a startup phase, so it is not being guided to a stop\r\n");
//		////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		//Motor is already moving slowly/on startup
//	    MC_StopMotor1();
//
//		//  Wait for motor to become IDLE
//		uint32_t tStart = HAL_GetTick();
//		while (1) {
//			HAL_Delay(5); // poll the state until it IDLE
//			MCI_State_t currState = MC_GetSTMStateMotor1();
////			int currentFaults = MC_GetOccurredFaultsMotor1();
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("current state is %d\r\n", currState);
//
//			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			if (currState == IDLE) {
//
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				 uart_debug_print("Block wait \r\n");
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				HAL_Delay(WAIT_AFTER_STOP); // Tune this value for seemless transition
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				 uart_debug_print("Motor is now stopped \r\n");
//				////////////////////////////////////////////////////////////////////////////////////////////////////////
//				s_previousDirection = 0;
//				g_lastCommandedSpeed = 0;
//				return true;
//			}
//
//		    if (HAL_GetTick() - tStart > 2000) { //use 1 sec for timeout period
//		        uart_debug_print("Motor failed to stop in time!!\r\n");
//			    s_previousDirection = 0;
//			    g_lastCommandedSpeed = 0;
//		        MCI_FaultProcessing(&Mci[0], MC_DP_FAULT, 0);
//		        return false;
//		    }
//		}
//	}
//}


bool checkReversing(float speedCmd){
	/* This function is used to check if the motor is changing directions. If the motor received a setpoint which is in the opposite
	 * direction, then true is returned, otherwise false.
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	 uart_debug_print("Checking reversing...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool reversing = false;
	if (s_previousDirection > 0 && speedCmd < 0) {
		reversing = true;
	}
	else if (s_previousDirection < 0 && speedCmd > 0) {
		reversing = true;
	}

	return reversing;
}


uint16_t computeRampTimeMs(float currentSpeedRpm, float targetSpeedRpm, bool isStartup){

    // Convert from RPM to rad/s
    float w1 = currentSpeedRpm * 2.0f * (float)M_PI / 60.0f;
    float w2 = targetSpeedRpm * 2.0f * (float)M_PI / 60.0f;

    // If the magnitude is below threshold, treat as 0 for more stable math
    if (fabsf(currentSpeedRpm) < g_speedThresh) {
        w1 = 0.0f;
    }
    if (fabsf(targetSpeedRpm) < g_speedThresh) {
        w2 = 0.0f;
    }

    // Max acceleration alpha = T_max / J
    float alpha = (isStartup ? g_startupTorque : g_maxTorque) / g_inertia;// [rad/s^2]

    // Time (seconds) = delta_omega / alpha
    float deltaW = fabsf(w2 - w1);
    float timeSec = deltaW / alpha;

    // Convert to milliseconds
    float timeMs = timeSec * 1000.0f;

    // For safety, clamp time to at least 200 ms or so
    float tMin = isStartup ? RAMP_MIN_MS_STARTUP : RAMP_MIN_MS_RUN;
    if (timeMs < tMin)   timeMs = tMin;
    if (timeMs > 5000.0f) timeMs = 5000.0f;      // 5 s ceiling – tune as desired
    return (uint16_t)(timeMs + 0.5f);
}


void sendCANResponse(ParsedCANID *CANMessageID, float information){

	/*
	 Function used to send back data from ESCs to the master. The ID of the information that is being sent back is identical to the one
	 that was sent to the esc, except the MSB is now 1, indicating that it is the esc which is sending data back to the master. In turn, this also
	 makes sure that no other escs begin processing the message that the current one is trying to send.

	 */

//    FDCAN_TxHeaderTypeDef txHeader;
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
    uint16_t txID = 0;
    uint8_t txData[8];


    uint8_t spec = (CANMessageID->commandType == ACTION_READ)
                   ? (uint8_t)(CANMessageID->readSpec)
                   : (uint8_t)(CANMessageID->runSpec);


    // Configure the ID of the message according to what was received
    txID |= (0 & 0x01) << SENDER_DEVICE_SHIFT;
    // TONY: I changed this to zero since I want the sent can response to act as a master
    // This caluevis now always 1 since the esc is sending data.
    txID |= (CANMessageID->motorType & 0x01) << NDRIVE_STEERING_SHIFT;
    txID |= (CANMessageID->motorConfig & 0x01) << NMULTI_SINGLE_SHIFT;
    txID |= (CANMessageID->commandType & 0x01) << NACTION_READ_ID_DEVICE_SHIFT;
    txID |= (CANMessageID->readSpec & 0x07) << MSG_SPECIFICATION_SHIFT;
    txID |= (CANMessageID->motorID & 0x0f);

    //TONY: I want the txID to be exactly what I specified in main.c
    txID = CANMessageID;
    // Move the information into the data paylaod
    memcpy(txData, &information, sizeof(float)); // data[0] --> data[3] now stores float


	////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 uart_debug_print("CAN Command Sent back!\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

    //format other message peripherals
//    txHeader.Identifier          = txID;
//    txHeader.IdType              = FDCAN_STANDARD_ID;
//    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
//    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
//    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
//    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
//    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
//    txHeader.MessageMarker       = 0;

	 TxHeader.DLC = 8;
	 TxHeader.ExtId = 0;
	 TxHeader.IDE = CAN_ID_STD;
	 TxHeader.RTR = CAN_RTR_DATA;
	 TxHeader.StdId = txID;

	// Send the response

	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, txData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

//    // Send the response
//    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK) {
//        Error_Handler();
//    }
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
    uint16_t offset = STEERING_ID * 2;
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
}

