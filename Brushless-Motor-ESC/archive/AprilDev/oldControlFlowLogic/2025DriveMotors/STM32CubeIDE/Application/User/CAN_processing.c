/*
 * This file contains all of the logic which is used to properly receive and send CAN commands on the ESC. The way each message is decoded, and the breakdown
 * off all the bits in each message can be found in the Rover Drive System Documentation.
 *
 * Made by: James Di Sciullo
 */
#include <stdint.h>
#include "CAN_processing.h"
#include "uart_debugging.h"
#include <stdbool.h>


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

// Variable declarations
static float g_lastCommandedSpeed = 0;
int s_previousDirection = 0 ; // 0 means idle, 1 means forward, -1 means backward
extern UART_HandleTypeDef huart2;
int DELTA_SPEED_THRESH = 200; // Threshold to clip differing speed commands to
// Threshold to consider "almost zero" / unreliable speed feedback point
const float SPEED_ZERO_THR = 50.0f;
const float MAX_SPEED_THR = 3200.0f;


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
int get_CAN_device_ID (int CAN_ID) {
	return (CAN_ID & ID_DEVICE_MASK);
}

/*
 * This function includes the main control flow for CAN commands.
 */
void CAN_Parse_MSG (FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData){

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	uart_debug_print("Parsing the ID...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////


	ParsedCANID CANMessage; //initialize a struct for the received message
	uint16_t msg_ID = rxHeader->Identifier & 0x07ff ; // We only care about the first 11 bits here

	// First check to see who is transmitting --> ESCS cannot command other ECSs
	CANMessage.messageSender = (Transmitter) get_CAN_transmitter(msg_ID);
	if (CANMessage.messageSender == SLAVE){
		return;
	}

	//Check to make sure the command is directed toward the ESC
	CANMessage.motorType = (MotorType) get_CAN_motor_type(msg_ID);
	if (CANMessage.motorType == STEERING_MOTOR){
		return;
	}

	// Check the type of action to determine which field of the struct needs to be filled in
	CANMessage.commandType = (Action) get_CAN_action(msg_ID);
	if (CANMessage.commandType == ACTION_RUN){
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		uart_debug_print("Run Command Detected\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

			CANMessage.runSpec = (RunSpec) get_CAN_SPEC(msg_ID);
	}
	else{
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		uart_debug_print("Read Command Detected\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		CANMessage.readSpec = (ReadSpec) get_CAN_SPEC(msg_ID);
	}

	// Determine if the command effects a single, or mulitple motors, and call the appropriate helper functions
	CANMessage.motorConfig = (MotorConfig) get_CAN_motor_mov_type(msg_ID);
	if (CANMessage.motorConfig == SINGLE_MOTOR){
		CANMessage.motorID = (MotorID) get_CAN_device_ID(msg_ID);
		if (CANMessage.motorID == ESC_ID){

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			uart_debug_print("Processing Single Command\r\n");
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			Process_Single_ESC_Command(&CANMessage, rxData);
		}
		else {
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			 uart_debug_print("Not My IDr\n");

			//////////////////////////////////////////////////////////////////////////////////////////////////////
			return;
		}
	}
	else{
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		uart_debug_print("Processing Multiple Commands\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		Process_Multiple_ESC_Command(&CANMessage, rxData);
	}

}


void Process_Single_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData){
//	uart_debug_print("Im in can things ouuu \r\n");

	//single information will always come as a float (signed 4 bytes)
	float information = SingleExtractFloatFromCAN(rxData);

	if (CANMessageID->commandType == ACTION_RUN){
		switch(CANMessageID->runSpec){

		case (RUN_STOP):
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				uart_debug_print("Motor Stopped \r\n");
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				safeStopMotor(MC_GetAverageMecSpeedMotor1_F());
				break;

		case (RUN_ACKNOWLEDGE_FAULTS):
				MC_AcknowledgeFaultMotor1();
				break;

		case (RUN_SPEED):
				uart_debug_print("In case RUN_SPEED\r\n");
//				MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
				runSingleMotor(information);
				break;
		break;
		}
		//otherwise ignore whatever was received :)
	}

	else{ // IT IS A READ COMMAND
		switch(CANMessageID->readSpec){

		case(READ_SPEED):
				float currentSpeed = MC_GetMecSpeedReferenceMotor1_F();
				sendCANResponse(CANMessageID, currentSpeed);
				break;
		case(VOLTAGE):
				float phaseVoltage = MC_GetPhaseVoltageAmplitudeMotor1();
				sendCANResponse(CANMessageID, phaseVoltage);
				break;
		case(CURRENT):
				float phaseCurrent = MC_GetPhaseCurrentAmplitudeMotor1();
				sendCANResponse(CANMessageID, phaseCurrent);
				break;
		case(GET_ALL_FAULTS):
				float currentFaults = MC_GetOccurredFaultsMotor1();
				sendCANResponse(CANMessageID, currentFaults);
				break;
		case(GET_CURRENT_STATE):
				float currentState = MC_GetSTMStateMotor1();
				sendCANResponse(CANMessageID, currentState);
				break;
		case(GET_TEMPERATURE):
				//TODO --> Might need to poll the gpio, check the datasheed later
				break;
		case(GET_PING):
				float feedback = 69;// ;)
				sendCANResponse(CANMessageID, feedback);
				break;
		break; // ignore if not one of the options
		}

	}
}

void Process_Multiple_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData){
	switch(CANMessageID->runSpec){
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	 uart_debug_print("Running Multiple Motors...\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	case (RUN_SPEED):


			int16_t curESCSpeed = extract_multiple_speeds(rxData);

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			 uart_debug_print("Running This Motor\r\n");
			 uart_debug_print("Setpoint %d RPM\r\n", (int)curESCSpeed);
			 uart_debug_print("Previous Direction %d\r\n", (int)s_previousDirection);
			////////////////////////////////////////////////////////////////////////////////////////////////////////

//			MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
			runSingleMotor(curESCSpeed);
			break;

	case (RUN_STOP):
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			 uart_debug_print("Stop this motor\r\n");
			////////////////////////////////////////////////////////////////////////////////////////////////////////

	 	 	safeStopMotor(MC_GetAverageMecSpeedMotor1_F());
			break;
	break;
	}


}


int16_t extract_multiple_speeds(const uint8_t *rxData){
    uint16_t offset = ESC_ID * 2;
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
}


float speedCheck(float targetSpeed){
	/*
	 * The main purpose of this function is to check 2 things, the first is if the speed demanded is much greater than the previous speed, and if so, to adjust
	 * the change in speed such that it is not too great and will cause too high of a voltage spike to fault the motor. THe other function is to make sure that the speed
	 * setpoint is something that is actually a setpoint (like within the motors actual parameters) in case anything in the sending logic doesnt work
	 *
	 */

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	 uart_debug_print("CHECK FOR CLIPPING\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (MC_GetSTMStateMotor1() == RUN) {
		float delta = currentSpeedSetpoint - g_lastCommandedSpeed;

		if (fabsf(delta) > DELTA_SPEED_THRESH) {
			g_lastCommandedSpeed += (delta > 0.0f ? DELTA_SPEED_THRESH : -DELTA_SPEED_THRESH);
		} else {
			g_lastCommandedSpeed = currentSpeedSetpoint;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("Clipping occured !\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		return speedCheck(g_lastCommandedSpeed);
	}
	else{
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("No chopping!\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		return speedCheck(g_lastCommandedSpeed);
	}
}

void safeStopMotor(float currentSpeedRpm){

	//First check to see if the motor was moving a significant amount before this function was being called
	if (fabsf(currentSpeedRpm) > 250){

		//Slow down through a speed ramp
		float rampTarget = (currentSpeedRpm > 0) ? 100.0f : -100.0f;
		float deltaSpeed = currentSpeedRpm - rampTarget;

		int rampDownDivisions = (int)(fabsf(deltaSpeed) / DELTA_SPEED_THRESH);
		if (rampDownDivisions < 1) rampDownDivisions = 1;  // Ensure at least one division

		float stepSize = deltaSpeed / rampDownDivisions;

		// Slowly ramp down to target before stopping
		for (int i = 1; i <= rampDownDivisions; i++) {
			float intermediateTarget = currentSpeedRpm - (stepSize * i);
			uint16_t rampTime = computeRampTimeMs(MC_GetAverageMecSpeedMotor1_F(), intermediateTarget);
			MC_ProgramSpeedRampMotor1_F(intermediateTarget, rampTime);
			HAL_Delay(rampTime);  // Wait for ramp to apply before next one
		}

		// Final ramp to fixed low-speed value to soften the last deceleration step
		uint16_t finalRampTime = computeRampTimeMs(MC_GetAverageMecSpeedMotor1_F(), rampTarget);
		MC_ProgramSpeedRampMotor1_F(rampTarget, finalRampTime);

		// Wait for motor speed to reach near 100 --> Use timeout in case this runs into an issue
		uint32_t tStart = HAL_GetTick();
		while (fabsf(MC_GetAverageMecSpeedMotor1_F()) > 120.0f) {
			if (HAL_GetTick() - tStart > 1000) break; // Timeout safety
			HAL_Delay(10);
		}

	    MC_StopMotor1();
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("Motor is now stopped\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		s_previousDirection = 0;
		g_lastCommandedSpeed = 0;
	}

	else{
		//Motor is already moving slowly/on startup
	    MC_StopMotor1();
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("Motor is now stopped\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////
	    s_previousDirection = 0;
	    g_lastCommandedSpeed = 0;
	}
}

void checkReversing(float speedCmd){
	 // If previous direction was positive but new speed is negative (or opposite)
	// then first ramp to 0 before allowing opposite sign.

	//First if its in run state, then make sure the offset is applied (edge case on startup)

	bool reversing = false;
	if (s_previousDirection > 0 && speedCmd < 0) {
		reversing = true;
	}
	else if (s_previousDirection < 0 && speedCmd > 0) {
		reversing = true;
	}

	if (reversing) {

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("REVERSING DETECTED!\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		// Stop motor in order to bring it back to zero
		 safeStopMotor(MC_GetAverageMecSpeedMotor1_F());
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		 uart_debug_print("hit1\r\n");
		////////////////////////////////////////////////////////////////////////////////////////////////////////


		//  Wait for motor to become IDLE
		uint32_t tStart = HAL_GetTick();
		while (1) {
			HAL_Delay(5); // poll the state until it IDLE
			MCI_State_t currState = MC_GetSTMStateMotor1();
			int currentFaults = MC_GetOccurredFaultsMotor1();
			////////////////////////////////////////////////////////////////////////////////////////////////////////
//			 uart_debug_print("current state is %d\r\n", currState);
//			 uart_debug_print("current fault is %d\r\n", currentFaults);
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (currState == IDLE && MC_GetAverageMecSpeedMotor1_F() == 0) {
				HAL_Delay(500); // Tune this value for seemless transition, but
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				 uart_debug_print("Motor is now stopped after direction Change\r\n");
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				 break;
			}

		    if (HAL_GetTick() - tStart > 1000) { //use 1 sec for timeout period
		        uart_debug_print("Motor failed to stop in time!!\r\n");
			    s_previousDirection = 0;
			    g_lastCommandedSpeed = 0;
		        MCI_FaultProcessing(&Mci[0], MC_DP_FAULT, 0);
		        break;
		    }

		}
	}
	return;

}
// This function is used in order to run a single motor at a given setpoint.
void runSingleMotor(float newSpeed) {

    float speedCmd = newSpeed;
    MCI_State_t motorState = MC_GetSTMStateMotor1();

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	 uart_debug_print("Running Single Motor...\r\n");
	 uart_debug_print("Setpoint %d RPM\r\n", (int)speedCmd);
	 uart_debug_print("Previous Direction %d\r\n", (int)s_previousDirection);
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	 //First Check if the speed command needs any changing
	 speedCmd = speedCheck(speedCmd); // adjust in case the inserted speed is not an option
	 if (motorState == RUN){ //account for clipping in case the next command is too crazy
		 speedCmd = clippingCheck(speedCmd);
	 }

    //Check if there is a reversal happening
    // If the motor is currently in RUN, or startup, or any ready state --> check for reversal
    if (motorState != IDLE)
    {
       checkReversing(speedCmd);
    }

    // Handle case where motor is not running
    motorState = MC_GetSTMStateMotor1();
    if (motorState != RUN) // This means it is in IDLE state, or startup
    {
    	checkReversing(speedCmd);
   	    speedCmd = speedCheck(speedCmd); // adjust in case the inserted speed is not an option
        if (speedCmd != 0.0f)
        {
            // Start from IDLE
            uint16_t myRampTime = (uint16_t)computeRampTimeMs(MC_GetAverageMecSpeedMotor1_F(), speedCmd); // TODO Chrck why its 0.0f

        	////////////////////////////////////////////////////////////////////////////////////////////////////////
        	 uart_debug_print("Motor was not moving previously\r\n");
        	 uart_debug_print("New Ramp Setpoint %d RPM\r\n", (int)speedCmd);
        	 uart_debug_print("New Ramp time allocated %d ms\r\n", (int)myRampTime);
        	////////////////////////////////////////////////////////////////////////////////////////////////////////

            MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime);
            s_previousDirection = (speedCmd > 0) ? 1 : -1; //update the previous direction global var
            g_lastCommandedSpeed = speedCmd;
            if (!MC_StartMotor1()) {
            	////////////////////////////////////////////////////////////////////////////////////////////////////////
            	 uart_debug_print("Start Failed");
            	////////////////////////////////////////////////////////////////////////////////////////////////////////
                return; // start failed
            }

        	////////////////////////////////////////////////////////////////////////////////////////////////////////
        	 uart_debug_print("Motor has been started\r\n");
        	 uart_debug_print("Direction has been updated to: %d\r\n", (int)s_previousDirection);
        	////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
        else { // setpoint was zero while idle...

            // speedCmd is 0, remain IDLE
            s_previousDirection = 0;
            g_lastCommandedSpeed = 0;
        	////////////////////////////////////////////////////////////////////////////////////////////////////////
        	 uart_debug_print("Speed Command was considered to be or is 0\r\n");
        	 uart_debug_print("Direction has been updated to: %d\r\n", (int)s_previousDirection);
        	////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
    }
    else
    	//Motor was already running, in the same direction too
    {
    	if (speedCmd == 0){
    		safeStopMotor(MC_GetAverageMecSpeedMotor1_F());

			//  Wait for motor to become IDLE
			while (1) {
				HAL_Delay(5); // poll the state until it IDLE
				MCI_State_t currState = MC_GetSTMStateMotor1();
				if (currState == IDLE && MC_GetAverageMecSpeedMotor1_F() == 0) {
					s_previousDirection =0;
					////////////////////////////////////////////////////////////////////////////////////////////////////////
					 uart_debug_print("Motor is now stopped after receiving 0 setpoint\r\n");
		        	 uart_debug_print("Direction has been updated to: %d\r\n", (int)s_previousDirection);
					////////////////////////////////////////////////////////////////////////////////////////////////////////
					 break;
				}
				HAL_Delay(350); // Tune this value for seemless transition, but
			}
    	}
    	else{

			// Motor is running in same direction
			s_previousDirection = (speedCmd > 0) ? 1 : -1;
			uint16_t myRampTime = (uint16_t)computeRampTimeMs(MC_GetAverageMecSpeedMotor1_F(), speedCmd);

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			 uart_debug_print("Motor is already moving, change setpoint in same direction\r\n");
			 uart_debug_print("New Ramp Setpoint %d RPM\r\n", (int)speedCmd);
			 uart_debug_print("New Ramp time allocated %d ms\r\n", (int)myRampTime);
			 uart_debug_print("Direction is %d ms\r\n", s_previousDirection);
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime);
            g_lastCommandedSpeed = speedCmd;
    	}
    }

    return;
}


/*
 Function used to send back data from ESCs to the master. The ID of the information that is being sent back is identical to the one
 that was sent to the esc, except the MSB is now 1, indicating that it is the esc which is sending data back to the master. In turn, this also
 makes sure that no other escs begin processing the message that the current esc is trying to send.

 */
void sendCANResponse(ParsedCANID *CANMessageID, float information){

    FDCAN_TxHeaderTypeDef txHeader;
    uint16_t txID = 0;
    uint8_t txData[8];


    uint8_t spec = (CANMessageID->commandType == ACTION_READ)
                   ? (uint8_t)(CANMessageID->readSpec)
                   : (uint8_t)(CANMessageID->runSpec);


    // Configure the ID of the message according to what was received
    txID |= (1 & 0x01) << SENDER_DEVICE_SHIFT; // This calue is now always 1 since the esc is sending data.
    txID |= (CANMessageID->motorType & 0x01) << NDRIVE_STEERING_SHIFT;
    txID |= (CANMessageID->motorConfig & 0x01) << NMULTI_SINGLE_SHIFT;
    txID |= (CANMessageID->commandType & 0x01) << NACTION_READ_ID_DEVICE_SHIFT;
    txID |= (CANMessageID->readSpec & 0x07) << MSG_SPECIFICATION_SHIFT;
    txID |= (CANMessageID->motorID & 0x0f);

    // Move the information into the data paylaod
    memcpy(txData, &information, sizeof(float)); // data[0] --> data[3] now stores float


	////////////////////////////////////////////////////////////////////////////////////////////////////////
	 uart_debug_print("CAN Command Sent back!\r\n");
	////////////////////////////////////////////////////////////////////////////////////////////////////////

    //format other message peripherals
    txHeader.Identifier          = txID;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    // Send the response
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK) {
        Error_Handler();
    }
}


uint16_t computeRampTimeMs(float currentSpeedRpm, float targetSpeedRpm){

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
    float alpha = g_maxTorque / g_inertia; // [rad/s^2]

    // Time (seconds) = delta_omega / alpha
    float deltaW = fabsf(w2 - w1);
    float timeSec = deltaW / alpha;

    // Convert to milliseconds
    float timeMs = timeSec * 1000.0f;

    // For safety, clamp time to at least 200 ms or so
    if (timeMs < 200.0f) {
        timeMs = 200.0f;
    }
    // Also clamp to an upper limit if you do not want extremely large times
    if (timeMs > 2000.0f) {
        timeMs = 2000.0f; // 20 sec max
    }

    return (uint16_t)(timeMs);
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








