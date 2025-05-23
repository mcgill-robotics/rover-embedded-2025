/*
 * This file contains all of the logic which is used to properly receive and send CAN commands on the ESC. The way each message is decoded, and the breakdown
 * off all the bits in each message can be found in the Rover Drive System Documentation.
 *
 * Made by: James Di Sciullo
 */

#include "CAN_processing.h"

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

	struct ParsedCANID CANMessage; //initialize a struct for the received message
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
	if (CANMessage.commandType == RUN){
			CANMessage.runSpec = (RunSpec) get_CAN_SPEC(msg_ID);
	}
	else{
		CANMessage.readSpec = (ReadSpec) get_CAN_SPEC(msg_ID);
	}

	// Determine if the command effects a single, or mulitple motors, and call the appropriate helper functions
	CANMessage.motorConfig = (MotorConfig) get_CAN_motor_mov_type(msg_ID);
	if (CANMessage.motorConfig == SINGLE){
		CANMessage.motorID = (MotorID) get_CAN_device_ID(msg_ID);
		Process_Single_ESC_Command(&CANMessage, rxData);
	}
	else{
	Process_Multiple_ESC_Command(&CANMessage, rxData);
	}

}


void Process_Single_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData){
	//single information will always come as a float (signed 4 bytes)
	float information = SingleExtractFloatFromCAN(rxData);

	if (CANMessageID->commandType == RUN){
		switch(CANMessageID->runSpec){

		case (STOP):
				MC_StopMotor1();
				break;

		case (ACKNOWLEDGE_FAULTS):
				MC_AcknowledgeFaultMotor1();
				break;

		case (SPEED):
				MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
				runSingleMotor(information);
				break;
		}
		//otherwise ignore whatever was received :)
		break;
	}

	else{ // IT IS A READ COMMAND
		switch(CANMessageID->readSpec){

		case(SPEED):
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
				float feedback = 69;
				sendCANResponse(CANMessageID, feedback);
				break;
		}
		break; // ignore if not one of the options
	}
}

void Process_Multiple_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData){
	switch(CANMessageID->runSpec){

	case (SPEED):
			int16_t curESCSpeed = extract_multiple_speeds(rxData);
			MC_AcknowledgeFaultMotor1(); // ensuring there is no fault on run <-- sketch
			runSingleMotor(curESCSpeed);
			break;

	case (STOP):
			MC_StopMotor1();
			break;
	break;
	}


}


int16_t extract_multiple_speeds(const uint8_t *rxData)
{
    uint16_t offset = ESC_ID * 2;
    int16_t value = (int16_t)((rxData[offset + 1] << 8) | rxData[offset]);
    return value;
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


    // Configure the ID of the message according to what was received
    txID |= (1 & 0x01) << SENDER_DEVICE_SHIFT; // This calue is now always 1 since the esc is sending data.
    txID |= (CANMessageID->motorType & 0x01) << NACTION_READ_ID_DEVICE_SHIFT;
    txID |= (CANMessageID->motorConfig & 0x01) << NMULTI_SINGLE_SHIFT;
    txID |= (CANMessageID->commandType & 0x01) << NDRIVE_STEERING_SHIFT;
    txID |= (CANMessageID->commandType & 0x07) << MSG_SPECIFICATION_SHIFT;
    txID |= (CANMessageID->commandType & 0x0f);

    // Move the information into the data paylaod
    memcpy(txData, &information, sizeof(float)); // data[0] --> data[3] now stores float


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

// This function is used in order to run a single motor at a given setpoint.
void runSingleMotor(float newSpeed) {
    // Threshold to consider "almost zero" / unreliable speed feedback point
    const float SPEED_ZERO_THR = 50.0f;

    //Deadzone for small speeds
    float speedCmd = newSpeed;
    if (fabsf(speedCmd) < SPEED_ZERO_THR) {
        speedCmd = 0.0f;
    }

    MCI_State_t motorState = MC_GetSTMStateMotor1();

    // If the motor is currently RUN, check for reversal
    if (motorState == RUN)
    {
        // If previous direction was positive but new speed is negative (or opposite)
        // then first ramp to 0 before allowing opposite sign.
        bool reversing = false;
        if (s_previousDirection > 0 && speedCmd < 0) {
            reversing = true;
        }
        else if (s_previousDirection < 0 && speedCmd > 0) {
            reversing = true;
        }

        if (reversing) {
            // Stop motor in order to bring it back to zero
            MC_StopMotor1();

            //  Wait for motor to become IDLE
            while (1) {
                HAL_Delay(5); // poll the state until it IDLE
                MCI_State_t currState = MC_GetSTMStateMotor1();
                if (currState == IDLE) {
                    break;
                }
            }
        }
    }

    // Here the motor is either IDLE or continuing in same direction it was in
    motorState = MC_GetSTMStateMotor1();
    if (motorState != RUN)
    {
        if (speedCmd != 0.0f)
        {
            // Start from IDLE
            uint16_t myRampTime = (uint16_t)computeRampTimeMs(0.0f, speedCmd);
            MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime);
            s_previousDirection = (speedCmd > 0) ? 1 : -1; //upfate the previous direction global var
            if (!MC_StartMotor1()) {
                return 1; // start failed
            }
        }
        else {
            // speedCmd is 0, remain IDLE
            s_previousDirection = 0;
        }
    }
    else
    {
        // Already in RUN, so just do a new ramp if nonzero
        if (fabsf(speedCmd) < 1e-3f) // effectively zero => request stop
        {
            MC_StopMotor1();
            s_previousDirection = 0;
            return 0;
        }
        else
        {
            // Motor is running in same direction
            s_previousDirection = (speedCmd > 0) ? 1 : -1;
            uint16_t myRampTime = (uint16_t)computeRampTimeMs(MC_GetAverageMecSpeedMotor1_F(), speedCmd);
            MC_ProgramSpeedRampMotor1_F(speedCmd, myRampTime);
        }
    }

    return 0;
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

    // For safety, clamp time to at least 10 ms or so
    if (timeMs < 10.0f) {
        timeMs = 10.0f;
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




