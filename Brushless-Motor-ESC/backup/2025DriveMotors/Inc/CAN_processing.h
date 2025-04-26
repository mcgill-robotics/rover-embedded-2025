// Defines
#ifndef CAN_processing_H
#define CAN_processing_H


#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

// Defining variables

//Motor parameters --> Get these from the profiled motor!!
static float g_maxTorque   = 0.400f;    // [N·m]  conservative value
static float g_inertia     = 0.00001242f; // [kg·m^2]
static float g_speedThresh = 50.0f;    // threshold below which we treat speed as zero

extern int ESC_ID;
int s_previousDirection = 0 ; // 0 means idle, 1 means forward, -1 means backward


// Enumaeration classes for different possible CAN Message casess
typedef enum {
    MASTER			= 0,
    SLAVE			= 1
} Transmitter;

typedef enum {
    RUN			= 0,
    READ		= 1
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
    STOP               	= 0,
	ACKNOWLEDGE_FAULTS 	= 1,
    SPEED              	= 2,
    POSITION           	= 3,
} RunSpec;

typedef enum {
    SPEED             	 = 0,
    POSITION          	 = 1,
    VOLTAGE           	 = 2,
    CURRENT           	 = 3,
    GET_ALL_FAULTS    	 = 4,
    GET_CURRENT_STATE    = 5,
	GET_TEMPERATURE	  	 = 6,
	GET_PING			 = 7
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



//Function prototypes
void CAN_Parse_MSG (FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void Process_Multiple_ESC_Command (ParsedCANID *parsedMessageID, uint8_t *rxData);
void Process_Single_ESC_Command (ParsedCANID *CANMessageID, uint8_t *rxData);
void sendCANResponse(ParsedCANID *CANMessageID, float information);
float SingleExtractFloatFromCAN(uint8_t *data);
void runSingleMotor(float newSpeed)
uint16_t computeRampTimeMs(float currentSpeedRpm, float targetSpeedRpm);


