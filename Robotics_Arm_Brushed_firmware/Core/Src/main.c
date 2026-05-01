/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calibration.h"
#include "CAN_processing.h"
#include "motorControl.h"
#include "encoder.h"
#include "pid.h" // use arguments to pass the specific motor used?

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim20;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

//for CAN
FDCAN_RxHeaderTypeDef RxHeader; //header for receiving msgs
uint8_t RxData[64]; //receiver data for CANFD: 64 bits

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM20_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Motor * all_motors_list[NB_MOTORS];


int CAN_flag = 0;
int LMSW1_flag_pitch_up = 0;
int LMSW2_flag_pitch_down = 0;
int LMSW5_flag_roll = 0;
int LMSW6_flag_gripper = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM20_Init();
  MX_USB_PCD_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */




  //initialize motors for all 3 motors
  Motor gripper_motor;
  Motor pitch_motor;
  Motor roll_motor;

  Motor_Encoding_Struct gripper_encoding;
  Motor_Encoding_Struct pitch_encoding;
  Motor_Encoding_Struct roll_encoding;

  /*
   * (int encoder_max_counts, int lm_sw_reset_counts, GPIO_TypeDef* limit_port, uint16_t limit_pin)
   * - encoder max counts: Find the one specific with gear ratio of motors (i.e. more than 1 MAX_COUNT)
   * - limit switch reset counts: depends how angles defined -- corresponds to 180
   */
  motor_encoding_struct_init(&gripper_encoding, 33024, 50000);
  motor_encoding_struct_init(&pitch_encoding, 33024, 50000);
  motor_encoding_struct_init(&roll_encoding, 33024, 50000);



  /* (Motor* motor, TIM_TypeDef * pwm, TIM_TypeDef * encoder, MotorName motorName,
   * 		GPIO_TypeDef* DIR_port, uint16_t DIR_pin, int kPw, int kDw)
   * - motorName: as defined by the motorName enum.
   * - kPw; // proportional gain (how far away from goal)
   * - kDw; //derivative gain (smoothing)
   */
  motor_struct_init(&gripper_motor, TIM20, TIM3, &gripper_encoding, 0, DIR_gripper_GPIO_Port, DIR_gripper_Pin, 50, 5);
  motor_struct_init(&roll_motor, TIM8, TIM4, &pitch_encoding, 2, DIR_roll_GPIO_Port, DIR_roll_Pin, 50, 5);
  motor_struct_init(&pitch_motor, TIM1, TIM5, &roll_encoding, 1, DIR_pitch_GPIO_Port, DIR_pitch_Pin, 50, 5);


  all_motors_list[0] = &gripper_motor;
  all_motors_list[1] = &roll_motor;
  all_motors_list[2] = &pitch_motor;

  //set up Encoders
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // gripper encoder
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // roll encoder
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // pitch encoder
  	  //TIM_CHANNEL_ALL: Enable both channel needed for encoder


  //Setup Motor PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // pitch PWM
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // roll PWM
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);  // gripper PWM
  // For Driver, running in PH/EN mode, PWM port (EN) always to 1
  HAL_GPIO_WritePin(PWM_pitch_GPIO_Port, PWM_pitch_Pin, 1);   // EN/IN1 = 1
  HAL_GPIO_WritePin(PWM_roll_GPIO_Port, PWM_roll_Pin, 1);   // EN/IN1 = 1
  HAL_GPIO_WritePin(PWM_gripper_GPIO_Port, PWM_gripper_Pin, 1);   // EN/IN1 = 1


  // Initialize motor state

  //setting encoder positions
  //change depending on gear ratio of the motor
  /*
   * offset such that can go 180 right and 180 left without getting to 0 counts(- values)
   */

  gripper_motor.ENCODER_type->CNT = 41744; //gripper
  roll_motor.ENCODER_type->CNT = 41744; //roll
  pitch_motor.ENCODER_type->CNT = 41744; //pitch

  set_motor_speed_raw(&gripper_motor, 0);
  set_motor_speed_raw(&pitch_motor, 0);
  set_motor_speed_raw(&roll_motor, 0);

  //change to set directions for all 3 motors
  set_motor_direction(&gripper_motor, 1);
  set_motor_direction(&pitch_motor, 1);
  set_motor_direction(&roll_motor, 1);


  set_counts(&gripper_encoding, 41744);
  set_counts(&pitch_encoding, 41744);
  set_counts(&roll_encoding, 41744);


  setPIDGoalA(&gripper_motor, 90);
  setPIDGoalA(&pitch_motor, 90);
  setPIDGoalA(&roll_motor, 90);



  CalibrateMotor(&gripper_motor); // Calibrate the motor (see Calibration.c).
  CalibrateMotor(&roll_motor);
  CalibrateMotor(&pitch_motor);


    // CAN initialization
  // Configure accept-all filter for standard IDs into FIFO0 /
      FDCAN_FilterTypeDef filter;
      filter.IdType = FDCAN_STANDARD_ID;
      //filter.IdType = FDCAN_CLASSIC_CAN;
      filter.FilterIndex = 0;
      filter.FilterType = FDCAN_FILTER_MASK;
      filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
      filter.FilterID1 = 0x000; // ID pattern: don't care /
      filter.FilterID2 = 0x000; // Mask: 0 = all bits ignored = accept all /
      if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) {
          Error_Handler();
      }

      if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
          Error_Handler();
      }

      // Activate RX FIFO0 new message notification //
      if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
              0) != HAL_OK) {
          Error_Handler();
      }




      if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
          Error_Handler();
      }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int LMSW1_buffer = 0;
  int LMSW2_buffer = 0;
  int LMSW5_buffer = 0;
  int LMSW6_buffer = 0;

  int LMSW1_isDebouncing = 0;
  int LMSW2_isDebouncing = 0;
  int LMSW5_isDebouncing = 0;
  int LMSW6_isDebouncing = 0;



  while (1)
  {
	  //MOTOR TESTING; ACTUAL CODE SHOULD START AT CAN FLAG
	  __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, 65535/2); // 50% Speed
	  //TODO: Figure out why running 50% speed causes motor jittering

	  //HAL_GPIO_WritePin(PWM_gripper_GPIO_Port, PWM_gripper_Pin, 1);   // IN1 = 1
	  //^ moved on top
	  HAL_GPIO_WritePin(DIR_gripper_GPIO_Port, DIR_gripper_Pin, 1); //  PH = 1 = go forward

//	  HAL_Delay(10000);
//
//	  __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, 65535/4);
//
//	  HAL_GPIO_WritePin(PWM_gripper_GPIO_Port, PWM_gripper_Pin, 1);   // IN1 = 1
//	  HAL_GPIO_WritePin(DIR_gripper_GPIO_Port, DIR_gripper_Pin, 0); // IN2 = 0 = go backwards
	  // TODO: Check on motor if this will actually go in opposite direction


	  HAL_GPIO_TogglePin(LED_gripper_GPIO_Port, LED_gripper_Pin);

	  HAL_Delay(10000);


	  //HAL_GPIO_TogglePin(LED_pitch_GPIO_Port, LED_pitch_Pin);
	  //HAL_GPIO_TogglePin(LED_comms_GPIO_Port , LED_comms_Pin);
	  //HAL_Delay(500); // wait 1000 ms (1 second)


	  // Process Message if available
	  if (CAN_flag){

		CAN_flag = 0;
		//HAL_GPIO_TogglePin(LED_pitch_GPIO_Port, LED_pitch_Pin);
		//HAL_GPIO_TogglePin(LED_comms_GPIO_Port , LED_comms_Pin);

		//if (steering_state != CALIBRATION){
		CAN_Parse_MSG(&RxHeader, RxData);
		//}
		// led pin to check msg processing working

		//HAL_Delay(1000);
	  }


	  if (LMSW1_flag_pitch_up){
		  int switch_state = HAL_GPIO_ReadPin(Limit_switch_1_GPIO_Port, Limit_switch_1_Pin);
			if (!switch_state){
				LMSW1_isDebouncing = 1;// 1st trigger detected: following code for debouncing
			}

		  LMSW1_flag_pitch_up = 0;
	  }

	  if (LMSW1_isDebouncing){
		 //check that at–– least 32 consecutive readings of switch = 1
		 //ensures that the switch reading is stabilized
		 int current_switch_reading = HAL_GPIO_ReadPin(Limit_switch_1_GPIO_Port, Limit_switch_1_Pin);
			LMSW1_buffer = (LMSW1_buffer<<1) | current_switch_reading;

		 //only once stable switch reading that perform switch actions
		 if (LMSW1_buffer == 0xFFFFFFFF){
			LMSW1_isDebouncing = 0;
			LMSW1_buffer = 0;
			lmsw_pitch_up_recalibrate(&pitch_motor); // actual action to do on switch
		 }
	  }



	  if (LMSW2_flag_pitch_down){
		  int switch_state = HAL_GPIO_ReadPin(Limit_switch_2_GPIO_Port, Limit_switch_2_Pin);
			if (!switch_state){
				LMSW2_isDebouncing = 1;
			}
		  LMSW2_flag_pitch_down = 0;
	  }

	  if (LMSW2_isDebouncing){
	  		 //check that at–– least 32 consecutive readings of switch = 1
	  		 //ensures that the switch reading is stabilized
	  		 int current_switch_reading = HAL_GPIO_ReadPin(Limit_switch_2_GPIO_Port, Limit_switch_2_Pin);
	  			LMSW2_buffer = (LMSW2_buffer<<1) | current_switch_reading;

	  		 //only once stable switch reading that perform switch actions
	  		 if (LMSW2_buffer == 0xFFFFFFFF){
	  			LMSW2_isDebouncing = 0;
	  			LMSW2_buffer = 0;
	  			lmsw_pitch_down_recalibrate(&pitch_motor); // actual action to do on switch
	  		 }
	  }



	  if (LMSW5_flag_roll){
		  int switch_state = HAL_GPIO_ReadPin(Limit_switch_5_GPIO_Port, Limit_switch_5_Pin);
			if (!switch_state){
				LMSW5_isDebouncing = 1;
			}
		  LMSW5_flag_roll = 0;
	  }
	  if (LMSW5_isDebouncing){
	  		 //check that at–– least 32 consecutive readings of switch = 1
	  		 //ensures that the switch reading is stabilized
	  		 int current_switch_reading = HAL_GPIO_ReadPin(Limit_switch_5_GPIO_Port, Limit_switch_5_Pin);
	  			LMSW5_buffer = (LMSW5_buffer<<1) | current_switch_reading;

	  		 //only once stable switch reading that perform switch actions
	  		 if (LMSW5_buffer == 0xFFFFFFFF){
	  			LMSW5_isDebouncing = 0;
	  			LMSW5_buffer = 0;
	  			lmsw_roll_recalibrate(&roll_motor); // actual action to do on switch
	  		 }
	  }


	  if (LMSW6_flag_gripper){
		  int switch_state = HAL_GPIO_ReadPin(Limit_switch_6_GPIO_Port, Limit_switch_6_Pin);
			if (!switch_state){
				LMSW6_isDebouncing = 1;
			}
		  LMSW6_flag_gripper = 0;
	  }

	  if (LMSW6_isDebouncing){
	  		 //check that at–– least 32 consecutive readings of switch = 1
	  		 //ensures that the switch reading is stabilized
	  		 int current_switch_reading = HAL_GPIO_ReadPin(Limit_switch_6_GPIO_Port, Limit_switch_6_Pin);
	  			LMSW6_buffer = (LMSW6_buffer<<1) | current_switch_reading;

	  		 //only once stable switch reading that perform switch actions
	  		 if (LMSW6_buffer == 0xFFFFFFFF){
	  			LMSW6_isDebouncing = 0;
	  			LMSW6_buffer = 0;
	  			lmsw_gripper_recalibrate(&gripper_motor); // actual action to do on switch
	  		 }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 2;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 3;
  hfdcan2.Init.DataTimeSeg1 = 13;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10C18DCC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 71;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 65535;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim20, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_pitch_Pin|LED_pitch_Pin|LED_roll_Pin|LED_gripper_Pin
                          |Lazer_ctrl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_gripper_GPIO_Port, DIR_gripper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_roll_Pin|LED_comms_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FAULT_pitch_Pin FAULT_roll_Pin */
  GPIO_InitStruct.Pin = FAULT_pitch_Pin|FAULT_roll_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_gripper_Pin */
  GPIO_InitStruct.Pin = FAULT_gripper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_gripper_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_pitch_Pin LED_pitch_Pin LED_roll_Pin LED_gripper_Pin
                           Lazer_ctrl_Pin */
  GPIO_InitStruct.Pin = DIR_pitch_Pin|LED_pitch_Pin|LED_roll_Pin|LED_gripper_Pin
                          |Lazer_ctrl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_gripper_Pin */
  GPIO_InitStruct.Pin = DIR_gripper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_gripper_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ToF_int_Pin */
  GPIO_InitStruct.Pin = ToF_int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ToF_int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_roll_Pin LED_comms_Pin */
  GPIO_InitStruct.Pin = DIR_roll_Pin|LED_comms_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_sense_Pin */
  GPIO_InitStruct.Pin = VBUS_sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_sense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_switch_6_Pin Limit_switch_5_Pin Limit_switch_4_Pin */
  GPIO_InitStruct.Pin = Limit_switch_6_Pin|Limit_switch_5_Pin|Limit_switch_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Limit_switch_3_Pin */
  GPIO_InitStruct.Pin = Limit_switch_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Limit_switch_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_switch_2_Pin Limit_switch_1_Pin */
  GPIO_InitStruct.Pin = Limit_switch_2_Pin|Limit_switch_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_FDCAN_RxFifo0Callback (FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
		//check that RX FIFO 0 received a new message
	  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	  {
	    /* Retrieve Rx messages from RX FIFO0, stores into RxHeader and RxData */
	    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	    {
	    Error_Handler();
	    }

	    //upon successful interrupt, the CAN flag is set high for main loop to catch
	    //HAL_GPIO_TogglePin(LED_comms_GPIO_Port , LED_comms_Pin);
	    CAN_flag = 1;

	  }
	}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == Limit_switch_1_Pin){
		LMSW1_flag_pitch_up = 1;
	}else if (GPIO_Pin == Limit_switch_2_Pin){
		LMSW2_flag_pitch_down = 1;
	}else if (GPIO_Pin == Limit_switch_5_Pin){
		LMSW5_flag_roll = 1;
	}else if (GPIO_Pin == Limit_switch_6_Pin){
		LMSW6_flag_gripper = 1;
	}else{
		__NOP(); //callback problem
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
