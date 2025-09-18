/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CANOPEN_ACCEL_CONST 2166276				// 0.5g (4.9m/s^2),  Wheel Diameter : 0.180m, Gear Ratio: 25
												// = 4.9 *  25 / (0.18 * PI) = 216.6275 rev/s^2
 	 	 	 	 	 	 	 	 	 	 	 	// = rev/s^2 * Unit Resol(10000) = 2166276
#define CANOPEN_DECEL_CONST CANOPEN_ACCEL_CONST

#define CANOPEN_WHEEL_PI			3.141592654
#define CANOPEN_WHEEL_ENCODERUNIT	10000.0
#define CANOPEN_WHEEL_DIAMETER		0.160		// m
#define CANOPEN_WHEEL_GEARRATIO		25.0		// Gear Ratio
#define CANOPEN_WHEEL_CONV_radps2Munitps 	39788.735		// =  1/(2*pi()) * GearRatio * ENCODERUNIT = 39788.735
#define CANOPEN_WHEEL_CONV_Munitps2radps    (1.0/CANOPEN_WHEEL_CONV_radps2Munitps)

/* 6041h status word
0	Servo ready	1 - valid, 0 - invalid
1	Start	1 - valid, 0 - invalid
2	Servo running	1 - valid, 0 - invalid  -------------------> using
3	Fault	1 - valid, 0 - invalid          -------------------> using
4	Main circuit power on	1 - valid, 0 - invalid
5	Quick stop	0- valid, 1 - invalid
6	Servo cannot run	1 - valid, 0 - invalid
7	Warning	1 - valid, 0 - invalid
8	Reserved	Reserved
9	Remote control	1 - valid, 0 - invalid
10	Arrived at position	1 - valid, 0 - invalid
11	Internal limit valid	1 - valid, 0 - invalid
12,13	Mode related	Related to each servo operation mode
14	Reserved	Reserved
15	Origin found	1 - valid, 0 - invalid

 */
typedef enum {
    CANOPEN_NMT_CMD_ACTIVATE = 0x01,			/** 0x1, Activate remote nodes */
	CANOPEN_NMT_CMD_DEACTIVATE = 0x02,			/** 0x2, Deactivate remote nodes */
	CANOPEN_NMT_CMD_PREOPERATION = 0x80,		/** 0x80, Pre-operation */
	CANOPEN_NMT_CMD_RESETNODE = 0x81,			/** 0x81, Reset nodes */
	CANOPEN_NMT_CMD_RESETCOMMUNICATION = 0x82	/** 0x82, Reset communication */
} CANOPEN_NMT_command_t;

typedef enum {
    CANOPEN_STATUS_INITIALIZE = 0x00,		/** 0x0, Initialize */
	CANOPEN_STATUS_NOTCONNECTED = 0x01,		/** 0x1, NOT CONNECTED */
	CANOPEN_STATUS_CONNECTED = 0x02,		/** 0x2, Connected */
	CANOPEN_STATUS_READY = 0x03,			/** 0x3, Ready */
	CANOPEN_STATUS_STOP = 0x04,				/** 0x4, STOP */
	CANOPEN_STATUS_OPERATION = 0x05,		/** 0x5, OPERAION */
	CANOPEN_STATUS_RREOPERATION = 0x7F,		/** 0x7F, RREOPERATION */
} CANOPEN_NMT_reset_cmd_t;

typedef enum {
    CANOPEN_COBID_NMT = 0x000,				/**< NMT module control */
	CANOPEN_COBID_SYNC = 0x080,				/**< SYNC */
	CANOPEN_COBID_TIMESTAMNP = 0x100,		/**< TIME STAMP */

	CANOPEN_COBID_URGENT = 0x080,			/**< 0x080+Node-ID */

	CANOPEN_COBID_TXPDO1 = 0x180,			/**< 0x180+Node-ID, Transmit */
	CANOPEN_COBID_TXPDO2 = 0x280,			/**< 0x280+Node-ID, Transmit */
	CANOPEN_COBID_TXPDO3 = 0x380,			/**< 0x380+Node-ID, Transmit */
	CANOPEN_COBID_TXPDO4 = 0x480,			/**< 0x480+Node-ID, Transmit */

	CANOPEN_COBID_RXPDO1 = 0x200,			/**< 0x200+Node-ID, Receive */
	CANOPEN_COBID_RXPDO2 = 0x300,			/**< 0x300+Node-ID, Receive */
	CANOPEN_COBID_RXPDO3 = 0x400,			/**< 0x400+Node-ID, Receive */
	CANOPEN_COBID_RXPDO4 = 0x500,			/**< 0x500+Node-ID, Receive */

	CANOPEN_COBID_SDO_SERVER_TRANS = 0x580,	/**< 0x580+Node-ID, Server Transmission */
	CANOPEN_COBID_SDO_CLIENT_TRAS = 0x600,	/**< 0x600+Node-ID, Client Transmission */

	CANOPEN_COBID_NMT_ERROR = 0x700			/**< 0x700+Node-ID, NMT Error Control */
} CANOPEN_COBID_t;

typedef enum {
	Software_overcurrent = 	0x2211,
	Hardware_overcurrent =	0x2212,
	Phase_missing = 0x3130,
	PhaseA_circuit_current_detection_error = 0x3150,
	PhaseB_circuit_current_detection_error = 0x3151,
	Analog_input1_circuit_error = 0x3152,
	Motor_power_cable_not_connected	= 0x3153,
	Analog_input2_circuit_error = 0x3154,
	Excessive_analog_input1 = 0x3160,
	Excessive_analog_input2	= 0x3161,
	Excessive_analog_input3	= 0x3162,
	DCbus_base_voltage_error = 0x3201,
	Control_circuit_voltage_too_low = 0x3205,
	Control_circuit_voltage_too_high = 0x3206,
	DCbus_voltage_too_high = 0x3211,
	DCbus_voltage_too_low = 0x3221,
	Main_power_supply_disconnected = 0x3222,
	Temperature_base_sampling_error = 0x4201,
	Drive_over_temperature = 0x4210,
	Servo_unable_to_enable_under_current_mode = 0x5201
} CANOPEN_ALARM_CODE_t;		// 603Fh


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t gu32IdleLoopCount;
uint32_t gu32Tim6Count;
uint32_t gu32Can1RxCount;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CANOpen_WheelMotorDrive_Init(void);					// Initial function
void CANOpen_WheelMotorDriveParam_Set(void);				// not used
void CANOpen_BrakeReleaseForceControl(void);
void CANOpen_ServoEnableControl(void);
void CANOpen_Status_Error_Check(void);

uint8_t CANOpen_NMT_ResetNodes(uint8_t u8CanNodeID);			// '0' All
uint8_t CANOpen_NMT_ActiveNode_Set(uint8_t u8CanNodeID);		// '0' All
uint8_t CANOpen_NMT_PreOperatingNode_Set(uint8_t u8CanNodeID);	// '0' All

uint8_t CANOpen_WatchdogDisable_Set(uint8_t u8CanNodeID);		// Watch dog disable

uint8_t CANOpen_OperationModeOD_Set(uint8_t u8CanNodeID);		// Profile velocity mode set

uint8_t CANOpen_ProfileAccelerationOD_Set(uint8_t u8CanNodeID);
uint8_t CANOpen_ProfileDecelerationOD_Set(uint8_t u8CanNodeID);
uint8_t CANOpen_TargetVelocityOD_Set(uint8_t u8CanNodeID, float f32Vel_radps);
uint8_t CANOpen_TargetVelocityOD_PDOSet(uint8_t u8CanNodeID, float f32Vel_radps);

uint8_t CANOpen_ControlWordOD_ServoEnable_Set(uint8_t u8CanNodeID);		// ServoOn + Quick Stop off / Circuit PowerON / Start
uint8_t CANOpen_ControlWordOD_ServoDisable_Set(uint8_t u8CanNodeID);	// ServoOff + Quick Stop off / Circuit PowerON / Start
uint8_t CANOpen_ControlWordOD_FaultReset_Set(uint8_t u8CanNodeID);		// bit7 rising

uint8_t CANOpen_BrakeOutNormal_Set(uint8_t u8CanNodeID);					// Brake output normal
uint8_t CANOpen_BrakeOutInvert_Set(uint8_t u8CanNodeID);					// Brake output Invert

uint8_t CANOpen_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);

float   gaf32TargetVel_radps[2];		// radian per sec
float   gaf32MeasureVel_radps[2];		// radian per sec
uint16_t gau16CANOpen_Status[2];
uint16_t gau16CANOpen_ErrorCode[2];
uint8_t gau8NMTStatus[2];				// CANOPEN_NMT_reset_cmd_t

uint8_t flagActServoEnable[2] = {0};	// servo control enable, sync brake release on 	
uint8_t flagActForceBrake[2] = {0};

uint8_t flagCANOpenStateNMT[2] = {0};				// '1' : operating
uint8_t flagStateServoControl[2] = {0};		// '1' : Servo On
uint8_t flagCANOpenError[2] = {0};			// '1' : CANOpen Error

uint8_t flagStateBrakeoutInvert[2] = {0};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM6_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CANOpen_WheelMotorDrive_Init();			// CANOpen wheel Motor Drive

  HAL_TIM_Base_Start_IT(&htim6);			// CANOpen Target Velocity

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    gu32IdleLoopCount++;

	// CANOpen forced brake control
	CANOpen_ServoEnableControl();
	CANOpen_BrakeReleaseForceControl();
	
	//HAL_Delay(1);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CANOpen_WheelMotorDrive_Init(void)
{
	//step1) Reset & Active node
	CANOpen_NMT_ResetNodes(0);				    // Reset node,  return NMT msg 0x7f
	HAL_Delay(10);								// need to delay after reset node

	//step2) watch dog disable
	CANOpen_WatchdogDisable_Set(1);
	CANOpen_WatchdogDisable_Set(2);

	//step3) Active remote node
	CANOpen_NMT_ActiveNode_Set(0);			// Active remote node, return NMT msg 0x05

	//step4) Mode of Operation : Profile velocity mode
	CANOpen_OperationModeOD_Set(1);
	CANOpen_OperationModeOD_Set(2);

	//step5) Profile acceleration & Profile deceleration
	CANOpen_ProfileAccelerationOD_Set(1);
	CANOpen_ProfileDecelerationOD_Set(1);
	CANOpen_ProfileAccelerationOD_Set(2);
	CANOpen_ProfileDecelerationOD_Set(2);

	// step6) sync  flagActServoEnable 
	//CANOpen_ControlWordOD_ServoEnable_Set(1);
	//CANOpen_ControlWordOD_ServoEnable_Set(2);
}

void CANOpen_WheelMotorDriveParam_Set(void)		// It's not completed.
{
	// 0. Get into pre-opration state
	CANOpen_NMT_PreOperatingNode_Set(0);
	HAL_Delay(10);

	// 1. Watch dog disable
	CANOpen_WatchdogDisable_Set(1);
	HAL_Delay(10);
	CANOpen_WatchdogDisable_Set(2);
	HAL_Delay(10);
}

void CANOpen_ServoEnableControl(void)
{
	// Node1 
	if( (flagActServoEnable[0] == 1) && (flagStateServoControl[0] == 0))
	{
		CANOpen_ControlWordOD_ServoEnable_Set(1);
	}
	else if( (flagActServoEnable[0] == 0) && (flagStateServoControl[0] == 1))
	{
		CANOpen_ControlWordOD_ServoDisable_Set(1);
	}
   
   	//  Node2 
	if( (flagActServoEnable[1] == 1) && (flagStateServoControl[1] == 0))
	{
		CANOpen_ControlWordOD_ServoEnable_Set(2);
	}
	else if( (flagActServoEnable[1] == 0) && (flagStateServoControl[1] == 1))
	{
		CANOpen_ControlWordOD_ServoDisable_Set(2);
	}
}


void CANOpen_BrakeReleaseForceControl(void)
{
	// CANOpen Node1 forced brake
	if( (flagActForceBrake[0] == 1) && (flagStateBrakeoutInvert[0] == 0))
	{
		if(flagStateServoControl[0] == 1)
		{
			CANOpen_ControlWordOD_ServoDisable_Set(1);
		}	
		HAL_Delay(1000);																// need long delay

		CANOpen_BrakeOutInvert_Set(1);
	}
	else if( (flagActForceBrake[0] == 0) && (flagStateBrakeoutInvert[0] == 1))
	{
		CANOpen_BrakeOutNormal_Set(1);
		HAL_Delay(1000);																// need long delay

		if(flagStateServoControl[0] == 0)
		{
			CANOpen_ControlWordOD_ServoEnable_Set(1);
		}
	}
   
   	// CANOpen Node2 forced brake
	if( (flagActForceBrake[1] == 1) && (flagStateBrakeoutInvert[1] == 0))
	{
		if(flagStateServoControl[1] == 1)
		{
			CANOpen_ControlWordOD_ServoDisable_Set(2);
		}
		HAL_Delay(1000);																// need long delay

		CANOpen_BrakeOutInvert_Set(2);
	}
	else if( (flagActForceBrake[1] == 0) && (flagStateBrakeoutInvert[1] == 1))
	{
		CANOpen_BrakeOutNormal_Set(2);
		HAL_Delay(1000);																// need long delay
		if(flagStateServoControl[1] == 0)
		{
			CANOpen_ControlWordOD_ServoEnable_Set(2);
		}
	}	
}

void CANOpen_Status_Error_Check(void)
{
	// NMT Status check
	if(gau8NMTStatus[0] == 0x5)	  flagCANOpenStateNMT[0] = 1;
	else						  flagCANOpenStateNMT[0] = 0;

	if(gau8NMTStatus[1] == 0x5)	  flagCANOpenStateNMT[1] = 1;
	else						  flagCANOpenStateNMT[1] = 0;

	// Control word Status check - servo on, 6041h-bit2 : Servo running
	if( ((gau16CANOpen_Status[0]>>2) & 0x1) == 1) flagStateServoControl[0] = 1;
	else 										  flagStateServoControl[0] = 0;

	if( ((gau16CANOpen_Status[1]>>2) & 0x1) == 1) flagStateServoControl[1] = 1;
	else 										  flagStateServoControl[1] = 0;

	// Control word Status check - fault, 6041h-bit3 :fault
	if( ((gau16CANOpen_Status[0]>>3) & 0x1) == 1) flagCANOpenError[0] = 1;
	else 										  flagCANOpenError[0] = 0;

	if( ((gau16CANOpen_Status[1]>>3) & 0x1) == 1) flagCANOpenError[1] = 1;
	else										  flagCANOpenError[1] = 0;

	// Error code
	// gau16CANOpen_ErrorCode[]
}

uint8_t CANOpen_NMT_ResetNodes(uint8_t u8CanNodeID)			// '0' All
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x000;					// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x81;						// Command Word, '0x81': Reset nodes
	CanTxData[1] = 0x0;							// Node-ID,  '0' : all NMT slave nodes
	CanTxData[2] = 0x0;
	CanTxData[3] = 0x0;
	CanTxData[4] = 0x0;
	CanTxData[5] = 0x0;
	CanTxData[6] = 0x0;
	CanTxData[7] = 0x0;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_NMT_ActiveNode_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x000;						// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x1;							// Command Word, '1': Activate remote nodes
	CanTxData[1] = 0x0;							// Node-ID,  '0' : all NMT slave nodes
	CanTxData[2] = 0x0;
	CanTxData[3] = 0x0;
	CanTxData[4] = 0x0;
	CanTxData[5] = 0x0;
	CanTxData[6] = 0x0;
	CanTxData[7] = 0x0;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_NMT_PreOperatingNode_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x000;						// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x80;						// '0x80': Get into pre-operation state
	CanTxData[1] = 0x0;							// Node-ID,  '0' : all NMT slave nodes
	CanTxData[2] = 0x0;
	CanTxData[3] = 0x0;
	CanTxData[4] = 0x0;
	CanTxData[5] = 0x0;
	CanTxData[6] = 0x0;
	CanTxData[7] = 0x0;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_WatchdogDisable_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;	// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x2B;						// 2B: 16bit token
	CanTxData[1] = 0x04;						// 0x5004 , 0x06
	CanTxData[2] = 0x50;
	CanTxData[3] = 0x06;
	CanTxData[4] = 0x0;							// 0 ~ 65535msec,  '0': Disable
	CanTxData[5] = 0x0;
	CanTxData[6] = 0x0;
	CanTxData[7] = 0x0;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}


uint8_t CANOpen_OperationModeOD_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;		// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x2F;							// 2F: 8bit token
	CanTxData[1] = 0x60;							// OD: Operation Mode - 6060
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x03;							// 3: Profile velocity mode
	CanTxData[5] = 0x00;
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_ProfileAccelerationOD_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;
	uint32_t AccelVal;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	AccelVal = (uint32_t)CANOPEN_ACCEL_CONST;

	CanTxData[0] = 0x23;							// 23: 32bit token
	CanTxData[1] = 0x83;							// OD: 6083
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = (uint8_t)((AccelVal >> 0) & 0xFF); 		// data: 32bit
	CanTxData[5] = (uint8_t)((AccelVal >> 8) & 0xFF);
	CanTxData[6] = (uint8_t)((AccelVal >> 16) & 0xFF);
	CanTxData[7] = (uint8_t)((AccelVal >> 24) & 0xFF);

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_ProfileDecelerationOD_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;
	uint32_t DecelVal;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	DecelVal = (uint32_t)CANOPEN_DECEL_CONST;

	CanTxData[0] = 0x23;							// 23: 32bit token
	CanTxData[1] = 0x84;							// OD: 6084
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = (DecelVal >> 0) & 0xFF; 			// data: 32bit
	CanTxData[5] = (DecelVal >> 8) & 0xFF;
	CanTxData[6] = (DecelVal >> 16) & 0xFF;
	CanTxData[7] = (DecelVal >> 24) & 0xFF;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_TargetVelocityOD_Set(uint8_t u8CanNodeID, float f32Vel_radps)		// f32Vel_radps : radian per sec
{
	uint8_t rVal = 0;
	int32_t s32TargetVel;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	s32TargetVel = (int32_t)(f32Vel_radps * CANOPEN_WHEEL_CONV_radps2Munitps);

	CanTxData[0] = 0x23;							// 23: 32bit token
	CanTxData[1] = 0xFF;							// OD: 60FF
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = (s32TargetVel >> 0) & 0xFF; 		// data: 32bit
	CanTxData[5] = (s32TargetVel >> 8) & 0xFF;
	CanTxData[6] = (s32TargetVel >> 16) & 0xFF;
	CanTxData[7] = (s32TargetVel >> 24) & 0xFF;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

// RPDO1-1, Targer Velocity
uint8_t CANOpen_TargetVelocityOD_PDOSet(uint8_t u8CanNodeID, float f32Vel_radps)		// f32Vel_radps : radian per sec,
{
	uint8_t rVal = 0;
	int32_t s32TargetVel;

	CanTxHeader.StdId = 0x200 + u8CanNodeID;			// RPDO1
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	s32TargetVel = (int32_t)(f32Vel_radps * CANOPEN_WHEEL_CONV_radps2Munitps);

	CanTxData[0] = (s32TargetVel >> 0) & 0xFF;
	CanTxData[1] = (s32TargetVel >> 8) & 0xFF;
	CanTxData[2] = (s32TargetVel >> 16) & 0xFF;
	CanTxData[3] = (s32TargetVel >> 24) & 0xFF;
	CanTxData[4] = 0x00;
	CanTxData[5] = 0x00;
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}



uint8_t CANOpen_ControlWordOD_ServoEnable_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x2B;							// 2B: 16bit token
	CanTxData[1] = 0x40;							// OD: Control Word - 6040
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x0F;							// data: 16bit
	CanTxData[5] = 0x00;							// 3: '1'servo running, 2: '0' quick stop, 1: '1' circuit power on, 0: '1' Start
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_ControlWordOD_ServoDisable_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x2B;							// 2B: 16bit token
	CanTxData[1] = 0x40;							// OD: Control Word - 6040
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x07;							// data: 16bit
	CanTxData[5] = 0x00;							// 3: '1'servo running, 2: '0' quick stop, 1: '1' circuit power on, 0: '1' Start
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

uint8_t CANOpen_ControlWordOD_FaultReset_Set(uint8_t u8CanNodeID)
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x2B;							// 2B: 16bit token
	CanTxData[1] = 0x40;							// OD: Control Word - 6040
	CanTxData[2] = 0x60;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x80;							// data: 16bit
	CanTxData[5] = 0x00;							// bit7 : rising then
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	flagStateServoControl[u8CanNodeID - 1] = 0;

	return rVal;
}

uint8_t CANOpen_BrakeOutInvert_Set(uint8_t u8CanNodeID)			// Brake Release & Servo Off
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x23;							// 23: 32bit token
	CanTxData[1] = 0x12;							// OD: 
	CanTxData[2] = 0x24;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x83;							// 83h : Normally close
	CanTxData[5] = 0x00;							
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

    flagStateBrakeoutInvert[u8CanNodeID-1] = 1;
	
	return rVal;
}

uint8_t CANOpen_BrakeOutNormal_Set(uint8_t u8CanNodeID)		// Brake Release of & Servo On
{
	uint8_t rVal = 0;

	CanTxHeader.StdId = 0x600 + u8CanNodeID;			// COBID
	CanTxHeader.ExtId = 0x00;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.DLC = 8;
	CanTxHeader.TransmitGlobalTime = DISABLE;

	CanTxData[0] = 0x23;							// 23: 32bit token
	CanTxData[1] = 0x12;							// OD: 
	CanTxData[2] = 0x24;
	CanTxData[3] = 0x00;							// Sub-index
	CanTxData[4] = 0x03;							// 03h : Normally Open
	CanTxData[5] = 0x00;							
	CanTxData[6] = 0x00;
	CanTxData[7] = 0x00;

	rVal = CANOpen_TxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	flagStateBrakeoutInvert[u8CanNodeID-1] = 0;

	return rVal;
}

//  HAL_OK       = 0x00U,HAL_ERROR    = 0x01U,HAL_BUSY     = 0x02U,HAL_TIMEOUT  = 0x03U
uint8_t CANOpen_TxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
	uint8_t rVal = 0;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}

	rVal = HAL_CAN_AddTxMessage(&hcan1, &CanTxHeader, CanTxData, &CanTxMailbox);

	return rVal;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  gu32Can1RxCount++;

  int32_t s32MeasureVel_unitps;

  // Get RX message
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, CanRxData) != HAL_OK)
  {
    Error_Handler();
  }

  // Message Parsing
  if (CanRxHeader.StdId == 0x701)		// NMT message - every 1sec
  {
	  gau8NMTStatus[0] = CanRxData[0];
  }
  else if(CanRxHeader.StdId == 0x702)
  {
	  gau8NMTStatus[1] = CanRxData[0];
  }
  else if(CanRxHeader.StdId == 0x181)
  {
	  // velocity: 32bit, status: 16bit, Error:16bit
	  s32MeasureVel_unitps = (int32_t)((CanRxData[3]<<24) | (CanRxData[2]<<16) | (CanRxData[1]<<8) | CanRxData[0]<<0);
	  gaf32MeasureVel_radps[0] = (float)s32MeasureVel_unitps * CANOPEN_WHEEL_CONV_Munitps2radps;

	  gau16CANOpen_Status[0] = (uint16_t)((CanRxData[5]<<8) | (CanRxData[4]<<0));

	  gau16CANOpen_ErrorCode[0] = (uint16_t)((CanRxData[7]<<8) | (CanRxData[6]<<0));
  }
  else if(CanRxHeader.StdId == 0x182)
  {
	  // velocity: 32bit, status: 16bit, Error:16bit
	  s32MeasureVel_unitps = (int32_t)((CanRxData[3]<<24) | (CanRxData[2]<<16) | (CanRxData[1]<<8) | CanRxData[0]<<0);
	  gaf32MeasureVel_radps[1] = (float)s32MeasureVel_unitps * CANOPEN_WHEEL_CONV_Munitps2radps;

	  gau16CANOpen_Status[1] = (uint16_t)((CanRxData[5]<<8) | (CanRxData[4]<<0));

	  gau16CANOpen_ErrorCode[1] = (uint16_t)((CanRxData[7]<<8) | (CanRxData[6]<<0));
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{  
  if(htim->Instance == htim6.Instance) 
  {    
    gu32Tim6Count++;

    if(gu32Tim6Count % 10 == 0 )
    {	
		// CANOpen Control
		if(flagStateServoControl[0] == 1 )
		{
			CANOpen_TargetVelocityOD_PDOSet(1, gaf32TargetVel_radps[0]);
		}
		if(flagStateServoControl[1] == 1 )
		{
    		CANOpen_TargetVelocityOD_PDOSet(2, gaf32TargetVel_radps[1]);
		}

		// CANOpen status & error check
		CANOpen_Status_Error_Check();
    }

    if(gu32Tim6Count % 1000 == 0 )
    {
        HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin);
    }
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

#ifdef  USE_FULL_ASSERT
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
