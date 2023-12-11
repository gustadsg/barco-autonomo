/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "SERVO.h"
#include "DCMOTOR.h"
#include "JDY18.h"
#include "POSITIONING_BLE.h"
#include "HMC5883L.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SERVO DEFINITIONS
#define SERVO_PERIOD 1250;
#define SERVO_MIN_DUTY_CICLE 0.05;
#define SERVO_MAX_DUTY_CICLE 0.115;
#define SERVO_CALIBRATION_GAIN 1.47;
#define SERVO_CALIBRATION_OFFSET -12.6;

// SERVO CONTROLLER DEFINITIONS
#define SERVO_CONTROLLER_KP 1
#define SERVO_CONTROLLER_KI 0
#define SERVO_CONTROLLER_KD 0

// CYCLE PERIOD IN MILLIS
#define CYCLE_PERIOD_MS 3000

// DCMOTOR DEFINITIONS
#define DCMOTOR_PERIOD 1250;

// POSITIONING DEFINITIONS
#define DEPART_X 14.495
#define DEPART_Y 34.342

#define ARRIVAL_X 0
#define ARRIVAL_Y 0

#define OTHER_X -12.64
#define OTHER_Y 16.948

// FIRST STATE TIME
#define FIRST_STATE_MS 23000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void servoConfigFactory(SERVO_Config_t *config);
void dcMotorConfigFactory(DCMOTOR_Config_t *config);
void positioningBleConfigFactory(POSITIONING_BLE_Devices_Info_t *devicesInfo);
void magnetometerConfigFactory(HMC5883L_Config_t *config);
void servoPidControllerFactory(PID_Controller_t *controller);
void startInitialState(DCMOTOR_Config_t dcMotorConfig, SERVO_Config_t servoConfig);
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20000);

	SERVO_Config_t servoConfig;
	servoConfigFactory(&servoConfig);
//
//	DCMOTOR_Config_t dcMotorConfig;
//	dcMotorConfigFactory(&dcMotorConfig);

	JDY18_Device_t devices[JDY18_MAX_DEVICES];
	JDY18_Setup(&huart3);
//	JDY18_SetRole(JDY18_ROLE_MASTER);
//	JDY18_SetBaudRate(JDY18_Baud_115200);

//	JDY18_Scan(devices);

	POSITIONING_BLE_Config_t positiningBleConfig;
	POSITIONING_BLE_Devices_Info_t devicesInfo;
	positioningBleConfigFactory(&devicesInfo);
	POSITIONING_BLE_Cartesian_Point_t currentPosition;
	currentPosition.x = 0;
	currentPosition.y = 0;

	POSITIONING_BLE_Cartesian_Point_t targetPoint = {0,0};

	DCMOTOR_Config_t dcMotorConfig;
	dcMotorConfigFactory(&dcMotorConfig);
    DCMOTOR_SetSpeedPercentage(dcMotorConfig, 0);

	HMC5883L_Config_t magnetometerConfig;
	magnetometerConfigFactory(&magnetometerConfig);

	HMC5883L_Init(magnetometerConfig);
	//HMC5883L_GetCalibrationData(magnetometerConfig, &huart2);
	HMC5883L_Data_t data;
	data.x = 0;
	data.y = 0;
	data.z = 0;
	data.degrees = 0;
	data.radians = 0;

	PID_Controller_t servoPidController;

	HMC5883L_Data_t currentAngleData;
	float currentAngle = 0;
	float angleSetpoint = 0;
	float angleControlAction = 0;
	int numBleDevices = 0;
	float error = 0;

	startInitialState(dcMotorConfig, servoConfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// position reading
//		numBleDevices = JDY18_Scan(devices);
//		POSITIONING_BLE_CreateConfig(&positiningBleConfig, devicesInfo, devices, numBleDevices);
//		POSITIONING_BLE_Cartesian_Point_t currentPosition = POSITIONING_BLE_GetPosition(&positiningBleConfig);

		currentPosition.x = 14.75;
		currentPosition.y = 33.11;
		// angle reading
		angleSetpoint = POSITIONING_BLE_CalculateDesiredAngleSetpoint(currentPosition, targetPoint);

		HMC5883L_Read(magnetometerConfig, &currentAngleData);
		currentAngle = currentAngleData.degrees;

		// angle processing
//		PID_SetSetpoint(&servoPidController, angleSetpoint);
//		PID_ProcessInput(&servoPidController, currentAngle);
//		angleControlAction = PID_CalculateControlAction(&servoPidController);

		if(currentAngle < 0) currentAngle+=360;
		error = angleSetpoint - currentAngle;

//		if(error<-90) error=-90;
//		if(error>90) error = 90;

		angleControlAction = error;

		// angle writing
		SERVO_SetAngle(servoConfig, angleControlAction);

		HAL_Delay(CYCLE_PERIOD_MS);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 128-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1250;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1250;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L293D_LATCH_Pin|L293D_EN_Pin|L293D_SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L293D_CLK_GPIO_Port, L293D_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L293D_LATCH_Pin L293D_EN_Pin L293D_SER_Pin */
  GPIO_InitStruct.Pin = L293D_LATCH_Pin|L293D_EN_Pin|L293D_SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L293D_CLK_Pin */
  GPIO_InitStruct.Pin = L293D_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L293D_CLK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void servoConfigFactory(SERVO_Config_t *config) {
	SERVO_TimerConfig_t servoPWMConfig;
	servoPWMConfig.handle = htim4;
	servoPWMConfig.channel = TIM_CHANNEL_1;
	const uint32_t servoPeriod = SERVO_PERIOD
	;
	servoPWMConfig.period = servoPeriod;
	const float servoMinDutyCicle = SERVO_MIN_DUTY_CICLE
	;
	const float servoMaxDutyCicle = SERVO_MAX_DUTY_CICLE
	;
	servoPWMConfig.minDutyCyclePercentage = servoMinDutyCicle;
	servoPWMConfig.maxDutyCyclePercentage = servoMaxDutyCicle;

	SERVO_Calibration_t servoCalibration;
	const float servoCalibrationGain = SERVO_CALIBRATION_GAIN
	;
	const float servoOffset = SERVO_CALIBRATION_OFFSET
	;
	servoCalibration.gain = servoCalibrationGain;
	servoCalibration.offset = servoOffset;

	config->timerConfig = servoPWMConfig;
	config->calibration = servoCalibration;
}

void dcMotorConfigFactory(DCMOTOR_Config_t* config) {
	int dcMotorPeriod = DCMOTOR_PERIOD;
	DCMOTOR_TimerConfig_t dcmotorTimerConfig;
	dcmotorTimerConfig.channel = TIM_CHANNEL_2;
	dcmotorTimerConfig.handle = htim3;
	dcmotorTimerConfig.period = dcMotorPeriod;

	config->timerConfig = dcmotorTimerConfig;
	config->clk.GPIOx = L293D_CLK_GPIO_Port;
	config->clk.GPIO_Pin = L293D_CLK_Pin;
	config->enable.GPIOx = L293D_EN_GPIO_Port;
	config->enable.GPIO_Pin = L293D_EN_Pin;
	config->latch.GPIOx = L293D_LATCH_GPIO_Port;
	config->latch.GPIO_Pin = L293D_LATCH_Pin;
	config->data.GPIOx = L293D_SER_GPIO_Port;
	config->data.GPIO_Pin = L293D_SER_Pin;
}

void positioningBleConfigFactory(POSITIONING_BLE_Devices_Info_t *devicesInfo) {
	strcpy(devicesInfo->departureDevice.name,"PSE2022_B1");
	devicesInfo->departureDevice.x = DEPART_X;
	devicesInfo->departureDevice.y = DEPART_Y;

	strcpy(devicesInfo->arrivalDevice.name,"PSE2022_B2");
	devicesInfo->arrivalDevice.x = ARRIVAL_X;
	devicesInfo->arrivalDevice.y = ARRIVAL_Y;

	strcpy(devicesInfo->otherDevice.name,"PSE2022_B3");
	devicesInfo->otherDevice.x = OTHER_X;
	devicesInfo->otherDevice.y = OTHER_Y;
}

void magnetometerConfigFactory(HMC5883L_Config_t *config) {
	config->dataOutputRate = HMC5883L_DOR_15;
	config->gain = HMC5883L_GAIN_0_88;
	config->measurementMode = HMC5883L_MESUAREMENT_NORMAL;
	config->operatingMode = HMC5883L_CONTINUOUS_MODE;
	config->samplesNum = HMC5883L_SAMPLES_8;
	config->handle = &hi2c1;

	// Final axis data = read - offset
	config->calibration.x_offset = -10.6425;
	config->calibration.y_offset = -22.8545;
	config->calibration.z_offset = -71.7215; // don't care, only xy plane is relevant for boat navigation
}

void servoPidControllerFactory(PID_Controller_t *controller) {
	float kp = (float) SERVO_CONTROLLER_KP;
	float ki = (float) SERVO_CONTROLLER_KI;
	float kd = (float) SERVO_CONTROLLER_KD;

	float servoMax = (float) SERVO_MAX_ANGLE;
	float servoMin = (float) SERVO_MIN_ANGLE;

	PID_Create(controller, kp, ki, kd, CYCLE_PERIOD_MS);
	PID_SetSaturationLimits(controller, servoMin, servoMax);
}

/**
 * The trajectory begins with 10s of motor pwm 100% on and with middle servo position.
 */
void startInitialState(DCMOTOR_Config_t dcMotorConfig, SERVO_Config_t servoConfig) {
	int dcMotorInitialSpeed = 100;

	DCMOTOR_SetDirection(dcMotorConfig, FORWARD);
	DCMOTOR_SetSpeedPercentage(dcMotorConfig, dcMotorInitialSpeed);

	float servoInitialAngle = (float) SERVO_MIDDLE_ANGLE;
	SERVO_SetAngle(servoConfig, servoInitialAngle);

	HAL_Delay(FIRST_STATE_MS);
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
	while (1) {
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
