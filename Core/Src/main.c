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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "MessageSend.h"
#include "PID_Controller.h"
#include "UART_DataProcess.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*----------------------------Wifi----------------------------*/
#define AT         "AT\r\n"
#define ATE        "ATE0\r\n"
#define RESTORE    "AT+RESTORE\r\n"
#define CWMODE     "AT+CWMODE=2\r\n"
#define CWSAP      "AT+CWSAP=\"esp_test\",\"1234567890\",5,3\r\n"
#define CIPMUX     "AT+CIPMUX=1\r\n"
#define CIPSERVER  "AT+CIPSERVER=1,8080\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ControlMessage ControlMsg = {0};

uint16_t CalibrationSpeed[20] = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };

uint16_t TargetPWM_Coefficient[2][20] = {
  { 7419, 7926, 9093, 10239, 11568, 12663, 13709, 14952, 16235, 17686, 19773, 21536, 23592, 26488, 28864, 31445, 34393, 36772, 39053, 41571},
  {0}
};

double Coefficient[2][6] = {
  { 6393.2863777406f, 150.8072828816f,  3.9744715635f, -0.1313606797f,  0.0022325135f, -0.0000111731f },
  { 6559.9999999987f, 303.4261072263f, -6.5722610723f,  0.1653146853f, -0.0018135198f,  0.0000082051f }
};

PWM_LimitCalibration_t PWM_LimitCal = {0};

ControlINFO_t ControlINFO;

Encoder_t Encoder[2]   = {0};
PID_t     MOTOR_PID[2] = {0};
uint8_t   speed        =  0;

//Control_flag_t Control_flag = {false};
MPU6050_t      MPU6050      = {0};
Count_t        Count        = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void WIFI_Init(void);
static void MessageProcess(void);
static void MOTOR_Control(void);

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
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /*-------------------------------- UART Init ----------------------------------*/
  WIFI_Init();
  ControlMsg.ESP32Info.Aisle = '0';
  /*-------------------------- MOTOR controller Init ----------------------------*/
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_ALL);
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->CNT = 0;
  TIM2->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E);

  /*--------------------------- MOTOR1 Encoder Init -----------------------------*/
  PID_Init(&MOTOR_PID[MOTOR1], 0.85, 0.90, 0.02);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /*--------------------------- MOTOR2 Encoder Init -----------------------------*/
  PID_Init(&MOTOR_PID[MOTOR2], 0.85, 0.90, 0.02);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*------------- read WIFI data -------------*/
    MessageProcess();
    /*-------------- MOTOR Control -------------*/
    MOTOR_Control();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 48000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void AT_SEND(UART_HandleTypeDef *huart, uint8_t Aisle, int SendSize)
{
  AT_CIPSEND(huart, ControlMsg.ESP32Info.Aisle, SendSize);
  HAL_Delay(20);
  HAL_UART_Transmit(huart, (uint8_t *)ControlMsg.TXBuffer, SendSize, 10);
}

void SendINFO(UART_HandleTypeDef *huart, const char *format, ...)
{
  va_list args;
  int SendSize;

  va_start(args, format);
  SendSize = vsprintf(ControlMsg.TXBuffer, format, args);
  va_end(args);

  AT_SEND(huart, ControlMsg.ESP32Info.Aisle, SendSize);
}

uint32_t Gettime(uint32_t StartTime)
{
  uint32_t CurrentTime = HAL_GetTick();

  if(CurrentTime >= StartTime) return (CurrentTime - StartTime);
  else                         return (0xFFFFFFFF - StartTime + CurrentTime + 1);
}

static void WIFI_Init(void)
{
  USART1->CR1 |= USART_CR1_RXNEIE;

  HAL_Delay(5000);
  HAL_UART_Transmit(&huart1, (uint8_t *)AT,        sizeof(AT)        -1, 10); HAL_Delay(800);
  HAL_UART_Transmit(&huart1, (uint8_t *)ATE,       sizeof(ATE)       -1, 10); HAL_Delay(800);
  HAL_UART_Transmit(&huart1, (uint8_t *)CWMODE,    sizeof(CWMODE)    -1, 10); HAL_Delay(800);
  HAL_UART_Transmit(&huart1, (uint8_t *)CIPMUX,    sizeof(CIPMUX)    -1, 10); HAL_Delay(800);
  HAL_UART_Transmit(&huart1, (uint8_t *)CIPSERVER, sizeof(CIPSERVER) -1, 10); HAL_Delay(800);
}


void CalculateSpeedAngle(Encoder_t      Encoder[2],
                         PID_t          PID[2],
                         ControlINFO_t *ControlINFO,
                         uint8_t        ControlParameters[],
                         const float    MAXRPM,
                         const double   Coefficient[2][6])
{
#define use 1

  if(ControlParameters[3] == 0x11 && ControlParameters[8] == 0x12)
  {
    uint8_t SpeedData[4] = { ControlParameters[7], ControlParameters[6], ControlParameters[5], ControlParameters[4] };
    memcpy(&ControlINFO->MotorSpeed, (void *)&SpeedData, 4);
    uint8_t AngleData[4] = { ControlParameters[12], ControlParameters[11], ControlParameters[10], ControlParameters[9] };
    memcpy(&ControlINFO->SteeringAngle, (void *)&AngleData, 4);

    ControlINFO->Action = Forward;
    if(ControlINFO->SteeringAngle >  90.0 && ControlINFO->SteeringAngle <=  180.0)
    {
      ControlINFO->SteeringAngle =  180.0 - ControlINFO->SteeringAngle;
      ControlINFO->Action = Backward;
    }
    else if(ControlINFO->SteeringAngle < -90.0 && ControlINFO->SteeringAngle >= -180.0)
    {
      ControlINFO->SteeringAngle = -180.0 - ControlINFO->SteeringAngle;
      ControlINFO->Action = Backward;
    }

    if(  (ControlINFO->SteeringAngle >= -20.0 && ControlINFO->SteeringAngle <=  20.0)
       ||(ControlINFO->SteeringAngle >=  70.0 && ControlINFO->SteeringAngle <=  90.0)
       ||(ControlINFO->SteeringAngle <= -70.0 && ControlINFO->SteeringAngle >= -90.0))
    {
      Encoder[MOTOR1].TargetRPM = MAXRPM;
      Encoder[MOTOR2].TargetRPM = MAXRPM;
    }
    else if(ControlINFO->SteeringAngle > 20.0 && ControlINFO->SteeringAngle < 70.0)
    {
#if use
      Encoder[MOTOR1].TargetRPM = MAXRPM;
      Encoder[MOTOR2].TargetRPM = MAXRPM * ((90 - ControlINFO->SteeringAngle) / 90.0);
#else
      Encoder[MOTOR1].TargetRPM = (MAXRPM / 2.0) * (1 + ControlINFO->SteeringAngle / 90.0);
      Encoder[MOTOR2].TargetRPM = (MAXRPM / 2.0) * (1 - ControlINFO->SteeringAngle / 90.0);
      Encoder[MOTOR1].TargetRPM = 320000.0 / Encoder[MOTOR1].TargetRPM; // 400 * 800
      Encoder[MOTOR2].TargetRPM = 2.000000 * Encoder[MOTOR2].TargetRPM;
#endif
    }
    else if(ControlINFO->SteeringAngle < -20.0 && ControlINFO->SteeringAngle > -70.0)
    {
#if use
      Encoder[MOTOR1].TargetRPM = MAXRPM * ((90 - fabsf(ControlINFO->SteeringAngle)) / 90.0);
      Encoder[MOTOR2].TargetRPM = MAXRPM;
#else
      Encoder[MOTOR1].TargetRPM = (MAXRPM / 2.0) * (1 - fabsf(ControlINFO->SteeringAngle) / 90.0);
      Encoder[MOTOR2].TargetRPM = (MAXRPM / 2.0) * (1 + fabsf(ControlINFO->SteeringAngle) / 90.0);
      Encoder[MOTOR1].TargetRPM = 2.000000 * Encoder[MOTOR1].TargetRPM; // 400 * 800
      Encoder[MOTOR2].TargetRPM = 320000.0 / Encoder[MOTOR2].TargetRPM;
#endif
    }

    if     (Encoder[MOTOR1].TargetRPM > MAXRPM) Encoder[MOTOR1].TargetRPM = MAXRPM;
    else if(Encoder[MOTOR1].TargetRPM <      0) Encoder[MOTOR1].TargetRPM = 0;
    if     (Encoder[MOTOR2].TargetRPM > MAXRPM) Encoder[MOTOR2].TargetRPM = MAXRPM;
    else if(Encoder[MOTOR2].TargetRPM <      0) Encoder[MOTOR2].TargetRPM = 0;

    Encoder[MOTOR1].TargetRPM = Encoder[MOTOR1].TargetRPM * (ControlINFO->MotorSpeed / 100.0);
    Encoder[MOTOR2].TargetRPM = Encoder[MOTOR2].TargetRPM * (ControlINFO->MotorSpeed / 100.0);

//    Encoder[MOTOR1].TargetPWM = CalculateTargetPWM(Coefficient, ControlINFO->MotorSpeed);
  }
}

static void MessageProcess(void)
{
  if(Count.UART >= 100)
  {
    Count.UART = 0;
    ControlMsg.MessageSize = QueueFindControlMessage(&ControlMsg, BUFFER_SIZE);

    if(ControlMsg.MessageSize > 0 && ControlMsg.TmpData[0] == 0xF1)
    {
      switch(ControlMsg.TmpData[2])
      {
        case 0x10:
          CalculateSpeedAngle(Encoder, MOTOR_PID, &ControlINFO, ControlMsg.TmpData, 400.0f, Coefficient);
//          Encoder[MOTOR1].TargetPWM = CalculateTargetPWM(&Coefficient[MOTOR1][0], speed);
          break;

//        case 0x20: // step control
//          if(ControlMsg.TmpData[3] == 0xFF) Control_flag.Step = true;
//          break;

        case 0x30: // Calibration Coefficient point
          PWM_LimitCal.CalState[MOTOR1] = STARTCAL;
          SendINFO(&huart1, "Start Calibration Coefficient point\n");
          break;

        case 0x40: // test
          SendINFO(&huart1, "test ok\n");
          break;

        case 0x41: SendAuthor_Lkl(&huart1, ControlMsg.TmpData); break;
        case 0x63: SendCatPattern(&huart1, ControlMsg.TmpData); break;
      }
    }
  }
}

static void MOTOR_Control(void)
{
  if(Count.PID_Ctrl >= 25 && ((ControlMsg.TmpData[2] == 0x10 && ControlINFO.MotorSpeed != 0.0) || PWM_LimitCal.CalState[MOTOR1] != NONECTRL))
  {
    Count.PID_Ctrl = 0;
    CalculateSpeed(&htim3, &Encoder[MOTOR1]);
    CalculateSpeed(&htim4, &Encoder[MOTOR2]);

    switch(ControlINFO.Action)
    {
//      case Forward:
//        TIM2->CCR1 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR2 = 0;
//        TIM2->CCR3 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR4 = 0;
//        break;
//
//      case Backward:
//        TIM2->CCR2 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR1 = 0;
//        TIM2->CCR4 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR3 = 0;
//        break;
//
//      case Clockwise:
//        TIM2->CCR1 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR2 = 0;
//        TIM2->CCR4 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR3 = 0;
//        break;
//
//      case Anticlockwise:
//        TIM2->CCR2 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR1 = 0;
//        TIM2->CCR3 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR4 = 0;
//        break;

      default: break;
    }
//    if(ControlINFO.SteeringAngle >= 70.0 && ControlINFO.SteeringAngle <= 90.0)
//    {
//      TIM2->CCR1 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR2 = 0;
//      TIM2->CCR4 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR3 = 0;
//    }else if(ControlINFO.SteeringAngle <= -70.0 && ControlINFO.SteeringAngle >= -90.0){
//      TIM2->CCR2 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR1 = 0;
//      TIM2->CCR3 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR4 = 0;
//    }else if(ControlINFO.Direction){
//      TIM2->CCR1 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR2 = 0;
//      TIM2->CCR3 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR4 = 0;
//    }else{
//      TIM2->CCR2 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR1], Encoder[MOTOR1]); TIM2->CCR1 = 0;
//      TIM2->CCR4 = (uint32_t)PID_Compute(&MOTOR_PID[MOTOR2], Encoder[MOTOR2]); TIM2->CCR3 = 0;
//    }
  }
  else if(ControlINFO.MotorSpeed == 0.0)
  {
    PID_ParameterClear(&MOTOR_PID[MOTOR1]); TIM2->CCR1 = TIM2->CCR2 = 0;
    PID_ParameterClear(&MOTOR_PID[MOTOR2]); TIM2->CCR3 = TIM2->CCR4 = 0;
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
