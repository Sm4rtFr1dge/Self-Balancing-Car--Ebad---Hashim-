/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Self-Balancing Robot - Main Integration
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Sampling Time ---- */
#define DT 0.005f            // 200Hz -> 1/200 = 0.005s

/* ---- PID Gains (SET TO ZERO FOR NOW TO FIND SETPOINT) ---- */
#define KP  40.0f
#define KI  0.0f
#define KD  0.0f

/* ---- Setpoint ---- */
// Target angle. You will change this number after reading the serial monitor!
#define SETPOINT 0.0f

/* ---- Output Limits ---- */
#define PID_OUT_MAX  999.0f   // TIM3 period = 999, max PWM
#define PID_OUT_MIN -999.0f

/* ---- Integral Anti-Windup Limit ---- */
#define INTEGRAL_MAX  200.0f
#define INTEGRAL_MIN -200.0f

/* ---- Complementary Filter Weights ---- */
#define COMP_GYRO_WEIGHT  0.98f
#define COMP_ACC_WEIGHT   0.02f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* ---- Angle Estimation Variables ---- */
volatile float shared_angle   = 0.0f;   // Bridge between ISR and main
volatile float shared_pid_out = 0.0f;   // PID output for display
volatile uint8_t display_flag = 0;

// For display/debug in main loop
float gyro_rate = 0.0f;
float acc_angle = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 * I3G4250D (Gyroscope) SPI Read/Write
 * ============================================================ */
void I3G_WriteReg(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr & 0x7F;  // Bit 7 = 0 for WRITE
    tx_buffer[1] = data;

    // Use CS pin (PE3)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS HIGH
}

uint8_t I3G_ReadReg(uint8_t reg_addr) {
    uint8_t tx_data = reg_addr | 0x80;  // Bit 7 = 1 for READ
    uint8_t rx_data;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    return rx_data;
}

/* ============================================================
 * Motor Control Functions - CORRECTED PINOUT
 * Shield Pin → STM32 Pin Mapping:
 * D6 (Left IN2)  → PD3
 * D7 (Left IN1)  → PD1  
 * D8 (Right IN4) → PD5
 * D9 (Left PWM)  → PB4 (TIM3_CH1)
 * D10 (Right PWM)→ PA4 (TIM3_CH2)
 * D12 (Right IN3)→ PD4
 * ============================================================ */

void Motor_Left_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);    // D7 → PD1 = HIGH (IN1)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);  // D6 → PD3 = LOW (IN2)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);   // D9 → PB4 (TIM3_CH1)
}

void Motor_Left_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);  // D7 → PD1 = LOW (IN1)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);    // D6 → PD3 = HIGH (IN2)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);   // D9 → PB4 (TIM3_CH1)
} 

void Motor_Left_Stop(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

void Motor_Right_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);    // D12 → PD4 = HIGH (IN3)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  // D8 → PD5 = LOW (IN4)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);   // D10 → PA4 (TIM3_CH2)
}

void Motor_Right_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  // D12 → PD4 = LOW (IN3)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);    // D8 → PD5 = HIGH (IN4)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);   // D10 → PA4 (TIM3_CH2)
}

void Motor_Right_Stop(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

/* ============================================================
 * Drive logic based on PID
 * ============================================================ */
void Motors_Drive(float pid_output) {
    uint16_t pwm_val;

    if (pid_output > 0) {
        if (pid_output > PID_OUT_MAX) pid_output = PID_OUT_MAX;
        pwm_val = (uint16_t)pid_output;
        Motor_Left_Forward(pwm_val);
        Motor_Right_Forward(pwm_val);
    }
    else if (pid_output < 0) {
        float abs_out = -pid_output;
        if (abs_out > PID_OUT_MAX) abs_out = PID_OUT_MAX;
        pwm_val = (uint16_t)abs_out;
        Motor_Left_Backward(pwm_val);
        Motor_Right_Backward(pwm_val);
    }
    else {
        Motor_Left_Stop();
        Motor_Right_Stop();
    }
}

/* UART Print Helper */
void cout(const char *fmt, ...) {
    char buffer[128]; 
    va_list args;
    va_start(args, fmt);
    int l = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (l > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, l, HAL_MAX_DELAY);
    }
}

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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USB_PCD_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
/* USER CODE BEGIN 2 */

// ---- Initialize Gyroscope (I3G4250D via SPI) ----
I3G_WriteReg(0x20, 0x4F);

// ---- Initialize Accelerometer (LSM303DLHC via I2C) ----
uint8_t acc_init[2] = {0x20, 0x57};
HAL_I2C_Master_Transmit(&hi2c1, 0x32, acc_init, 2, 50);

// ---- Start PWM on both channels for motor speed control ----
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // D9 → PB4
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // D10 → PA4

// Initialize motors stopped
Motor_Left_Stop();
Motor_Right_Stop();

// Motor Test
cout("\r\n=== MOTOR TEST START ===\r\n");
HAL_Delay(1000);

cout("Testing LEFT motor forward (D7/D6 via D9)...\r\n");
Motor_Left_Forward(500);
HAL_Delay(2000);
Motor_Left_Stop();
HAL_Delay(500);

cout("Testing RIGHT motor forward (D12/D8 via D10)...\r\n");
Motor_Right_Forward(500);
HAL_Delay(2000);
Motor_Right_Stop();
HAL_Delay(500);

cout("Testing BOTH motors forward...\r\n");
Motor_Left_Forward(500);
Motor_Right_Forward(500);
HAL_Delay(2000);
Motor_Left_Stop();
Motor_Right_Stop();

cout("=== MOTOR TEST COMPLETE ===\r\n");
cout("Starting PID loop in 2 seconds...\r\n");
HAL_Delay(2000);

// ---- Start TIM2 interrupt for 200Hz control loop ----
HAL_TIM_Base_Start_IT(&htim2);

// Small delay to let sensors settle
HAL_Delay(100);

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (display_flag == 1) {
          display_flag = 0;

          // Float printing bypass for embedded C
          int shared_angle_i = (int)shared_angle;
          int shared_angle_f = (int)(fabsf(shared_angle - shared_angle_i) * 100);

          int acc_angle_i = (int)acc_angle;
          int acc_angle_f = (int)(fabsf(acc_angle - acc_angle_i) * 100);

          int gyro_rate_i = (int)gyro_rate;
          int gyro_rate_f = (int)(fabsf(gyro_rate - gyro_rate_i) * 100);

          int shared_pid_out_i = (int)shared_pid_out;
          int shared_pid_out_f = (int)(fabsf(shared_pid_out - shared_pid_out_i) * 100);

          // Print to Serial Monitor
          cout("Angle: %d.%02d  |  Acc: %d.%02d  |  Gyro: %d.%02d  |  PID: %d.%02d\r\n", 
                shared_angle_i, shared_angle_f, 
                acc_angle_i, acc_angle_f, 
                gyro_rate_i, gyro_rate_f, 
                shared_pid_out_i, shared_pid_out_f);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; 
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH; 
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level for Port E */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LD3_Pin|LD5_Pin|LD7_Pin|GPIO_PIN_12
                          |LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins for Port E */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LD3_Pin|LD5_Pin|LD7_Pin|GPIO_PIN_12
                          |LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ============================================================
 * TIM2 ISR Callback — runs at 200 Hz (every 5ms)
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM2) {

        static float tilt_angle    = 0.0f;
        static float integral      = 0.0f;
        static float prev_error    = 0.0f;
        static int   uart_counter  = 0;

        // --- CALIBRATION VARIABLES ---
        static int   calib_counter    = 0;
        static float gyro_y_sum       = 0.0f;
        static float acc_angle_sum    = 0.0f;
        static float gyro_y_offset    = 0.0f;
        static float acc_angle_offset = 0.0f;

        /* --- STEP 1: GYRO --- */
        uint8_t y_low  = I3G_ReadReg(0x2A);  // OUT_Y_L
        uint8_t y_high = I3G_ReadReg(0x2B);  // OUT_Y_H
        int16_t raw_gyro_y = (int16_t)((y_high << 8) | y_low);
        float gyro_y_dps = (float)raw_gyro_y * 0.00875f;

        /* --- STEP 2: ACCEL --- */
        uint8_t acc_reg = 0x28 | 0x80;
        uint8_t acc_buf[6] = {0};
        HAL_I2C_Master_Transmit(&hi2c1, 0x32, &acc_reg, 1, 10);
        HAL_I2C_Master_Receive(&hi2c1, 0x33, acc_buf, 6, 10);
        int16_t acc_raw_x = (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
        int16_t acc_raw_z = (int16_t)((acc_buf[5] << 8) | acc_buf[4]);
        float acc_angle_deg = atan2f((float)acc_raw_x, (float)acc_raw_z) * (180.0f / 3.14159f);

        /* ====================================================
         * CALIBRATION ROUTINE (First 250 Samples = 1.25s)
         * ==================================================== */
        if (calib_counter < 250) {
            // Accumulate readings
            gyro_y_sum    += gyro_y_dps;
            acc_angle_sum += acc_angle_deg;
            calib_counter++;

            // Ensure motors remain stopped during calibration
            Motor_Left_Stop();
            Motor_Right_Stop();
            
            return; // Skip the rest of the ISR (Comp Filter, PID, Drive)
        }
        else if (calib_counter == 250) {
            // Calculate average offsets
            gyro_y_offset    = gyro_y_sum / 250.0f;
            acc_angle_offset = acc_angle_sum / 250.0f;
            
            // Set starting tilt to exactly 0 to prevent initial jumps
            tilt_angle = 0.0f; 
            
            calib_counter++; // Move to state 251 (Normal Operation)
            return;
        }

        /* ====================================================
         * NORMAL OPERATION
         * ==================================================== */
        // Apply calculated offsets to zero-out the sensors
        gyro_y_dps    -= gyro_y_offset;
        acc_angle_deg -= acc_angle_offset;

        /* --- STEP 3: COMP FILTER --- */
        tilt_angle = COMP_GYRO_WEIGHT * (tilt_angle + gyro_y_dps * DT)
                   + COMP_ACC_WEIGHT  * acc_angle_deg;

        /* --- STEP 4: PID --- */
        float error = SETPOINT - tilt_angle;

        float p_term = KP * error;

        integral += KI * error * DT;
        if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        if (integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

        float derivative = (error - prev_error) / DT;
        float d_term = KD * derivative;
        prev_error = error;

        float pid_output = p_term + integral + d_term;

        if (pid_output > PID_OUT_MAX) pid_output = PID_OUT_MAX;
        if (pid_output < PID_OUT_MIN) pid_output = PID_OUT_MIN;

        /* --- STEP 5: DRIVE --- */
        Motors_Drive(pid_output);

        /* --- STEP 6: SHARED VARS --- */
        shared_angle   = tilt_angle;
        shared_pid_out = pid_output;
        gyro_rate      = gyro_y_dps;
        acc_angle      = acc_angle_deg;

        /* --- STEP 7: UART THROTTLE (10Hz) --- */
        if (++uart_counter >= 20) {
            display_flag = 1;
            uart_counter = 0;
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
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */