/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Self-Balancing Robot with AUTO-TUNING PID
  ******************************************************************************
  * TIM2 = 200Hz control loop ISR (complementary filter + PID)
  * TIM3 = PWM generation for motors (CH1=left on PC6, CH2=right on PA4)
  * Motor direction pins:
  *   Left  motor: PB12 (IN1), PB13 (IN2)
  *   Right motor: PB14 (IN1), PB15 (IN2)
  *
  * AUTO-TUNING ALGORITHM:
  * Every 3 seconds, the code analyzes:
  *   - Average absolute error (how off-balance was the robot)
  *   - Number of zero-crossings (oscillation count)
  *   - Max angle deviation
  * Then adjusts Kp, Ki, Kd to improve balance over time.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Sampling Time ---- */
#define DT 0.005f            // 200Hz -> 1/200 = 0.005s

/* ---- INITIAL PID Gains (starting point for auto-tune) ---- */
#define KP_INITIAL  30.0f
#define KI_INITIAL  0.2f
#define KD_INITIAL  1.5f

/* ---- Setpoint ---- */
#define SETPOINT -3.5f

/* ---- Output Limits ---- */
#define PID_OUT_MAX  999.0f
#define PID_OUT_MIN -999.0f

/* ---- Integral Anti-Windup Limit ---- */
#define INTEGRAL_MAX  500.0f
#define INTEGRAL_MIN -500.0f

/* ---- Complementary Filter Weights ---- */
#define COMP_GYRO_WEIGHT  0.98f
#define COMP_ACC_WEIGHT   0.02f

/* ---- AUTO-TUNING PARAMETERS ---- */
#define TUNE_INTERVAL_TICKS    600     // 600 ticks * 5ms = 3 seconds per tuning cycle
#define TUNE_OSC_THRESHOLD     8       // Oscillations above this = too much Kp or too little Kd
#define TUNE_ERROR_THRESHOLD   3.0f    // Avg error above this = need more Kp
#define TUNE_DRIFT_THRESHOLD   1.5f    // Slow drift = need more Ki

/* ---- Gain adjustment step sizes ---- */
#define KP_STEP  1.0f
#define KI_STEP  0.05f
#define KD_STEP  0.1f

/* ---- Gain Limits (safety bounds) ---- */
#define KP_MIN  10.0f
#define KP_MAX  60.0f
#define KI_MIN  0.0f
#define KI_MAX  3.0f
#define KD_MIN  0.3f
#define KD_MAX  4.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* ---- PID Gains (now variables, not #defines) ---- */
volatile float Kp = KP_INITIAL;
volatile float Ki = KI_INITIAL;
volatile float Kd = KD_INITIAL;

/* ---- Auto-Tuning Metrics ---- */
volatile float tune_error_sum   = 0.0f;   // Sum of |error| over tuning window
volatile float tune_max_error   = 0.0f;   // Max |error| seen in window
volatile float tune_avg_error   = 0.0f;   // Running average of error (for drift detection)
volatile uint16_t tune_osc_count = 0;     // Zero-crossing count (oscillations)
volatile uint16_t tune_tick_count = 0;    // Counts ticks within window
volatile uint8_t tune_ready_flag = 0;     // Set by ISR when window complete

/* ---- Angle Estimation Variables ---- */
volatile float shared_angle   = 0.0f;
volatile float shared_pid_out = 0.0f;
volatile uint8_t display_flag = 0;

float gyro_rate = 0.0f;
float acc_angle = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void AutoTune_Adjust(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 *  I3G4250D (Gyroscope) SPI Read/Write
 * ============================================================ */
void I3G_WriteReg(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr & 0x7F;
    tx_buffer[1] = data;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

uint8_t I3G_ReadReg(uint8_t reg_addr) {
    uint8_t tx_data = reg_addr | 0x80;
    uint8_t rx_data;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return rx_data;
}

/* ============================================================
 *  Motor Control Functions
 * ============================================================ */

void Motor_Left_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

void Motor_Left_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

void Motor_Left_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

void Motor_Right_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

void Motor_Right_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

void Motor_Right_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

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

/* ============================================================
 *  AUTO-TUNING ALGORITHM
 *  
 *  Called from main loop when ISR signals window complete.
 *  Analyzes performance metrics and adjusts gains accordingly.
 *  
 *  Logic:
 *  1. Too much oscillation (high osc_count) → reduce Kp OR increase Kd
 *  2. Big errors but no oscillation → increase Kp (sluggish)
 *  3. Persistent drift (avg_error far from zero) → increase Ki
 *  4. Already balanced (small errors, low osc) → fine-tune slowly
 * ============================================================ */
void AutoTune_Adjust(void) {
    
    // Calculate average absolute error
    float avg_abs_error = tune_error_sum / (float)TUNE_INTERVAL_TICKS;
    
    char status_msg[150];
    int len;
    
    // Decision tree for tuning
    if (tune_osc_count > TUNE_OSC_THRESHOLD) {
        // PROBLEM: Too much oscillation
        // SOLUTION: Reduce Kp slightly + increase Kd to dampen
        if (Kp > KP_MIN) Kp -= KP_STEP * 0.5f;
        if (Kd < KD_MAX) Kd += KD_STEP;
        len = sprintf(status_msg, 
            ">> OSCILLATING (osc=%u, err=%.2f) | Kp-, Kd+ | Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
            tune_osc_count, avg_abs_error, Kp, Ki, Kd);
    }
    else if (tune_max_error > TUNE_ERROR_THRESHOLD * 2.0f) {
        // PROBLEM: Big swings, but not oscillating fast (sluggish recovery)
        // SOLUTION: Increase Kp for stronger response
        if (Kp < KP_MAX) Kp += KP_STEP;
        len = sprintf(status_msg,
            ">> SLUGGISH (max=%.2f, osc=%u) | Kp+ | Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
            tune_max_error, tune_osc_count, Kp, Ki, Kd);
    }
    else if (fabsf(tune_avg_error) > TUNE_DRIFT_THRESHOLD) {
        // PROBLEM: Steady drift in one direction (avg error not zero)
        // SOLUTION: Increase Ki to fight steady-state error
        if (Ki < KI_MAX) Ki += KI_STEP;
        len = sprintf(status_msg,
            ">> DRIFTING (avg=%.2f) | Ki+ | Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
            tune_avg_error, Kp, Ki, Kd);
    }
    else if (avg_abs_error < 0.5f && tune_osc_count < 3) {
        // GOOD: Robot is balancing well!
        // Try slightly reducing Ki if it's high (avoid windup over time)
        if (Ki > KI_MIN + KI_STEP) Ki -= KI_STEP * 0.5f;
        len = sprintf(status_msg,
            ">> BALANCED! (err=%.2f, osc=%u) | locked | Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
            avg_abs_error, tune_osc_count, Kp, Ki, Kd);
    }
    else {
        // Mild instability - small Kd boost to smooth things out
        if (Kd < KD_MAX) Kd += KD_STEP * 0.5f;
        len = sprintf(status_msg,
            ">> MILD UNREST (err=%.2f, osc=%u) | Kd+ | Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
            avg_abs_error, tune_osc_count, Kp, Ki, Kd);
    }
    
    HAL_UART_Transmit(&huart1, (uint8_t*)status_msg, len, HAL_MAX_DELAY);
    
    // Reset metrics for next window
    tune_error_sum  = 0.0f;
    tune_max_error  = 0.0f;
    tune_avg_error  = 0.0f;
    tune_osc_count  = 0;
    tune_tick_count = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  // Initialize Gyroscope
  I3G_WriteReg(0x20, 0x4F);

  // Initialize Accelerometer
  uint8_t acc_init[2] = {0x20, 0x57};
  HAL_I2C_Master_Transmit(&hi2c1, 0x32, acc_init, 2, 50);

  // Start PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  // Print startup message
  char *msg = "\r\n=== AUTO-TUNING PID ACTIVE ===\r\n"
              "Initial: Kp=30 Ki=0.2 Kd=1.5\r\n"
              "Adjusting every 3 seconds...\r\n\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // Start TIM2 interrupt for 200Hz control loop
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_Delay(100);

  /* USER CODE END 2 */

  while (1)
  {
      // Check if auto-tune window is complete
      if (tune_ready_flag == 1) {
          tune_ready_flag = 0;
          AutoTune_Adjust();
      }
      
      // Display angle data
      if (display_flag == 1) {
          display_flag = 0;

          char tx_buffer[100];
          int len = sprintf(tx_buffer, "%.2f,%.2f,%.2f,%.2f\r\n",
                            shared_angle, acc_angle, gyro_rate, shared_pid_out);
          HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
      }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ============================================================
 *  TIM2 ISR Callback — 200 Hz control loop with metric collection
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM2) {

        static float tilt_angle    = 0.0f;
        static float integral      = 0.0f;
        static float prev_error    = 0.0f;
        static int   uart_counter  = 0;

        /* Read Gyro */
        uint8_t y_low  = I3G_ReadReg(0x2A);
        uint8_t y_high = I3G_ReadReg(0x2B);
        int16_t raw_gyro_y = (int16_t)((y_high << 8) | y_low);
        float gyro_y_dps = (float)raw_gyro_y * 0.00875f;

        /* Read Accel */
        uint8_t acc_reg = 0x28 | 0x80;
        uint8_t acc_buf[6] = {0};
        HAL_I2C_Master_Transmit(&hi2c1, 0x32, &acc_reg, 1, 10);
        HAL_I2C_Master_Receive(&hi2c1, 0x33, acc_buf, 6, 10);
        int16_t acc_raw_x = (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
        int16_t acc_raw_z = (int16_t)((acc_buf[5] << 8) | acc_buf[4]);
        float acc_angle_deg = atan2f((float)acc_raw_x, (float)acc_raw_z) * (180.0f / 3.14159f);

        /* Complementary Filter */
        tilt_angle = COMP_GYRO_WEIGHT * (tilt_angle + gyro_y_dps * DT)
                   + COMP_ACC_WEIGHT  * acc_angle_deg;

        /* PID Controller - using runtime gain VARIABLES */
        float error = tilt_angle - SETPOINT;

        // P term
        float p_term = Kp * error;

        // I term
        integral += Ki * error * DT;
        if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        if (integral < INTEGRAL_MIN) integral = INTEGRAL_MIN;

        // D term
        float derivative = (error - prev_error) / DT;
        float d_term = Kd * derivative;

        // Total output
        float pid_output = p_term + integral + d_term;
        if (pid_output > PID_OUT_MAX) pid_output = PID_OUT_MAX;
        if (pid_output < PID_OUT_MIN) pid_output = PID_OUT_MIN;

        /* ========================================================
         *  AUTO-TUNING METRIC COLLECTION
         * ======================================================== */
        float abs_error = fabsf(error);
        
        // Sum absolute errors
        tune_error_sum += abs_error;
        
        // Track max error in window
        if (abs_error > tune_max_error) tune_max_error = abs_error;
        
        // Running sum of signed error (for drift detection)
        tune_avg_error += error / (float)TUNE_INTERVAL_TICKS;
        
        // Count zero-crossings (oscillations)
        // If error sign flipped from previous tick, count it
        if ((error > 0.0f && prev_error < 0.0f) || (error < 0.0f && prev_error > 0.0f)) {
            tune_osc_count++;
        }
        
        prev_error = error;
        
        // Increment tick counter, set flag when window complete
        tune_tick_count++;
        if (tune_tick_count >= TUNE_INTERVAL_TICKS) {
            tune_ready_flag = 1;  // Signal main loop to run AutoTune_Adjust
            // Note: counters reset inside AutoTune_Adjust()
        }

        /* Drive Motors */
        Motors_Drive(pid_output);

        /* Update shared variables */
        shared_angle   = tilt_angle;
        shared_pid_out = pid_output;
        gyro_rate      = gyro_y_dps;
        acc_angle      = acc_angle_deg;

        /* UART Display throttle */
        if (++uart_counter >= 20) {
            display_flag = 1;
            uart_counter = 0;
        }
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif