/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Self-Balancing Robot - top-level orchestrator
  *
  *          All control logic (IMU, filter, PID, motors, encoders) lives in
  *          control.c / control.h. This file handles peripheral init,
  *          interrupt dispatch, and UART logging only.
  *
  *          TIM2 = 200 Hz control loop ISR
  *          TIM3 = PWM for motors (CH1=PC6 left, CH2=PA4 right)
  *          PD10 / PD11 = encoder EXTI inputs
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t display_flag = 0;
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* Peripheral handles (set in Control_Init) */
static I2C_HandleTypeDef *ctrl_hi2c = NULL;
static SPI_HandleTypeDef *ctrl_hspi = NULL;
static TIM_HandleTypeDef *ctrl_htim = NULL;

/* Filter and PID state (persist across ISR calls) */
static float tilt_angle = 0.0f;
static float integral   = 0.0f;
static float prev_error = 0.0f;

/* Encoder counters (volatile - modified in EXTI ISR) */
static volatile int32_t encoder_left  = 0;
static volatile int32_t encoder_right = 0;
static volatile int8_t  motor_dir     = 0;  // +1 fwd, -1 bwd, 0 stop

/* Latest telemetry snapshot for main loop */
static volatile Control_Telemetry_t telemetry = {0};

/* ============================================================
 *  IMU Drivers
 * ============================================================ */

/* Gyro register addresses */
#define GYRO_REG_CTRL1    0x20
#define GYRO_REG_OUT_Y_L  0x2A
#define GYRO_REG_OUT_Y_H  0x2B

/* Accel register addresses (LSM303DLHC at I2C addr 0x32 write) */
#define ACC_I2C_ADDR      0x32
#define ACC_REG_CTRL1     0x20
#define ACC_REG_OUT_X_L   0x28
#define ACC_AUTO_INC      0x80

static void Gyro_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg & 0x7F, data };  // bit 7 = 0 -> write
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  // CS low
    HAL_SPI_Transmit(ctrl_hspi, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);    // CS high
}

static uint8_t Gyro_ReadReg(uint8_t reg) {
    uint8_t tx = reg | 0x80;  // bit 7 = 1 -> read
    uint8_t rx;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(ctrl_hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive (ctrl_hspi, &rx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return rx;
}

/* ============================================================
 *  Motor Drivers (L298N-style H-bridge)
 *
 *  Left  motor: PWM on TIM3_CH1 (PC6), IN1=PB12, IN2=PB13
 *  Right motor: PWM on TIM3_CH2 (PA4), IN1=PB14, IN2=PB15
 * ============================================================ */

static void Motor_Left_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_1, speed);
}

static void Motor_Left_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_1, speed);
}

static void Motor_Left_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_1, 0);
}

static void Motor_Right_Forward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_2, speed);
}

static void Motor_Right_Backward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_2, speed);
}

static void Motor_Right_Stop(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_2, 0);
}

/* Drive both motors. Updates motor_dir for encoder pulse signing. */
static void Motors_Drive(float pid_output) {
    uint16_t pwm;

    if (pid_output > 0.0f) {
        if (pid_output > PID_OUT_MAX) pid_output = PID_OUT_MAX;
        pwm = (uint16_t)pid_output;
        Motor_Left_Forward(pwm);
        Motor_Right_Forward(pwm);
        motor_dir = +1;
    }
    else if (pid_output < 0.0f) {
        float abs_out = -pid_output;
        if (abs_out > PID_OUT_MAX) abs_out = PID_OUT_MAX;
        pwm = (uint16_t)abs_out;
        Motor_Left_Backward(pwm);
        Motor_Right_Backward(pwm);
        motor_dir = -1;
    }
    else {
        Motor_Left_Stop();
        Motor_Right_Stop();
        motor_dir = 0;
    }
}

/* ============================================================
 *  Public API
 * ============================================================ */

void Control_Init(I2C_HandleTypeDef *hi2c,
                  SPI_HandleTypeDef *hspi,
                  TIM_HandleTypeDef *htim)
{
    ctrl_hi2c = hi2c;
    ctrl_hspi = hspi;
    ctrl_htim = htim;

    /* --- Configure gyro: 200 Hz ODR, normal mode, X/Y/Z enabled --- */
    Gyro_WriteReg(GYRO_REG_CTRL1, 0x4F);

    /* --- Configure accel: 100 Hz ODR, normal mode, X/Y/Z enabled --- */
    uint8_t acc_init[2] = { ACC_REG_CTRL1, 0x57 };
    HAL_I2C_Master_Transmit(ctrl_hi2c, ACC_I2C_ADDR, acc_init, 2, 50);

    /* --- Start motor PWM channels at 0 --- */
    HAL_TIM_PWM_Start(ctrl_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(ctrl_htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(ctrl_htim, TIM_CHANNEL_2, 0);

    /* --- Reset internal state --- */
    tilt_angle    = 0.0f;
    integral      = 0.0f;
    prev_error    = 0.0f;
    encoder_left  = 0;
    encoder_right = 0;
    motor_dir     = 0;
}

void Control_RunStep(void) {

    /* --- 1. Read gyroscope (Y-axis = pitch rate) --- */
    uint8_t y_low  = Gyro_ReadReg(GYRO_REG_OUT_Y_L);
    uint8_t y_high = Gyro_ReadReg(GYRO_REG_OUT_Y_H);
    int16_t raw_y  = (int16_t)((y_high << 8) | y_low);
    float gyro_y_dps = (float)raw_y * 0.00875f;   // 245 dps full scale

    /* --- 2. Read accelerometer (X and Z for pitch) --- */
    uint8_t reg = ACC_REG_OUT_X_L | ACC_AUTO_INC;
    uint8_t buf[6] = {0};
    HAL_I2C_Master_Transmit(ctrl_hi2c, ACC_I2C_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive (ctrl_hi2c, ACC_I2C_ADDR | 0x01, buf, 6, 10);
    int16_t acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t acc_z = (int16_t)((buf[5] << 8) | buf[4]);
    float acc_angle_deg = atan2f((float)acc_x, (float)acc_z) * (180.0f / 3.14159f);

    /* --- 3. Complementary filter: fuse gyro + accel --- */
    tilt_angle = COMP_GYRO_WEIGHT * (tilt_angle + gyro_y_dps * DT)
               + COMP_ACC_WEIGHT  * acc_angle_deg;

    /* --- 4. Encoder position feedback --- */
    float position = (float)(encoder_left + encoder_right) * 0.5f;

    float position_bias = POSITION_KP * position;
    if (position_bias > POSITION_BIAS_MAX) position_bias = POSITION_BIAS_MAX;
    if (position_bias < POSITION_BIAS_MIN) position_bias = POSITION_BIAS_MIN;

    /* If robot rolled forward (+ position), bias setpoint to lean back */
    float effective_setpoint = SETPOINT_BASE - position_bias;

    /* --- 5. PID controller --- */
    float error = tilt_angle - effective_setpoint;

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

    /* --- 6. Drive motors (also updates motor_dir for encoder ISR) --- */
    Motors_Drive(pid_output);

    /* --- 7. Update telemetry snapshot --- */
    telemetry.angle              = tilt_angle;
    telemetry.acc_angle          = acc_angle_deg;
    telemetry.gyro_rate          = gyro_y_dps;
    telemetry.effective_setpoint = effective_setpoint;
    telemetry.position           = position;
    telemetry.pid_output         = pid_output;
    telemetry.error              = error;
}

void Control_GetTelemetry(Control_Telemetry_t *out) {
    *out = *(Control_Telemetry_t *)&telemetry;
}

void Control_ResetEncoders(void) {
    encoder_left  = 0;
    encoder_right = 0;
}

void Control_OnEncoderPulse(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) {
        encoder_left += motor_dir;
    }
    else if (GPIO_Pin == GPIO_PIN_11) {
        encoder_right += motor_dir;
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize all control subsystems */
  Control_Init(&hi2c1, &hspi1, &htim3);

  /* Print startup banner */
  char *msg = "\r\n=== Self-Balancing Robot ===\r\n"
              "Format: angle, setpoint, position, pid_out\r\n\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* Reset encoder counts (robot is at "home") */
  Control_ResetEncoders();

  /* Start the 200 Hz control ISR */
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (display_flag) {
          display_flag = 0;

          Control_Telemetry_t t;
          Control_GetTelemetry(&t);

          char buf[100];
          int len = sprintf(buf, "%.2f,%.2f,%.2f,%.2f\r\n",
                            t.angle, t.effective_setpoint,
                            t.position, t.pid_output);
          HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ============================================================
 *  TIM2 ISR - 200 Hz control loop
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    static uint8_t uart_div = 0;

    /* Run one full balance cycle (sensors -> filter -> PID -> motors) */
    Control_RunStep();

    /* Throttle UART output to ~10 Hz (every 20th tick) */
    if (++uart_div >= 20) {
        uart_div = 0;
        display_flag = 1;
    }
}

/* ============================================================
 *  EXTI ISR - Encoder pulse handler
 *  Called by HAL when PD10 or PD11 sees a rising edge.
 * ============================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Control_OnEncoderPulse(GPIO_Pin);
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
