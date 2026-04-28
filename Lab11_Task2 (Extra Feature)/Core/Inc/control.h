/**
  ******************************************************************************
  * @file    control.h
  * @brief   Self-balancing robot control module.
  *
  *          Combines all control logic into one module:
  *            - IMU drivers (gyro via SPI, accel via I2C)
  *            - Complementary filter
  *            - PID controller
  *            - Motor driver (L298N H-bridge)
  *            - Encoder pulse counting (position feedback)
  ******************************************************************************
  */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>
#include "main.h"

/* ============================================================
 *  Tunable Parameters
 * ============================================================ */

/* Sampling time (200 Hz control loop) */
#define DT 0.005f

/* PID gains */
#define KP  30.0f
#define KI  0.2f
#define KD  1.5f

/* Mechanical setpoint - the "balanced" angle in degrees */
#define SETPOINT_BASE -3.5f

/* PID output limits (matches TIM3 ARR = 999) */
#define PID_OUT_MAX  999.0f
#define PID_OUT_MIN -999.0f

/* Integral anti-windup limits */
#define INTEGRAL_MAX  500.0f
#define INTEGRAL_MIN -500.0f

/* Complementary filter weights (gyro_w + acc_w = 1.0) */
#define COMP_GYRO_WEIGHT  0.98f
#define COMP_ACC_WEIGHT   0.02f

/* Encoder position feedback */
#define POSITION_KP        0.003f   // Tilt bias per pulse (deg/pulse)
#define POSITION_BIAS_MAX  3.0f     // Cap on position contribution
#define POSITION_BIAS_MIN -3.0f

/* ============================================================
 *  Telemetry struct - shared between ISR and main loop
 *  Fields are updated by Control_RunStep() and read by main()
 * ============================================================ */
typedef struct {
    float angle;            // Current fused tilt angle (degrees)
    float acc_angle;        // Raw accelerometer angle (degrees)
    float gyro_rate;        // Gyro Y-axis rate (deg/s)
    float effective_setpoint;  // Setpoint after position bias
    float position;         // Average encoder position (pulses)
    float pid_output;       // Final PID output (PWM units)
    float error;            // Current angle error
} Control_Telemetry_t;

/* ============================================================
 *  Public API
 * ============================================================ */

/**
 * @brief Initialize all subsystems: IMU, motors, internal state.
 * @param hi2c  I2C handle for accelerometer
 * @param hspi  SPI handle for gyroscope
 * @param htim  TIM3 handle for motor PWM
 */
void Control_Init(I2C_HandleTypeDef *hi2c,
                  SPI_HandleTypeDef *hspi,
                  TIM_HandleTypeDef *htim);

/**
 * @brief Run one full control cycle. Called from the 200 Hz timer ISR.
 *        Reads IMU, runs filter + PID, drives motors.
 */
void Control_RunStep(void);

/**
 * @brief Copy the latest telemetry snapshot for logging.
 *        Safe to call from main loop while ISR is active.
 */
void Control_GetTelemetry(Control_Telemetry_t *out);

/**
 * @brief Reset encoder counts. Call after placing the robot at "home".
 */
void Control_ResetEncoders(void);

/**
 * @brief Encoder ISR handler. Call from HAL_GPIO_EXTI_Callback for PD10/PD11.
 * @param GPIO_Pin  Either GPIO_PIN_10 (left) or GPIO_PIN_11 (right)
 */
void Control_OnEncoderPulse(uint16_t GPIO_Pin);

#endif /* INC_CONTROL_H_ */