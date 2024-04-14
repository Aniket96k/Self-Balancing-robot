#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include <math.h>

#define leftMotorPWMPin   GPIO_PIN_4
#define leftMotor1   GPIO_PIN_2
#define leftMotor2   GPIO_PIN_3
#define rightMotorPWMPin  GPIO_PIN_5
#define rightMotor1  GPIO_PIN_6
#define rightMotor2  GPIO_PIN_7

#define TRIGGER_PIN GPIO_PIN_9
#define ECHO_PIN GPIO_PIN_8
#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.03
#define Ki  2
#define sampleTime  0.005
#define targetAngle 0

const float RAD_TO_DEG = 180 / M_PI;
const float FILTER_CONSTANT = 0.9934;

MPU6050_HandleTypeDef mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile uint8_t count = 0;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  HAL_GPIO_WritePin(GPIOA, leftMotorPWMPin, (leftMotorSpeed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, leftMotor1, (leftMotorSpeed >= 0) ? GPIO_PIN_LOW : GPIO_PIN_HIGH);
  HAL_GPIO_WritePin(GPIOB, leftMotor2, (leftMotorSpeed >= 0) ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
  HAL_GPIO_WritePin(GPIOA, rightMotorPWMPin, (rightMotorSpeed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, rightMotor1, (rightMotorSpeed >= 0) ? GPIO_PIN_LOW : GPIO_PIN_HIGH);
  HAL_GPIO_WritePin(GPIOB, rightMotor2, (rightMotorSpeed >= 0) ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

void init_PID() {
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  HAL_TIM_Base_Start_IT(&htim1);
}

void MPU6050_Init() {
  MPU6050_Init(&mpu);
  MPU6050_SetAccelOffset(&mpu, -825, 0, 1171);
  MPU6050_SetGyroOffset(&mpu, -79, 0, 0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    accAngle = atan2(accY, accZ) * RAD_TO_DEG;
    gyroRate = map(gyroX, -32768, 32767, -250, 250);
    gyroAngle = gyroRate * sampleTime;
    currentAngle = FILTER_CONSTANT * (prevAngle + gyroAngle) + (1 - FILTER_CONSTANT) * accAngle;
    error = currentAngle - targetAngle;
    errorSum += error;
    errorSum = (errorSum < -300) ? -300 : (errorSum > 300) ? 300 : errorSum;
    motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
    prevAngle = currentAngle;
    count++;
    if (count == 200) {
      count = 0;
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    accY = MPU6050_GetAccelerationY(&mpu);
    accZ = MPU6050_GetAccelerationZ(&mpu);
    gyroX = MPU6050_GetRotationX(&mpu);
    motorPower = (motorPower > 255) ? 255 : (motorPower < -255) ? -255 : motorPower;
    setMotors(motorPower, motorPower);
  }
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MPU6050_Init();
  init_PID();
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  while (1) {}
}
