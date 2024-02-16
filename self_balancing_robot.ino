#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define leftMotorPWMPin   4
#define leftMotor1   2
#define leftMotor2   3
#define rightMotorPWMPin  5
#define rightMotor1  6
#define rightMotor2  7

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.03
#define Ki  2
#define sampleTime  0.005
#define targetAngle 0

const float RAD_TO_DEG = 180 / PI;
const float FILTER_CONSTANT = 0.9934;

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  Serial.println(rightMotorSpeed);
  if (leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
  } else {
    analogWrite(leftMotorPWMPin, -leftMotorSpeed);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
  }
  if (rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  } else {
    analogWrite(rightMotorPWMPin, -rightMotorSpeed);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  }
}

void init_PID() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 9999;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void setup() {
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(13, OUTPUT);
  mpu.initialize();
  mpu.setYAccelOffset(-825);
  mpu.setZAccelOffset(1171);
  mpu.setXGyroOffset(-79);
  init_PID();
  Serial.begin(19200);
}

void loop() {
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
}

ISR(TIMER1_COMPA_vect) {
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = static_cast<float>(gyroRate) * sampleTime;
  currentAngle = FILTER_CONSTANT * (prevAngle + gyroAngle) + (1 - FILTER_CONSTANT) * accAngle;
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;
  count++;
  if (count == 200) {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
