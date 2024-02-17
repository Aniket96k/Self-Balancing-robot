const int LEFT_SENSOR_PIN = 10;
const int RIGHT_SENSOR_PIN = 11;
const int MOTOR_A1 = 40;
const int MOTOR_A2 = 41;
const int MOTOR_B1 = 38;
const int MOTOR_B2 = 39;
const int PWM_PIN_A = 4;
const int PWM_PIN_B = 5;
const int PWM_FORWARD = 130;
const int PWM_BACKWARD = 120;
const int STOP_DELAY = 300;

void stat_check(int leftSensor, int rightSensor, int &leftStatus, int &rightStatus) {
  leftStatus = digitalRead(leftSensor);
  rightStatus = digitalRead(rightSensor);
}

void move(int motorA1, int motorA2, int motorB1, int motorB2, int pwmA, int pwmB) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(PWM_PIN_A, pwmA);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(PWM_PIN_B, pwmB);
}

void stopMotors(int motorA1, int motorA2, int motorB1, int motorB2, int pwm) {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(PWM_PIN_A, pwm);
  analogWrite(PWM_PIN_B, pwm);
}

void backward() {
  move(MOTOR_B1, MOTOR_B2, MOTOR_A1, MOTOR_A2, PWM_BACKWARD, PWM_BACKWARD);
}

void forward() {
  move(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2, PWM_FORWARD, PWM_FORWARD);
}

void stop() {
  stopMotors(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2, 0);
}

void loop() {
  while (forward) {
    stat_check(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, stat_left, stat_right);
    
    if (stat_right == 0 && stat_left == 1) {
      while (stat_right == 0 && stat_left == 1) {
        stopMotors(MOTOR_B1, MOTOR_B2, MOTOR_A1, MOTOR_A2, PWM_BACKWARD);
        stat_check(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, stat_left, stat_right);
      }
      delay(STOP_DELAY);
    } else if (stat_right == 1 && stat_left == 0) {
      while (stat_right == 1 && stat_left == 0) {
        stopMotors(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2, PWM_BACKWARD);
        stat_check(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, stat_left, stat_right);
      }
      delay(STOP_DELAY);
    } else if (stat_right == 0 && stat_left == 0) {
      backward();
    } else {
      forward = false;
    }
  }

  while (!forward) {
    stat_check(RIGHT_SENSOR_PIN, LEFT_SENSOR_PIN, stat_right, stat_left);
    
    if (stat_right == 0 && stat_left == 1) {
      while (stat_right == 0 && stat_left == 1) {
        stopMotors(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2, PWM_BACKWARD);
        stat_check(RIGHT_SENSOR_PIN, LEFT_SENSOR_PIN, stat_right, stat_left);
      }
      delay(STOP_DELAY);
    } else if (stat_right == 1 && stat_left == 0) {
      while (stat_right == 1 && stat_left == 0) {
        stopMotors(MOTOR_B1, MOTOR_B2, MOTOR_A1, MOTOR_A2, PWM_BACKWARD);
        stat_check(RIGHT_SENSOR_PIN, LEFT_SENSOR_PIN, stat_right, stat_left);
      }
      delay(STOP_DELAY);
    } else if (stat_right == 1 && stat_left == 1) {
      forward();
    } else {
      forward = true;
    }
  }
}
