#include "CytronMotorDriver.h"

// 핀 설정
const int pwmPin_r = 3;
const int dirPin_r = 4;
const int pwmPin_l = 5;
const int dirPin_l = 6;
const int pwmPin_s = 8;
const int dirPin_s = 9;

CytronMD motor1(PWM_DIR, pwmPin_r, dirPin_r);
CytronMD motor2(PWM_DIR, pwmPin_l, dirPin_l);
CytronMD motor3(PWM_DIR, pwmPin_s, dirPin_s);

int speed = 0;  // 속도 변수 (전역)

void controlDriveMotor(char command) {
  if (command == 'W') {
    // 속도를 점진적으로 증가 (50씩 증가, 최대 255)
    speed += 50;
    if (speed > 255) {
      speed = 255;
    }
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
  } else {
    // 다른 이동 명령이 들어오면 속도 초기화
    speed = 0;

    if (command == 'X') {
      motor1.setSpeed(-30);
      motor2.setSpeed(-30);
    } else if (command == 'S') {
      motor1.setSpeed(0);
      motor2.setSpeed(0);
    } else if (command == 'F') {
      motor1.setSpeed(100);
      motor2.setSpeed(100);
    } else if (command == 'G') {
      motor1.setSpeed(100);
      motor2.setSpeed(100);
    } else if (command == 'H') {
      motor1.setSpeed(-30);
      motor2.setSpeed(-30);
    } else if (command == 'I') {
      motor1.setSpeed(-30);
      motor2.setSpeed(-30);
    } else if (command == 'B') {
      motor1.setSpeed(150);
      motor2.setSpeed(150);
    } else if (command == 'N') {
      motor1.setSpeed(150);
      motor2.setSpeed(150);
    } else if (command == 'M') {
      motor1.setSpeed(150);
      motor2.setSpeed(150);
    }
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'W' || input == 'F' || input == 'G' || input == 'S' || input == 'X' || input == 'B' || input == 'N' || input == 'M') {
      controlDriveMotor(input);
    } else if (input == 'A') {
      motor3.setSpeed(-100);
    } else if (input == 'D') {
      motor3.setSpeed(100);
    } else {
      motor3.setSpeed(0);
    }
  }
}
