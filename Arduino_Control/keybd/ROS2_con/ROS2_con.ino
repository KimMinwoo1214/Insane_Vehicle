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

int speed = 0;  // 속도 변수

void controlDriveMotor(String command) {
  command.trim();  // 개행 문자 제거

  if (command.startsWith("forward:")) {
    speed = command.substring(8).toInt();  // 속도 값 파싱
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
  } else if (command.startsWith("backward:")) {
    speed = command.substring(9).toInt();  // 속도 값 파싱
    motor1.setSpeed(-speed);
    motor2.setSpeed(-speed);
  } else if (command == "stop") {
    speed = 0;
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  } else if (command == "ccw") {
    motor3.setSpeed(-100);
  } else if (command == "cw") {
    motor3.setSpeed(100);
  } else {
    motor3.setSpeed(0);
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // '\n'까지 문자열 읽기
    controlDriveMotor(command);
  }
}

