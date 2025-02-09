#include "CytronMotorDriver.h"

// 핀 설정
const int pwmPin_r= 3;
const int dirPin_r= 4;
const int pwmPin_l= 5;
const int dirPin_l= 6;
const int pwmPin_s = 8;            
const int dirPin_s = 9;            

CytronMD motor1(PWM_DIR, pwmPin_r, dirPin_r);
CytronMD motor2(PWM_DIR, pwmPin_l, dirPin_l);
CytronMD motor3(PWM_DIR, pwmPin_s, dirPin_s);

void controlDriveMotor(char command) {
  if (command == 'W') {
    // 전진
    motor1.setSpeed(100);
    motor2.setSpeed(100);
  }else if (command == 'X') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(-30);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(-30);
  }
   else if (command == 'S') {
    // 정지
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  } else if (command == 'F') {
    // 왼쪽 앞으로 (왼쪽 모터를 더 느리게, 오른쪽 모터는 정상 속도)
    motor1.setSpeed(100);
    motor2.setSpeed(100);  // 왼쪽 모터를 더 느리게 하여 왼쪽으로 회전
  } else if (command == 'G') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(100);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(100);
  }else if (command == 'H') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(-30);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(-30);
  }else if (command == 'I') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(-30);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(-30);
  }
  else if (command == 'B') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(150);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(150);
  }
  else if (command == 'N') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(150);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(150);
  }
  else if (command == 'M') {
    // 오른쪽 앞으로 (오른쪽 모터를 더 느리게, 왼쪽 모터는 정상 속도)
    motor1.setSpeed(150);  // 오른쪽 모터를 더 느리게 하여 오른쪽으로 회전
    motor2.setSpeed(150);
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'W' || input == 'F' || input == 'G' || input == 'S'|| input=='X' || input=='B' || input=='N' || input=='M') {
      controlDriveMotor(input);  // 이동 제어
    } else if (input == 'A') {
      motor3.setSpeed(-100);  // 왼쪽으로 조향
    } else if (input == 'D') {
      motor3.setSpeed(100);  // 오른쪽으로 조향
    } else {
      motor3.setSpeed(0);  // 조향 움직임 멈춤
    }
  }
}
