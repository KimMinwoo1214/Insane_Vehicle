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

int speed = 0;  // 이동 속도 변수
int turn_speed = 0; // 회전 속도 변수

void controlDriveMotor(String command) {
  command.trim();  // 개행 문자 제거
  // Serial.print("Received Command: "); 
  // Serial.println(command);  // 수신된 명령을 시리얼 모니터에 출력

  if (command.startsWith("forward:")) {
    speed = command.substring(8).toInt();  // 숫자만 추출하여 정수 변환
  } else if (command.startsWith("backward:")) {
    speed = -command.substring(9).toInt();  // 후진 속도 값 파싱
  } else if (command == "stop") {
    speed = 0;
    turn_speed = 0;
  } else if (command.startsWith("cw:")) {
    turn_speed = command.substring(3).toInt(); // 시계 방향 회전 속도 설정
  } else if (command.startsWith("ccw:")) {
    turn_speed = -command.substring(4).toInt(); // 반시계 방향 회전 속도 설정
  } else {
    Serial.println("⚠️ 오류: 알 수 없는 명령");
  }

  // 모터 속도 설정
  // Serial.print("Motor Speed: ");
  // Serial.println(speed);
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
}

void setup() {
  Serial.begin(9600);
  Serial.println("🔹 아두이노 모터 컨트롤러 시작");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // '\n'까지 문자열 읽기
    controlDriveMotor(command);
  }
}
