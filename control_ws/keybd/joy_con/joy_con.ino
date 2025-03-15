#include "CytronMotorDriver.h"

const int pwmPin_r = 3;
const int dirPin_r = 4;
const int pwmPin_l = 5;
const int dirPin_l = 6;
const int pwmPin_s = 8;
const int dirPin_s = 9;

// Cytron 모터 드라이버 객체 생성
CytronMD motorLeft(PWM_DIR, pwmPin_l, dirPin_l);   // 왼쪽
CytronMD motorRight(PWM_DIR, pwmPin_r, dirPin_r); // 오른쪽
CytronMD motorSteer(PWM_DIR, pwmPin_s, dirPin_s); // 스티어링

void setup() {
  Serial.begin(9600);
  Serial.println("🔹 Arduino Motor Controller Initialized");
}

void loop() {
  if (Serial.available() > 0) {
    // '\n' 기준으로 문자열 받기
    String command = Serial.readStringUntil('\n');
    command.trim(); // 공백, 개행 문자 제거
    parseCommand(command);
  }
}

void parseCommand(String cmd) {
  // 예) "drive: -100 120" -> 왼쪽 모터 -100, 오른쪽 모터 120
  // 예) "steer: 50"       -> 스티어링 모터 50
  // 예) "stop"            -> 정지
  // 각 문자열 구문을 파싱

  if (cmd.startsWith("drive:")) {
    // "drive:" 뒤에 오는 값 2개 (왼쪽, 오른쪽) 파싱
    // "drive: -100 120" 구조라고 가정
    String valuePart = cmd.substring(6); 
    valuePart.trim(); 
    int spaceIndex = valuePart.indexOf(' ');

    if (spaceIndex == -1) {
      Serial.println("Invalid drive command format!");
      return;
    }

    String leftStr = valuePart.substring(0, spaceIndex);
    String rightStr = valuePart.substring(spaceIndex + 1);

    int leftVal = leftStr.toInt(); 
    int rightVal = rightStr.toInt();

    motorLeft.setSpeed(leftVal);
    motorRight.setSpeed(rightVal);

    Serial.print("Driving => Left: ");
    Serial.print(leftVal);
    Serial.print(", Right: ");
    Serial.println(rightVal);
  }

  else if (cmd.startsWith("steer:")) {
    // "steer: 50" 구조
    String valuePart = cmd.substring(6);
    valuePart.trim();
    int steerVal = valuePart.toInt();

    motorSteer.setSpeed(steerVal);

    Serial.print("Steering => ");
    Serial.println(steerVal);
  }

  else if (cmd == "stop") {
    // 모든 모터 정지
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    motorSteer.setSpeed(0);
    Serial.println("All motors stopped!");
  }

  else {
    // 정의되지 않은 명령
    Serial.print("Invalid Command: ");
    Serial.println(cmd);
  }
}
