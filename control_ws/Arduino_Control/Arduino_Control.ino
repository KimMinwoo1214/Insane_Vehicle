#include <Wire.h>
#include <Adafruit_MCP4725.h>

// 두 개의 MCP4725 객체 생성
Adafruit_MCP4725 dac1;  // 주소 0x60 사용
Adafruit_MCP4725 dac2;  // 주소 0x61 사용

void setup() {
  Serial.begin(115200);
  while (!Serial); // 시리얼 모니터가 연결될 때까지 대기
  Serial.println("두 개의 Voltage 값을 입력하세요 (0~4095, 공백으로 구분):");
  
  Wire.begin();  // I2C 시작
  Wire.setClock(400000);  // 400 kHz로 올려서 응답 속도 및 안정성 향상


  // 각 DAC 초기화 (각자의 주소 지정)
  dac1.begin(0x60);
  dac2.begin(0x61);
}

void loop() {
  if (Serial.available() > 0) {
    // 시리얼 입력 읽기
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // 공백을 기준으로 두 값을 분리
    int spaceIndex = input.indexOf(',');
    if (spaceIndex != -1) {
      String voltage1Str = input.substring(0, spaceIndex);
      String voltage2Str = input.substring(spaceIndex + 1);
      
      uint16_t voltage1 = voltage1Str.toInt();
      uint16_t voltage2 = voltage2Str.toInt();
      
      // 값의 범위를 0~4095로 제한
      if (voltage1 > 4095) voltage1 = 4095;
      if (voltage2 > 4095) voltage2 = 4095;
      
      // 각 DAC에 전압 값 설정
      dac1.setVoltage(voltage1, false);
      dac2.setVoltage(voltage2, false);
      
      Serial.print("DAC1을 ");
      Serial.print(voltage1);
      Serial.print("으로, DAC2를 ");
      Serial.print(voltage2);
      Serial.println("으로 설정했습니다.");
    } else {
      Serial.println("두 개의 값을 공백으로 구분하여 입력하세요.");
    }
  }
}
