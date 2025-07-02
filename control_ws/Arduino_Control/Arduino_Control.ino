#include <Wire.h>
#include <Adafruit_MCP4725.h>

// === 두 개의 MCP4725 DAC 객체 생성 ===
Adafruit_MCP4725 dac1;  // 주소 0x60
Adafruit_MCP4725 dac2;  // 주소 0x61

// === 이전 값 저장 ===
uint16_t lastVoltage1 = 0;
uint16_t lastVoltage2 = 0;

void setup() {
  Serial.begin(2000000);
  while (!Serial);  // 시리얼 모니터 연결될 때까지 대기

  Serial.println("두 개의 Voltage 값을 입력하세요 (0~4095, 쉼표로 구분):");

  Wire.begin();               // I2C 시작
  Wire.setClock(400000);      // I2C 속도 400kHz로 향상

  dac1.begin(0x60);           // DAC1 초기화
  dac2.begin(0x61);           // DAC2 초기화
}

void loop() {
  String latestInput = "";

  // === 시리얼 버퍼에 쌓인 모든 입력 비우고 최신 값만 저장 ===
  while (Serial.available() > 0) {
    latestInput = Serial.readStringUntil('\n');
    latestInput.trim();
  }

  if (latestInput.length() > 0) {
    int commaIndex = latestInput.indexOf(',');
    if (commaIndex != -1) {
      String voltage1Str = latestInput.substring(0, commaIndex);
      String voltage2Str = latestInput.substring(commaIndex + 1);

      uint16_t voltage1 = voltage1Str.toInt();
      uint16_t voltage2 = voltage2Str.toInt();

      // 값 범위 제한 (0~4095)
      if (voltage1 > 4095) voltage1 = 4095;
      if (voltage2 > 4095) voltage2 = 4095;

      // === 값이 바뀐 경우에만 DAC 출력 ===
      if (voltage1 != lastVoltage1) {
        dac1.setVoltage(voltage1, false);
        lastVoltage1 = voltage1;
      }

      if (voltage2 != lastVoltage2) {
        dac2.setVoltage(voltage2, false);
        lastVoltage2 = voltage2;
      }

      // 디버깅 출력
      Serial.print("DAC1: ");
      Serial.print(voltage1);
      Serial.print(" | DAC2: ");
      Serial.println(voltage2);

    } else {
      Serial.println("형식 오류! 쉼표로 두 값을 구분하세요.");
    }
  }

  // === Non-blocking ===
  // 필요하다면 millis()로 주기 작업 추가 가능
}
