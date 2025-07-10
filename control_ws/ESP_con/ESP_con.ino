#include <Wire.h>
#include <Adafruit_MCP4725.h>

// === 두 개의 MCP4725 DAC 객체 생성 ===
Adafruit_MCP4725 dac1;  // I²C 주소 0x60
Adafruit_MCP4725 dac2;  // I²C 주소 0x61

// === 이전 출력 값 저장 ===
uint16_t lastVoltage1 = 0;
uint16_t lastVoltage2 = 0;

void setup() {
  Serial.begin(2000000);
  Serial.setRxBufferSize(512);
  Serial.setTxBufferSize(512);
  while (!Serial);
  Serial.println("ESP32: MCP4725 듀얼 DAC 제어 시작");

  Wire.begin(21, 22, 400000);
  dac1.begin(0x60);
  dac2.begin(0x61);
}

void loop() {
  // 들어온 데이터가 있으면 한 줄씩 읽어서 바로 처리
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) {
      continue;
    }

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("형식 오류! 값 두 개를 쉼표로 구분하세요.");
      continue;
    }

    uint16_t voltage1 = input.substring(0, commaIndex).toInt();
    uint16_t voltage2 = input.substring(commaIndex + 1).toInt();
    voltage1 = min(voltage1, (uint16_t)4095);
    voltage2 = min(voltage2, (uint16_t)4095);

    // 값이 변경된 경우에만 DAC 출력
    if (voltage1 != lastVoltage1) {
      dac1.setVoltage(voltage1, false);
      lastVoltage1 = voltage1;
    }
    if (voltage2 != lastVoltage2) {
      dac2.setVoltage(voltage2, false);
      lastVoltage2 = voltage2;
    }

    Serial.printf("DAC1: %u | DAC2: %u\n", voltage1, voltage2);
  }

  // (필요한 주기 작업이 있으면 여기 millis() 기반으로)
}
