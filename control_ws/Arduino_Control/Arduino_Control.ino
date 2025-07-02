#include <Wire.h>
#include <Adafruit_MCP4725.h>

// === 두 개의 MCP4725 객체 생성 ===
Adafruit_MCP4725 dac1;  // 주소 0x60
Adafruit_MCP4725 dac2;  // 주소 0x61

void setup() {
  Serial.begin(115200);
  while (!Serial); // 시리얼 모니터 연결될 때까지 대기

  Serial.println("두 개의 Voltage 값을 입력하세요 (0~4095, 쉼표로 구분):");

  Wire.begin();               // I2C 시작
  Wire.setClock(400000);      // 400 kHz로 속도 올려서 응답 속도 향상

  dac1.begin(0x60);           // DAC1 초기화
  dac2.begin(0x61);           // DAC2 초기화
}

void loop() {
  String latestInput = "";

  // === 시리얼 버퍼에 쌓인 모든 입력 비우고 최신 값만 가져오기 ===
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

      // DAC에 최신 전압 값 설정
      dac1.setVoltage(voltage1, false);
      dac2.setVoltage(voltage2, false);

      // 디버깅 출력
      Serial.print("DAC1: ");
      Serial.print(voltage1);
      Serial.print(" | DAC2: ");
      Serial.println(voltage2);
    } else {
      Serial.println("잘못된 형식! 쉼표로 두 값을 구분하세요.");
    }
  }

  // === Non-blocking 구조 ===
  // 필요하면 여기서 millis()로 주기적 모니터링 가능
}

