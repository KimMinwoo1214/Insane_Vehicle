#define STEERING_PIN 9   // PWM 출력 핀 (490Hz)
#define THROTTLE_PIN 10

int pwmValue1 = 87;
int pwmValue2 = 0;

void setup() {
    pinMode(STEERING_PIN, OUTPUT);
    pinMode(THROTTLE_PIN, OUTPUT);
    Serial.begin(9600);  // 시리얼 통신 시작
}

void loop() {
    if (Serial.available() > 0) {  // 시리얼 입력이 있으면 실행
        String data = Serial.readStringUntil('\n');
        int steering, throttle;

        if (sscanf(data.c_str(), "%d,%d", &steering, &throttle) == 2) {
            pwmValue1 = constrain(steering, 0, 255);
            pwmValue2 = constrain(throttle, 0, 255);
        }

        // 디버깅용 출력
        Serial.print("Steering Value: ");
        Serial.print(pwmValue1);
        Serial.print(" | Throttle Value: ");
        Serial.println(pwmValue2);
    }

    analogWrite(STEERING_PIN, pwmValue1);  // PWM 신호 유지
    analogWrite(THROTTLE_PIN, pwmValue2);
}
