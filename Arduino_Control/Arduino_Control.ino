#include "CytronMotorDriver.h"

#include <ros.h>

#include <std_msgs/Int16.h>

// 핀 설정

//오른쪽모터

const int pwmPin_r= 3;

const int dirPin_r= 4;

//왼쪽모터

const int pwmPin_l= 5;

const int dirPin_l= 6;

//스티어모터

const int pwmPin_s = 8;            // PWM 신호 핀

const int dirPin_s = 9;            // 방향 제어 핀

const int potPin = A10;           // 가변저항 입력 핀

// 조향각 관련 상수

const int minimumPotValue=312;

const int maximumPotValue=742;

const int centerPotValue = (minimumPotValue+maximumPotValue)/2;  // 가변저항의 중앙값

const float angleperres = float(1024)/float(270); // 가변저항 1도 당 저항값의 변화

const float potToSteeringRatio = 1.4;  // 가변저항과 조향각 비율 (7도 : 5도) -> 추후 수정

const float maxSteeringAngle = 30.0;   // 최대 조향각

//실시간으로 값을 받아와야 함 플러스는 왼쪽 마이너스는 오른쪽

float desiredSteeringAngle = 0.0;   // 원하는 조향각

// PID 제어 상수

float Kp = 7.0;   // 비례 상수

float Ki = 0;   // 적분 상수

float Kd = 0.4;   // 미분 상수

// PID 제어 변수

float prevError = 0;

float integral = 0;

unsigned long lastTime=millis();

//기본 potvalue -> potentialmeter 값

int potValue=centerPotValue;

//ROS 통신 관련 설정

ros::NodeHandle nh;

int16_t angle, speed;

// Configure the motor driver.

CytronMD motor1(PWM_DIR, pwmPin_r, dirPin_r);  // -> 오른쪽뒷바퀴

CytronMD motor2(PWM_DIR, pwmPin_l, dirPin_l); // -> 왼쪽뒷바퀴

CytronMD motor3(PWM_DIR, pwmPin_s, dirPin_s);  // -> 조향모터

float computeAngle();

float computePID(float error);

void controlSteeringMotor(float controlSignal);

void angleCallback(const std_msgs::Int16& msg);

void speedCallback(const std_msgs::Int16& msg);

float computeAngle(){

desiredSteeringAngle = (float) angle;       // 실수(float)로 변환

Serial.print("입력된 조향각: ");

Serial.println(desiredSteeringAngle);         // 입력된 값 출력

// 현재 가변저항 값 읽기

int potValue_temp=analogRead(potPin);

if (280<potValue_temp&&potValue_temp<800)

{

potValue=potValue_temp;

}

Serial.print("가변저항: ");

Serial.println(potValue);

// 가변저항 값을 실제 조향각으로 변환

float currentSteeringAngle = (centerPotValue-potValue) / angleperres / potToSteeringRatio;

// 오차 계산

float error = desiredSteeringAngle - currentSteeringAngle;

// 상태 출력 (디버깅용)

Serial.print("Current Steering Angle: ");

Serial.print(currentSteeringAngle);

Serial.print(", Desired Steering Angle: ");

Serial.print(desiredSteeringAngle);

return error;

}

float computePID(float error) {

// 현재 시간

unsigned long currentTime = millis();

float elapsedTime = (currentTime - lastTime) / 1000.0;  // 시간 차이 (초)

// 비례

float Pout = Kp * error;

// 적분

integral += error * elapsedTime;

float Iout = Ki * integral;

// 미분

float derivative = (error - prevError) / elapsedTime;

float Dout = Kd * derivative;

// PID 출력 합산

float output = Pout + Iout + Dout;

// 이전 값 갱신roslaunch ntrip_ros ntrip_ros.launch

prevError = error;

lastTime = currentTime;

return output;

}

void controlSteeringMotor(float controlSignal) { //문제 발생

// 상태 출력 (디버깅용)

Serial.print(", Control Signal: ");

Serial.println(controlSignal);

// PWM 값 계산 (제어 신호에 비례, 0~255 사이로 제한)

float constrain_signal = constrain(controlSignal, -254, 254);

Serial.print("방향 및 pwmvalue:");

if (constrain_signal>0)

{

Serial.print("오른쪽으로 힘  ");

}

else

{

Serial.print("왼쪽으로 힘  ");

}

Serial.println(constrain_signal);

motor3.setSpeed(constrain_signal); //pwm값 들어감.

Serial.println();

}

// callback 함수: ROS에서 받은 데이터를 처리

void angleCallback(const std_msgs::Int16& msg){

angle = msg.data;

Serial.print("Angle received: ");

Serial.println(angle);

}

void speedCallback(const std_msgs::Int16& msg){

speed = msg.data;

Serial.print("Speed received: ");

Serial.println(speed);

}

// 각 토픽을 구독 (Subscriber)로 설정

ros::Subscriber<std_msgs::Int16> sub_angle("/steering", angleCallback);

ros::Subscriber<std_msgs::Int16> sub_speed("/pwm", speedCallback);

void setup() {

Serial.begin(57600);

nh.initNode();

// 각 Subscriber를 ROS에 등록

nh.subscribe(sub_angle);

nh.subscribe(sub_speed);

}

void loop() {

// ROS와의 통신을 유지

nh.spinOnce();

//// 스티어링 제어 파트

// 오차 계산

float error = computeAngle();

// PID 제어 신호 계산

float controlSignal = computePID(error);

// 조향 모터 제어

controlSteeringMotor(controlSignal);

//// 구동모터 제어 파트-> 추가적으로 더 필요

motor1.setSpeed(speed);

motor2.setSpeed(speed);

delay(100); //-> 추후 줄여야함

}