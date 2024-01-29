// Подключаемые библиотеки
#include <GyverPID.h>
// Пины дляправления мотором
const int motorPin1 = 4;
const int motorPin2 = 5;
#define ENC_A 2       // пин энкодера
#define ENC_B 3       // пин энкодера
#define ENC_TYPE 1    // тип энкодера, 0 или 1

volatile boolean state0, lastState, turnFlag;
volatile int pos = 0;

// Константы для настройки PID контроллера
const float kp = 1.0;
const float ki = 0.0;
const float kd = 0.0;

// Переменные для PID контроллера
float output = 0.0;

// Переменные для управления мотором
int motorSpeed = 0;
int motorDirection = 0;

int potPin = A0; // Пин, к которому подключен потенциометр
// Объекты для работы с энкодером и PID контроллером

GyverPID regulator(kp, ki, kd);
// Функция для настройки пинов
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  Serial.begin(9600);
  regulator.setMode(0); 
  attachInterrupt(0, speedEncoder, CHANGE); //вызов енкодера
}

// Функция для управления мотором
void motorControl() {
  if (output > 0) {
    motorSpeed = map(output, 0, 255, 128, 255);
    motorDirection = 1;
  } else if (output < 0) {
    motorSpeed = map(output, -255, 0, 128, 255);
    motorDirection = -1;
  } else {
    motorSpeed = 0;
    motorDirection = 0;
  }
  analogWrite(motorPin1, motorSpeed);
  analogWrite(motorPin2, 0);
}
void speedEncoder() {
  state0 = digitalRead(ENC_A);
  if (state0 != lastState) {
  turnFlag = !turnFlag;
  if (turnFlag)
    pos += (digitalRead(ENC_B) != lastState) ? -1 : 1;
  lastState = state0;
}
}
// Основная функция
void loop() {
  
  regulator.input = pos;
  regulator.setpoint = analogRead(potPin);
  output = regulator.getResult();

  motorControl();
  Serial.print(" Setpnt: ");
  Serial.print(regulator.setpoint);
  Serial.print(" Input: ");
  Serial.print(regulator.input);
  Serial.print(" Output: ");
  Serial.print(output);
  Serial.print(" Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" Motor Direction: ");
  Serial.println(motorDirection);
}
