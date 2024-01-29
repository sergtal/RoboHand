#include <EncButton.h>     // библиотека энкодера гайвера
#include "GyverFilters.h"  // библиотека фильтров гайвера
#define MX1508_IN1_PIN 5   // пины драйвера двигателей
#define MX1508_IN2_PIN 6
// просто энкодер
Encoder e(2, 3);
float target;
float speed = 0;
float pos;
// глобальные переменные
int16_t motorPwm = 0;    // заполнение ШИМ, подаваемое на мотор
const int potPin = A0;   // Пин, к которому подключен потенциометр
float realPosition = 0;  // реальное положение сервопривода (градусы)
float realSpeed = 0;     // реальная скорость сервопривода (градусы/с)
//////////////
const int encoderPin = 2;                    // Пин, к которому подключен выход энкодера
volatile unsigned long prevTime = 0;         // Переменная для хранения предыдущего времени прерывания
volatile unsigned long deltaTime = 0;        // Переменная для хранения времени между импульсами
volatile float angularSpeed = 0.0;           // Переменная для хранения вычисленной угловой скорости
const unsigned long stopThreshold = 150000;  // Порог времени для считывания импульсов (в микросекундах) если енкодер остановлен но есть ложные выпады
const unsigned long checkInterval = 150000; // Период проверки остановки энкодера (в микросекундах)

void setup() {
  Serial.begin(115200);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  e.setEncISR(true);
  e.setEncType(EB_STEP1);
  //attachInterrupt(0, isr, CHANGE);  // прерывание энкодера RX0
  attachInterrupt(1, isr, CHANGE);  // прерывание энкодера RX1
  /////////////
}


void loop() {
  // put your main code here, to run repeatedly:
  e.tick();
  // Проверяем, прошло ли достаточно времени с последнего вызова handleEncoder
  unsigned long currentTime = micros();
  if (currentTime - prevTime > checkInterval) {
    angularSpeed = 0.0;  // Считаем, что энкодер остановлен
  }
  int potValue = analogRead(potPin);            // Считываем значение с потенциометра
  setMotorPwm(map(potValue, 0, 1023, 0, 255));  // Преобразуем значение в диапазон скорости мотора (0-255)
  ///////////
}


void setMotorPwm(int16_t pwm) {  // функция подающая ШИМ на драйвер двигателей MX1508
  if (pwm >= 0) {
    analogWrite(MX1508_IN1_PIN, LOW);
    analogWrite(MX1508_IN2_PIN, abs(pwm));
  } else {
    analogWrite(MX1508_IN1_PIN, abs(pwm));
    analogWrite(MX1508_IN2_PIN, LOW);
  }
}


void isr() {
  e.tickISR();                           // в прерывании вызываем тик ISR
  unsigned long currentTime = micros();  // Измеряем текущее время в микросекундах
  deltaTime = currentTime - prevTime;    // Вычисляем время между импульсами
  // Проверяем, прошло ли достаточно времени для считывания импульсов
  if (deltaTime < stopThreshold) {
    angularSpeed = 1000000.0 / deltaTime;
  } else {
    angularSpeed = 0.0;  // Если прошло слишком много времени, считаем, что энкодер остановлен
  }
  prevTime = currentTime;        // Обновляем предыдущее время
  Serial.println(angularSpeed);  // Выводим скорость в монитор порта (для отладки)
}
