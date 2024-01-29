#include <EncButton.h>     // библиотека энкодера гайвера
#include "GyverFilters.h"  // библиотека фильтров гайвера

#define MX1508_IN1_PIN 5  // пины драйвера двигателей
#define MX1508_IN2_PIN 6

// просто энкодер
Encoder e(2, 3);

//GMedian3<int> testFilter;
//GKalman testFilter(30, 50, 0.4);

// глобальные переменные
int16_t motorPwm = 0;   // заполнение ШИМ, подаваемое на мотор
const int potPin = A0;  // Пин, к которому подключен потенциометр

float realPosition = 0;  // реальное положение сервопривода (градусы)
float realSpeed = 0;     // реальная скорость сервопривода (градусы/с)
float realCurrent = 0;   // реальный ток сервопривода (амперы)

float smoothingFactor = 0.2;  //0.3
float previousSpeed = 0;

uint32_t pidTimer = 0;  // таймер обновления pid-регулятора

void setup() {
  Serial.begin(115200);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);

  e.setEncISR(true);
  e.setEncType(EB_STEP1);

  setMotorPwm(0);  //задаем скорость
  pidTimer = millis();

  attachInterrupt(0, isr, CHANGE);  // прерывание энкодера RX0
  attachInterrupt(1, isr, CHANGE);  // прерывание энкодера RX1
}


void loop() {
  // put your main code here, to run repeatedly:
  e.tick();

  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 10) {
    // пересчитываем все параметры
    float dt = (millis() - pidTimer) / 1000.f;  // пройденное с прошлого раза время
    float newPosition = e.counter;              // получаем новое положение

    realSpeed = (newPosition - realPosition) / dt;  // получаем скорость путем дифференцииорования положения по времени
    //realSpeed = testFilter.filtered((int)(realSpeed));
    //realSpeed = (smoothingFactor * previousSpeed) + ((1 - smoothingFactor) * realSpeed);
    //previousSpeed = realSpeed;
    realPosition = newPosition;
    //Serial.print(',');
    //Serial.println(realPosition, DEC);  // реальную позицию  
    Serial.println(realSpeed);  // реальную скорость


    int potValue = analogRead(potPin);            // Считываем значение с потенциометра
    setMotorPwm(map(potValue, 0, 1023, 0, 255));  // Преобразуем значение в диапазон скорости мотора (0-255)
  }


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
  e.tickISR();  // в прерывании вызываем тик ISR
}
