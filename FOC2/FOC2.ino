#include "ArduinoJson.h"
#define MIN_POS 0  // границы движения сервопривода в градусах
#define MAX_POS 270

#define ENCODER_PIN A0    // пин потенциометра или энкодера
#define MX1508_IN1_PIN 5  // пины драйвера двигателей
#define MX1508_IN2_PIN 6

#define ENCODER_SCALE 270.f / 1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270]

#include <EncButton.h>
EncButton<EB_TICK, 2, 3> enc;  // просто энкодер

float alpha = 0.4;  // коэффициент фильтрации при измерении положения с потенциометра, для AS5600 можно поставить 1.0

float kP = 1;  //50 пропорциональный коэффициент PID-регулятора
float kI = 0;  // интегральный коэффициент PID-регулятора
float kD = 0;  //4.5 дифференциальный коэффициент PID-регулятора

// json буфер для пакетов от ПК
StaticJsonDocument<200> jsondoc;

// глобальные переменные
float position = 1;      // задаваемое положение
float realPosition = 0;  // реальное положение сервопривода
int16_t motorPwm = 0;    // Заполнение ШИМ, подаваемое на мотор
uint32_t pidTimer = 0;   // таймер обновления pid-регулятора


void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  pinMode(2, INPUT); //D2 и D3 подтяжка пинов энкодера
  pinMode(3, INPUT);
  setMotorPwm(1, 0);
  pidTimer = millis();

  
  enc.setEncType(EB_HALFSTEP);
  attachInterrupt(0, isr, CHANGE);  // D2
  attachInterrupt(1, isr, CHANGE);  // D3
}


void isr() {
  enc.tickISR();  // в прерывании вызываем тик ISR
}


void loop() {
  enc.tick();        // отработка
  // получаем пакеты по uart
  if (Serial.available() > 0) {
    DeserializationError err = deserializeJson(jsondoc, Serial);
    if (err == DeserializationError::Ok) {               // если пакет принят
      position = (float)jsondoc["angle"];                // получаем угол поворота
      position = constrain(position, MIN_POS, MAX_POS);  // ограничиваем угол поворота сервопривода
    } else {
      while (Serial.available() > 0) Serial.read();
    }  // чистим буфер
  }


  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 5) {
    realPosition = getAngle();

    float temp_ki = kI;
    if (abs(motorPwm) == 255) temp_ki = 0.f;  // если заполнение ШИМ ушло в насыщение, то отключаем интегральную составляющую

    motorPwm = (int16_t)pid(realPosition, position, kP, temp_ki, kD, (millis() - pidTimer) / 1000.f);  // получаем заполнение ШИМ с PID-регулятора
    // motor direction
    int dir = 0;
    if (motorPwm < 0) {
      dir = 1;
    } else if (motorPwm > 0) {
      dir = -1;
    } else {
      dir = 0;
    }
    Serial.print(motorPwm);
    Serial.print(',');
    motorPwm = abs(motorPwm);
    if (motorPwm > 1024) {
      motorPwm = 1024;
    }

    motorPwm = map(motorPwm, 0, 1024, 90, 255);  // 90 ограничиваем диапаон ШИМа до допустимого диапазона

    setMotorPwm(dir, motorPwm);  // подаем ШИМ на мотор
    pidTimer = millis();
    //Serial.print(motorPwm);
    //Serial.print(',');
    Serial.print(dir);
    Serial.print(',');
    Serial.print(position, DEC);  // выводим заданную позицию
    Serial.print(',');
    Serial.println(realPosition, DEC);  // и реальную
  }
}


float pid(float input, float trgt, float kp, float ki, float kd, float dt) {
  static float integral = 0.f;   // храним значение суммы интегральной компоненты
  static float lastError = 0.f;  // и предыдущую ошибку регулирования для дифференциирования

  if (dt == 0.f) {
    integral = 0.f;
    lastError = 0.f;
    return 0.f;
  }

  float error = trgt - input;             // получаем ощибку регулирования
  integral += ki * error * dt;            // пересчитываем интегральную сумму
  float diff = (error - lastError) / dt;  // ищем дифференциал

  lastError = error;
  return kp * error + integral + kd * diff;  // считаем и выводим результат
}


void setMotorPwm(int dir, int16_t pwm) {  // функция подающая ШИМ на драйвер двигателей MX1508
  if (dir < 0) {
    analogWrite(MX1508_IN1_PIN, LOW);
    analogWrite(MX1508_IN2_PIN, pwm);
  } else if (dir > 0) {
    analogWrite(MX1508_IN1_PIN, pwm);
    analogWrite(MX1508_IN2_PIN, LOW);
  } else {
    analogWrite(MX1508_IN1_PIN, HIGH);
    analogWrite(MX1508_IN2_PIN, HIGH);
  }
}


float getAngle() {  // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float angle = enc.counter;
  //angle = lpFilter(angle, oldAngle, alpha);   // фильтруем показания с потенциометра, если надо
  oldAngle = angle;
  return angle;
}


float lpFilter(float value, float oldValue, float alp) {
  return oldValue * (1.f - alp) + alp * value;
}
