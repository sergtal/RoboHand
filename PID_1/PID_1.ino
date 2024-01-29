// PID простой
#define MX1508_IN1_PIN 5  // D5 D6 пины драйвера двигателей
#define MX1508_IN2_PIN 6

#include <EncButton.h>         // библиотека энкодера гайвера
EncButton<EB_TICK, 2, 3> enc;  // просто энкодер

float kP = 40;  // 40 пропорциональный коэффициент PID-регулятора
float kI = 0;   // интегральный коэффициент PID-регулятора
float kD = 2;   // 2 дифференциальный коэффициент PID-регулятора

// глобальные переменные
float position = 50;     // задаваемое положение
float realPosition = 0;  // реальное положение сервопривода
int16_t motorPwm = 0;    // Заполнение ШИМ, подаваемое на мотор
uint32_t pidTimer = 0;   // таймер обновления pid-регулятора


void setup() {
  Serial.begin(115200);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  pinMode(2, INPUT);  //D2 и D3 подтяжка пинов энкодера
  pinMode(3, INPUT);
  setMotorPwm(0);
  pidTimer = millis();
  enc.setEncType(EB_HALFSTEP);      // тип энкодера
  attachInterrupt(0, isr, CHANGE);  // прерывание энкодера RX0
  attachInterrupt(1, isr, CHANGE);  // прерывание энкодера RX1
}


void isr() {
  enc.tickISR();  // в прерывании вызываем тик ISR
}


void loop() {
  unsigned long startTime = micros();
  // Ваш код, который нужно измерить
  enc.tick();  // отработка
  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 2) {
    realPosition = enc.counter;
    float temp_ki = kI;
    if (abs(motorPwm) == 255) temp_ki = 0.f;  // если заполнение ШИМ ушло в насыщение, то отключаем интегральную составляющую

    motorPwm = (int16_t)pid(realPosition, position, kP, temp_ki, kD, (millis() - pidTimer) / 1000.f);  // получаем заполнение ШИМ с PID-регулятор
    motorPwm = constrain(motorPwm, -255, 255);                                                         // ограничиваем диапаон ШИМа до допустимого диапазона
    setMotorPwm(motorPwm);                                                                             // подаем ШИМ на мотор
    pidTimer = millis();

    Serial.print(motorPwm);  // duty cycle
    Serial.print(',');
    Serial.print(position, DEC);  // заданная позиция
    Serial.print(',');
    Serial.println(realPosition, DEC);  // реальная позиция
    //Serial.print(',');
    static float prevpos;
    //Serial.print((realPosition-prevpos)*1000.0/pidTimer); // скорость
    prevpos = realPosition;
    //Serial.print(',');
  }
  unsigned long endTime = micros();
  unsigned long executionTime = endTime - startTime;
  //Serial.print(executionTime);
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


void setMotorPwm(int16_t pwm) {  // функция подающая ШИМ 0..255 на драйвер двигателей MX1508
  if (pwm >= 0) {
    analogWrite(MX1508_IN1_PIN, LOW);
    analogWrite(MX1508_IN2_PIN, abs(pwm));
  } else {
    analogWrite(MX1508_IN1_PIN, abs(pwm));
    analogWrite(MX1508_IN2_PIN, LOW);
  }
}