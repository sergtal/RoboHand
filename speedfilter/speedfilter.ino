#include <EncButton.h>     // библиотека энкодера гайвера
#include "GyverFilters.h"  // библиотека фильтров гайвера
#define MX1508_IN1_PIN 5   // пины драйвера двигателей
#define MX1508_IN2_PIN 6
// глобальные переменные
int16_t motorPwm = 0;    // заполнение ШИМ, подаваемое на мотор
const int potPin = A0;   // Пин, к которому подключен потенциометр
Encoder e(2, 3);
float target;
float speed = 0;
float pos;
float realPosition = 0;  // реальное положение сервопривода (градусы)
float realSpeed = 0;     // реальная скорость сервопривода (градусы/с)
////////////// speed
volatile unsigned long prevTime = 0;         // Переменная для хранения предыдущего времени прерывания
volatile unsigned long deltaTime = 0;        // Переменная для хранения времени между импульсами
volatile float angularSpeed = 0.0;           // Переменная для хранения вычисленной угловой скорости
volatile float angularSpeed1 = 0.0;          // Переменная для хранения вычисленной угловой скорости
const unsigned long stopThreshold = 150000;  // Порог времени для считывания импульсов (в микросекундах) если енкодер остановлен но есть ложные выпады
const unsigned long checkInterval = 150000;  // Период проверки остановки энкодера (в микросекундах)
//////////////// filters
template<int order>  // order is 1 or 2
class LowPass {
private:
  float a[order];
  float b[order + 1];
  float omega0;
  float dt;
  bool adapt;
  float tn1 = 0;
  float x[order + 1];  // Raw values
  float y[order + 1];  // Filtered values

public:
  LowPass(float f0, float fs, bool adaptive) {
    // f0: cutoff frequency (Hz)
    // fs: sample frequency (Hz)
    // adaptive: boolean flag, if set to 1, the code will automatically set
    // the sample frequency based on the time history.

    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }

  void setCoef() {
    if (adapt) {
      float t = micros() / 1.0e6;
      dt = t - tn1;
      tn1 = t;
    }

    float alpha = omega0 * dt;
    if (order == 1) {
      a[0] = -(alpha - 2.0) / (alpha + 2.0);
      b[0] = alpha / (alpha + 2.0);
      b[1] = alpha / (alpha + 2.0);
    }
    if (order == 2) {
      float alphaSq = alpha * alpha;
      float beta[] = { 1, sqrt(2), 1 };
      float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
      b[0] = alphaSq / D;
      b[1] = 2 * b[0];
      b[2] = b[0];
      a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
      a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
    }
  }

  float filt(float xn) {
    // Provide me with the current raw value: x
    // I will give you the current filtered value: y
    if (adapt) {
      setCoef();  // Update coefficients if necessary
    }
    y[0] = 0;
    x[0] = xn;
    // Compute the filtered values
    for (int k = 0; k < order; k++) {
      y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];

    // Save the historical values
    for (int k = order; k > 0; k--) {
      y[k] = y[k - 1];
      x[k] = x[k - 1];
    }

    // Return the filtered value
    return y[0];
  }
};

// Filter instance
LowPass<1> lp(0.1, 5, false);  //4,30,false 0.1,5


void setup() {
  Serial.begin(115200);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  e.setEncISR(true);
  e.setEncType(EB_STEP1);
  attachInterrupt(0, isr, CHANGE);  // прерывание энкодера RX0
  attachInterrupt(1, handle, CHANGE);  // прерывание энкодера RX1
  /////////////
}


void loop() {
  e.tick();
  // Проверяем, прошло ли достаточно времени с последнего вызова isr
  unsigned long currentTime = micros();
  if (currentTime - prevTime > checkInterval) {
    angularSpeed1 = 0.0;  // Считаем, что энкодер остановлен
  }
  int potValue = analogRead(potPin);            // Считываем значение с потенциометра
  setMotorPwm(map(potValue, 0, 1023, 0, 255));  // Преобразуем значение в диапазон скорости мотора (0-255)
  ///////////
  //Serial.print(angularSpeed);     // Выводим скорость в монитор порта (для отладки)
  //Serial.print(',');
  //Serial.println(angularSpeed1);  // Выводим скорость в монитор порта (для отладки)
  delay(10);  // для стабильности
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

void handle() {
  e.tickISR();                           // в прерывании вызываем тик ISR для шагов
  unsigned long currentTime = micros();  // Измеряем текущее время в микросекундах
  deltaTime = currentTime - prevTime;    // Вычисляем время между импульсами
  //if (deltaTime < stopThreshold) { // Проверяем, прошло ли достаточно времени для считывания импульсов
  angularSpeed = 1000000.0 / deltaTime;
  // } else {
  //  angularSpeed = 0.0;  // Если прошло слишком много времени, считаем, что энкодер остановлен
  //}
  prevTime = currentTime;  // Обновляем предыдущее время
  ////////////
  angularSpeed1 = lp.filt(angularSpeed);
  Serial.print(angularSpeed);  // Выводим скорость в монитор порта (для отладки)
  Serial.print(',');
  Serial.println(angularSpeed1);  // Выводим скорость в монитор порта (для отладки)
}
void isr() {
  e.tickISR();  // в прерывании вызываем тик ISR для шагов
}
