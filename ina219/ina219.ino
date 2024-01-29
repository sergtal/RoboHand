#include <Wire.h>
#include <Adafruit_INA219.h>

// Инициализация INA219
Adafruit_INA219 ina219;
#define MX1508_IN1_PIN 5  // пины драйвера двигателей
#define MX1508_IN2_PIN 6
// глобальные переменные
int16_t motorPwm = 0;   // заполнение ШИМ, подаваемое на мотор
const int potPin = A0;  // Пин, к которому подключен потенциометр

// Фильтр низких частот
template<int order>
class LowPass {
private:
  float a[order];
  float b[order + 1];
  float omega0;
  float dt;
  float tn1 = 0;
  float x[order + 1];  // Сырые значения
  float y[order + 1];  // Сглаженные значения

public:
  LowPass(float f0, float fs, bool adaptive) {
    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }

  void setCoef() {

    float t = micros() / 1.0e6;
    dt = t - tn1;
    tn1 = t;


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

    setCoef();  // Обновить коэффициенты при необходимости

    y[0] = 0;
    x[0] = xn;
    for (int k = 0; k < order; k++) {
      y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];

    for (int k = order; k > 0; k--) {
      y[k] = y[k - 1];
      x[k] = x[k - 1];
    }

    return y[0];
  }
};

LowPass<2> lp(5, 1000, true);  // Фильтр низких частот: f0 = 5 Гц, fs = 1000 Гц

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  // Инициализация INA219
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop(void) {
  // Чтение напряжения, тока и мощности с INA219
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float busVoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();

  // Применение фильтрации низких частот к току (фильтр нижних частот)
  float filteredCurrent = current_mA;
  int potValue = analogRead(potPin);            // Считываем значение с потенциометра
  setMotorPwm(map(potValue, 0, 1023, 0, 255));  // Преобразуем значение в диапазон скорости мотора (0-255)
  // Вывод результатов

  Serial.print(busVoltage);
  Serial.print(",");

  //Serial.print(shuntVoltage);
  //Serial.print(",");

  Serial.println(filteredCurrent);  // Используется сглаженный ток
  //Serial.print(",");

  //Serial.print(power_mW);


  delay(100);  // Задержка для удобства чтения данных
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