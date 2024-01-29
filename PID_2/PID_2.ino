#include <EncButton.h>  // библиотека энкодера гайвера
#include <GyverINA.h>   // библиотека апмерметра гайвера

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
LowPass<1> lp(0.1, 5, false);  //4,30,false

#define MIN_POS -10000  // границы движения сервопривода в градусах
#define MAX_POS 10000
#define MIN_MAX_SPEED 300    // ограничение скорости 30 об/мин, тут же переводим в 180 град/сек   //
#define MIN_MAX_CURRENT 0.6  // в амперах

#define MX1508_IN1_PIN 5  // пины драйвера двигателей
#define MX1508_IN2_PIN 6

#define ENCODER_SCALE 270.f / 1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270]

// просто энкодер
Encoder e(2, 3);

float palpha = 0.2;  // коэффициент фильтрации при измерении положения с потенциометра, для AS5600 можно поставить 1.0
float ialpha = 0.1;  // коэффициент фильтрации при измерении тока

float pkP = 0.1;  // пропорциональный коэффициент PID-регулятора по положению 0.1
float pkI = 0.0;  // интегральный коэффициент PID-регулятора по положению
float pkD = 0.0;  // дифференциальный коэффициент PID-регулятора по положению

float vkP = 0.1;  // пропорциональный коэффициент PID-регулятора по скорости 0.01
float vkI = 0.0;  // интегральный коэффициент PID-регулятора по скорости
float vkD = 0.0;  // дифференциальный коэффициент PID-регулятора по скорости

float ikP = 457.0;  // пропорциональный коэффициент PID-регулятора по току 955
float ikI = 0.0;    // интегральный коэффициент PID-регулятора по току
float ikD = 0.0;    // дифференциальный коэффициент PID-регулятора по току

// глобальные переменные
float position = 7;     // задаваемое положение (градусы)
float speed = 0;        // задаваемая PIDом положения скорость сервопривода (градусы/с)
float current = 0;      // задаваемый PIDом скорости ток сервопривода (амперы)
int16_t motorPwm = 0;   // заполнение ШИМ, подаваемое на мотор
int16_t motorPwm1 = 0;  // заполнение ШИМ, подаваемое на мотор

float realPosition = 0;  // реальное положение сервопривода (градусы)
float realSpeed = 0;     // реальная скорость сервопривода (градусы/с)
float realCurrent = 0;   // реальный ток сервопривода (амперы)

//float smoothingFactor = 0.9;  //0.3
//float previousSpeed = 0;
const int potPin = A0;  // Пин, к которому подключен потенциометр

uint32_t pidTimer = 0;  // таймер обновления pid-регулятора

INA219 ina;  // Стандартный набор параметров для Arduino модуля (0.1, 3.2, 0x40)
float b = 0;
float a = 0;
float c = 0;
float d = 0;
//скорость
volatile unsigned long prevTime = 0;         // Переменная для хранения предыдущего времени прерывания
volatile unsigned long deltaTime = 0;        // Переменная для хранения времени между импульсами
volatile float angularSpeed = 0.0;           // Переменная для хранения вычисленной угловой скорости
volatile float angularSpeed1 = 0.0;          // Переменная для хранения вычисленной угловой скорости
const unsigned long stopThreshold = 150000;  // Порог времени для считывания импульсов (в микросекундах) если енкодер остановлен но есть ложные выпады
const unsigned long checkInterval = 150000;  // Период проверки остановки энкодера (в микросекундах)

bool firstRun = true;  //пинок для плавного старта мотора


void setup() {
  Serial.begin(500000);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);

  e.setEncISR(true);
  e.setEncType(EB_STEP1);

  if (ina.begin()) {  // Проверяем наличие и инициализируем INA219
    Serial.println(F("connected!"));
  } else {
    Serial.println(F("not found!"));
  }
  ina.setResolution(INA219_VBUS, INA219_RES_12BIT_X4);      // Напряжение в 12ти битном режиме + 4х кратное усреднение
  ina.setResolution(INA219_VSHUNT, INA219_RES_12BIT_X128);  // Ток в 12ти битном режиме + 128х кратное усреднение

  setMotorPwm(0);
  pidTimer = millis();

  attachInterrupt(0, isr, CHANGE);     // прерывание энкодера RX0
  attachInterrupt(1, handle, CHANGE);  // прерывание энкодера RX1
}


void loop() {
  e.tick();
  // отработка
  position = constrain(position, MIN_POS, MAX_POS);  // ограничиваем угол поворота сервопривода

  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 5) {
    // пересчитываем все параметры
    float dt = (millis() - pidTimer) / 1000.f;  // пройденное с прошлого раза время
    float newPosition = e.counter;              // получаем новое положение
    unsigned long currentTime = micros();
    if (currentTime - prevTime > checkInterval) {
      angularSpeed1 = 0.0;  // Считаем, что энкодер остановлен
      firstRun = false;
    } else {
      firstRun = false;
    }
    realSpeed = angularSpeed1 * 2 / 50;  // получаем скорость путем дифференцииорования положения по времени
    //Serial.print(angularSpeed / 2 / 50, DEC);  // реальную скорость
    //Serial.print(',');

    realPosition = newPosition;
    realCurrent = getCurrent();

    float temp_ki = pkI;
    if (abs(speed) == MIN_MAX_SPEED) temp_ki = 0.f;               // если скорость ушла в насыщение, то отключаем интегральную составляющую
    speed = ppid(realPosition, position, pkP, temp_ki, pkD, dt);  // получаем скорость с PID-регулятора
    speed = constrain(speed, -MIN_MAX_SPEED, MIN_MAX_SPEED);      // ограничиваем скорость до допустимого диапазона

    temp_ki = vkI;
    if (abs(current) == MIN_MAX_CURRENT) temp_ki = 0.f;               // если ток ушел в насыщение, то отключаем интегральную составляющую
    current = vpid(realSpeed, speed, vkP, temp_ki, vkD, dt);          // получаем ток с PID-регулятора
    current = constrain(current, -MIN_MAX_CURRENT, MIN_MAX_CURRENT);  // ограничиваем ток до допустимого диапазона

    temp_ki = ikI;
    if (abs(motorPwm) == 255) temp_ki = 0.f;                                // если заполнение ШИМ ушло в насыщение, то отключаем интегральную составляющую
    motorPwm = (int16_t)ipid(realCurrent, current, ikP, temp_ki, ikD, dt);  // получаем заполнение ШИМ с PID-регулятора
    motorPwm1 = constrain(motorPwm, -255, 255);                             // ограничиваем заполнение до допустимого диапазона
    if (motorPwm1< 80 & motorPwm1 > 0 & motorPwm1 != 0) {
      motorPwm = 80;
    } else if (motorPwm1 > -80 & motorPwm1 < 0 & motorPwm1 != 0) {
      if (firstRun) {
        analogWrite(MX1508_IN1_PIN, 255);
        analogWrite(MX1508_IN2_PIN, LOW);
      }
      motorPwm = -80;
    }
    //int potValue = analogRead(potPin);                                      // Считываем значение с потенциометра
    //setMotorPwm(map(potValue, 0, 1023, 0, 255));                            // Преобразуем значение в диапазон скорости мотора (0-255)

    setMotorPwm(motorPwm);  // подаем ШИМ на мотор
    pidTimer = millis();

    Serial.print(current, DEC);  // заданный ток
    Serial.print(',');
    Serial.print(motorPwm, DEC);
    Serial.print(',');
    Serial.print(realCurrent, DEC);  // реальный ток
    Serial.print(',');
    Serial.print(realSpeed, DEC);  // реальную скорость
    Serial.print(',');
    Serial.print(speed, DEC);  // заданную скорость
    Serial.print(',');
    Serial.println(realPosition);  // реальную позицию
    //Serial.print(',');
    //Serial.print(position, DEC);  // заданное положение
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
  //Serial.print(angularSpeed);  // Выводим скорость в монитор порта (для отладки)
  //Serial.print(',');
  //Serial.println(angularSpeed1);  // Выводим скорость в монитор порта (для отладки)
}


float pid(float input, float trgt, float kp, float ki, float kd, float dt, float* integral, float* lasterror) {
  if (dt == 0.f) {
    *integral = 0.f;
    *lasterror = 0.f;
    return 0.f;
  }

  float error = trgt - input;              // получаем ощибку регулирования
  *integral += ki * error * dt;            // пересчитываем интегральную сумму
  float diff = (error - *lasterror) / dt;  // ищем дифференциал

  *lasterror = error;
  return kp * error + *integral + kd * diff;  // считаем и выводим результат
}


float ppid(float input, float trgt, float kp, float ki, float kd, float dt) {
  static float pintegral = 0.f;   // храним значение суммы интегральной компоненты
  static float plastError = 0.f;  // и предыдущую ошибку регулирования для дифференциирования
  return pid(input, trgt, kp, ki, kd, dt, &pintegral, &plastError);
}


float vpid(float input, float trgt, float kp, float ki, float kd, float dt) {
  static float vintegral = 0.f;   // храним значение суммы интегральной компоненты
  static float vlastError = 0.f;  // и предыдущую ошибку регулирования для дифференциирования
  return pid(input, trgt, kp, ki, kd, dt, &vintegral, &vlastError);
}


float ipid(float input, float trgt, float kp, float ki, float kd, float dt) {
  static float iintegral = 0.f;   // храним значение суммы интегральной компоненты
  static float ilastError = 0.f;  // и предыдущую ошибку регулирования для дифференциирования
  return pid(input, trgt, kp, ki, kd, dt, &iintegral, &ilastError);
}


void setMotorPwm(int16_t pwm) {  // функция подающая ШИМ на драйвер двигателей MX1508
  if (pwm >= 0) {
    analogWrite(MX1508_IN1_PIN, LOW);
    analogWrite(MX1508_IN2_PIN, abs(pwm));
  } else if (pwm < 0) {
    if (firstRun) {
        analogWrite(MX1508_IN2_PIN, 255);
        analogWrite(MX1508_IN1_PIN, LOW);
      }
    analogWrite(MX1508_IN1_PIN, abs(pwm));
    analogWrite(MX1508_IN2_PIN, LOW);
  }
}

float getAngle() {  // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float angle = e.counter;
  //angle = lpFilter(angle, oldAngle, palpha);   // фильтруем показания с потенциометра, если надо
  oldAngle = angle;
  return angle;
}


float lpFilter(float value, float oldValue, float alp) {
  return oldValue * (1.f - alp) + alp * value;
}


float getCurrent() {
  static float oldCurr = 0;
  float curr = ina.getCurrent();
  //curr = lpFilter(curr, oldCurr, ialpha);  // фильтруем показания с потенциометра, если надо
  oldCurr = curr;
  return curr;
}


void isr() {
  e.tickISR();  // в прерывании вызываем тик ISR
}
