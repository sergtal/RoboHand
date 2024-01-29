#include <EncButton.h>  // библиотека энкодера гайвера
#include <GyverINA.h>   // библиотека апмерметра гайвера

#define MIN_POS 0  // границы движения сервопривода в градусах
#define MAX_POS 270
#define MIN_MAX_SPEED 360  // ограничение скорости 30 об/мин, тут же переводим в 180 град/сек   //
#define MIN_MAX_CURRENT 0.6          // в амперах
#define MIN_MAX_VOLTAGE 8          // в вольтах

#define MX1508_IN1_PIN 5  // пины драйвера двигателей
#define MX1508_IN2_PIN 6

#define ENCODER_SCALE 270.f / 1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270]
#define CURRENT_SCALE 0.185           // шкала датчика тока: 0.185В на выходе на 1А


float palpha = 0.2;  // коэффициент фильтрации при измерении положения с потенциометра, для AS5600 можно поставить 1.0
float ialpha = 0.2;  // коэффициент фильтрации при измерении тока

float pkP = 1.0;  // пропорциональный коэффициент PID-регулятора по положинию
float pkI = 0.0;  // интегральный коэффициент PID-регулятора по положинию
float pkD = 0.0;  // дифференциальный коэффициент PID-регулятора по положинию

float vkP = 10.0;  // пропорциональный коэффициент PID-регулятора по скорости
float vkI = 0.0;  // интегральный коэффициент PID-регулятора по скорости
float vkD = 0.0;  // дифференциальный коэффициент PID-регулятора по скорости

float ikP = 200.0;  // пропорциональный коэффициент PID-регулятора по току
float ikI = 0.0;    // интегральный коэффициент PID-регулятора по току
float ikD = 0.0;    // дифференциальный коэффициент PID-регулятора по току

// просто энкодер
Encoder e(2, 3);

// глобальные переменные
float position = 200;  // задаваемое положение (градусы)
float speed = 0;       // задаваемая PIDом положения скорость сервопривода (градусы/с)
float current = 0;     // задаваемый PIDом скорости ток сервопривода (амперы)
int16_t motorPwm = 0;  // заполнение ШИМ, подаваемое на мотор

float realPosition = 0;  // реальное положение сервопривода (градусы)
float realSpeed = 0;     // реальная скорость сервопривода (градусы/с)
float realCurrent = 0;  // реальный ток сервопривода (амперы) сглаженный тое
float realVoltage = 0;  // реальный ток сервопривода (амперы)

uint32_t pidTimer = 0;  // таймер обновления pid-регулятора

INA219 ina;  // Стандартный набор параметров для Arduino модуля (0.1, 3.2, 0x40)
float b = 0;
float a = 0;
float c = 0;
float d = 0;

void setup() {
  Serial.begin(115200);
  pinMode(MX1508_IN1_PIN, OUTPUT);
  pinMode(MX1508_IN2_PIN, OUTPUT);
  
  if (ina.begin()) {  // Проверяем наличие и инициализируем INA219
    Serial.println(F("connected!"));
  } else {
    Serial.println(F("not found!"));
  }
  ina.setResolution(INA219_VSHUNT, INA219_RES_12BIT_X2);  // Ток в 12ти битном режиме + 128х кратное усреднение
  //ina.setCalibration(4000); 
  e.setEncISR(true);
  e.setEncType(EB_STEP1);
  setMotorPwm(0);
  pidTimer = millis();

  attachInterrupt(0, isr, CHANGE);  // прерывание энкодера RX0
  attachInterrupt(1, isr, CHANGE);  // прерывание энкодера RX1
}


void loop() {
  e.tick();
  // отработка
  position = constrain(position, MIN_POS, MAX_POS);  // ограничиваем угол поворота сервопривода

  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 10) {
    // пересчитываем все параметры
    float dt = (millis() - pidTimer) / 1000.f;      // пройденное с прошлого раза время
    float newPosition = e.counter;                 // получаем новое положение
    if (e.turn()) {                 // любой поворот
      realSpeed = (newPosition - realPosition) / dt;  // получаем скорость путем дифференцииорования положения по времени
    }
    realPosition = newPosition;
    realCurrent = ina.getCurrent();
    realVoltage = ina.getVoltage();

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
    motorPwm = constrain(motorPwm, -255, 255);                              // ограничиваем заполнение до допустимого диапазона
    if(realCurrent > MIN_MAX_CURRENT) {
      motorPwm = 0;
    }
    setMotorPwm(100);  // подаем ШИМ на мотор
    pidTimer = millis();

    Serial.print(motorPwm);
    Serial.print(',');
    Serial.print(realVoltage, DEC);  // реальный ток
    Serial.print(',');
    Serial.println(realCurrent, DEC);  // реальный ток
    //Serial.print(',');
    //Serial.print(realSpeed, DEC);  // заданную скорость
    //Serial.print(',');
    //Serial.print(realPosition, DEC);  // реальную позицию
    //Serial.print(',');
    //Serial.print(position, DEC);  // заданное положение
  }
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
  } else {
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

void isr() {
  e.tickISR();
}