#include "ArduinoJson.h"
#define MIN_POS 0     // границы движения сервопривода в градусах
#define MAX_POS 270

#define ENCODER_PIN     A1    // пин потенциометра или энкодера
#define MX1508_IN1_PIN  5     // пины драйвера двигателей
#define MX1508_IN2_PIN  6

#define ENCODER_SCALE   270.f/1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270] 

float alpha = 0.4; // коэффициент фильтрации при измерении положения с потенциометра, для AS5600 можно поставить 1.0

float kP = 8.5;   // пропорциональный коэффициент PID-регулятора 
float kI = 3.0;   // интегральный коэффициент PID-регулятора    
float kD = 1.0;   // дифференциальный коэффициент PID-регулятора 

// json буфер для пакетов от ПК
StaticJsonDocument<200> jsondoc;

// глобальные переменные 
float position = 0;   // задаваемое положение
float realPosition = 0;   // реальное положение сервопривода
int16_t motorPwm = 0; // Заполнение ШИМ, подаваемое на мотор
uint32_t pidTimer = 0;  // таймер обновления pid-регулятора


void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PIN, INPUT); 
  pinMode(MX1508_IN1_PIN, OUTPUT); 
  pinMode(MX1508_IN2_PIN, OUTPUT); 
  
  setMotorPwm(0);
  pidTimer = millis();
}


void loop() {
  // получаем пакеты по uart
  if (Serial.available() > 0){
    DeserializationError err = deserializeJson(jsondoc, Serial);  
    if (err == DeserializationError::Ok){    // если пакет принят
      position = (float)jsondoc["angle"];  // получаем угол поворота
      position = constrain(position, MIN_POS, MAX_POS); // ограничиваем угол поворота сервопривода
    }
    else {while (Serial.available() > 0) Serial.read();}// чистим буфер
  }


  // обновляем PID каждые 10 мс
  if (millis() - pidTimer >= 10){   
    realPosition = getAngle();
    
    float temp_ki = kI;
    if(abs(motorPwm) == 255) temp_ki = 0.f; // если заполнение ШИМ ушло в насыщение, то отключаем интегральную составляющую
    
    motorPwm = (int16_t)pid(realPosition, position, kP, temp_ki, kD, (millis()-pidTimer)/1000.f);  // получаем заполнение ШИМ с PID-регулятора
    
    motorPwm = constrain(motorPwm, -255, 255);  // ограничиваем диапаон ШИМа до допустимого диапазона
    setMotorPwm(motorPwm);  // подаем ШИМ на мотор
    pidTimer = millis();

    Serial.print(position, DEC);        // выводим заданную позицию 
    Serial.print(',');    
    Serial.println(realPosition, DEC);  // и реальную
  }
}


float pid(float input, float trgt, float kp, float ki, float kd, float dt){
  static float integral = 0.f;    // храним значение суммы интегральной компоненты
  static float lastError = 0.f;   // и предыдущую ошибку регулирования для дифференциирования

  if(dt == 0.f){
    integral = 0.f;
    lastError = 0.f; 
    return 0.f;
  }
  
  float error = trgt - input;   // получаем ощибку регулирования
  integral += ki * error * dt;  // пересчитываем интегральную сумму
  float diff = (error - lastError) / dt;  // ищем дифференциал

  lastError = error;
  return kp * error + integral + kd * diff;   // считаем и выводим результат
}


void setMotorPwm(int16_t pwm){  // функция подающая ШИМ на драйвер двигателей MX1508
  if(pwm >= 0){
    analogWrite(MX1508_IN1_PIN, LOW);  
    analogWrite(MX1508_IN2_PIN, abs(pwm));
  } 
  else {
    analogWrite(MX1508_IN1_PIN, abs(pwm));  
    analogWrite(MX1508_IN2_PIN, LOW);
  }
}


float getAngle(){ // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float angle = ENCODER_SCALE * analogRead(ENCODER_PIN);
  angle = lpFilter(angle, oldAngle, alpha);   // фильтруем показания с потенциометра, если надо
  oldAngle = angle;
  return angle;
}


float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}
