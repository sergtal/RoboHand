/*
  Балансирующий робот на двух колесах:
  - измеряет текущий угол отклонения от равновесного положения;
  - работает ПИД-регулятор;
  - пока умеет только балансировать, да и то не на одном месте.
*/

// ------------------------ НАСТРОЙКИ ДВИГАТЕЛЯ ----------------------------
#define MOTOR_MAX 255  // максимальный ШИМ сигнал на двигатель
#define MOTOR_MIN 100   // минимальный ШИМ сигнал на двигатель

// выводы для подключения двигателя
#define IN1 2  // правый двигатель
#define IN2 3  // правый двигатель     
#define IN3 10 // левый двигатель
#define IN4 11 // левый двигатель      
#define BT_TX 9
#define BT_RX 8
// ------------------------ НАСТРОЙКИ ----------------------------
#define TIME_PID 2000     // период регулирования,мкс
#define TIME_START 500     // задержка старта, мс
#define ANGLE_ZERO -19.0    // угол, при котором робот уравновешен
#define ANGLE_CRASH 30    // угол, после которого робот падает
#define ANGLE_BALANCE_MIN 0.0  // зона равновесия, моторы отключены
#define ANGLE_BALANCE_MAX 5    // зона регулирования

#define KDV 30    // коэффициент двигателя
#define ERROR_MIN KDV*ANGLE_BALANCE_MIN   // мин значение ошибки регулятора
#define ERROR_MAX KDV*ANGLE_BALANCE_MAX   // макс значение ошибки регулятора

#define KP 2.5        
#define KI 0.0//2         
#define KD 0.0//1  

// -------------------- БИБЛИОТЕКИ ---------------------
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>

MPU6050 accgyro; 
SoftwareSerial BT(BT_RX, BT_TX); 
long int time_regul;
long int time_start;
int  Speed;              // скорость мотора
boolean isCrash;         // признак падения, когда отключаются моторы
float errorPID;          // ошибка регулятора, рассогласование
float gy;                // угловая скорость по гироскопу
float angle_ay;          //угол по акселерометру
float angleY;            // итоговый угол после фильтрации
float angleIntegral = 0; // интегральная составляющая ПИД

float kp = KP;
float ki = KI;
float kd = KD;

 
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
  
  accgyro.initialize();
  calibrate_gyro();
  isCrash = true;
  time_start = millis();
  while(millis() - time_start < TIME_START) {
      getGyroData();
    }
}

void loop() {
  receiveCommand();
  getGyroData();
  if (isCrash){
    stopMotors(); 
  }else {
    PID_regulator();
  }
  //BT.println(String(100*kp) + ' ' + String(1000*ki) + ' ' + String(1000*kd));
  BT.println(String(angleY - ANGLE_ZERO));
}

       

void PID_regulator() {
  if( time_regul < micros() ){
    time_regul = micros() + TIME_PID;
    angleIntegral +=  (angleY - ANGLE_ZERO);

    errorPID = kp*(angleY - ANGLE_ZERO) + kd*gy + ki*angleIntegral;

    if (errorPID > 0)
      Speed = -map(errorPID*KDV, ERROR_MIN, ERROR_MAX, MOTOR_MIN, MOTOR_MAX);
    else 
      Speed = map(-errorPID*KDV, ERROR_MIN, ERROR_MAX, MOTOR_MIN, MOTOR_MAX);
      
  if ((errorPID >= - ANGLE_BALANCE_MIN) && (errorPID <= ANGLE_BALANCE_MIN))
      Speed = 0;
      
    setSpeedMotors(Speed);    
  }
}
