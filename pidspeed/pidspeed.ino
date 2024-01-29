//maincode
#include "GyverPID.h"
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define IN 5
#define PWM 6

volatile int counter = 0;   // счётчик
volatile bool encFlag = 0;  // флаг поворота

GyverPID regulator(20, 0, 0); 

void setup() {
  Serial.begin(9600);
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 1; 

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN,OUTPUT);
  attachInterrupt(0, encIsr, CHANGE);
  attachInterrupt(1, encIsr, CHANGE);
}

void loop() {
  //encoder
  if (encFlag) {  
    // Serial.println(counter); если значение  енкодера поменялось прерывание
    encFlag = 0;
  }               
  regulator.input = counter; 
  regulator.getResult();
  
  float pwr;
  pwr = regulator.output;

  // motor direction
  int dir = 1;
  if(pwr<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.println(counter);
}

void setMotor(int dir, int pwmVal, int pwm, int in){
  float speed = map(pwmVal, 0, 255, 83, 255);
  
  //Serial.print(" ");
  //Serial.print(speed);

  if(dir == 1 & pwmVal>0){
    analogWrite(pwm,speed);
    analogWrite(in,LOW);
  }
  else if(dir == -1 & pwmVal>0){
    analogWrite(pwm,LOW);
    analogWrite(in,speed);
  }
  else{
    analogWrite(pwm,HIGH);
    analogWrite(in,HIGH);
  }
}

volatile byte reset = 0, last = 0;
void encIsr() {  
  byte state = (PIND & 0b1100) >> 2;  // D2 + D3
  if (reset && state == 0b11) {
    int prevCount = counter;
    if (last == 0b10) counter++;
    else if (last == 0b01) counter--;
    if (prevCount != counter) encFlag = 1;    
    reset = 0;
  }
  if (!state) reset = 1;
  last = state;
}