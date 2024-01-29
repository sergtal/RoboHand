// главный код программы


#define IN 4
#define PWM 5
#define ENC_A 2       // пин энкодера
#define ENC_B 3       // пин энкодера
#define ENC_TYPE 1    // тип энкодера, 0 или 1

volatile boolean state0, lastState, turnFlag;

volatile int pos;

long prevT = 0;
float eprev = 0;
float eintegral = 0;


void setup() {
  Serial.begin(9600);     // запускаем монитор порта motor A  and IN1 IN2
  pinMode(A0, INPUT);     //считываем с А0 для 1 мотора
  pinMode(IN, OUTPUT); //входы мотора
  pinMode(PWM, OUTPUT);
  attachInterrupt(0, speedEncoder, CHANGE); //вызов енкодера
}

void loop() {

  // set target position                           цель 
  int target = 150*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0;
  float ki = 0;

  // time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  // error
  int e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = abs(u);
  if(pwr>255){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(u);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
}



void speedEncoder() {
  state0 = digitalRead(ENC_A);
  if (state0 != lastState) {
  turnFlag = !turnFlag;
  if (turnFlag)
    pos += (digitalRead(ENC_B) != lastState) ? -1 : 1;
  lastState = state0;
}
}
void setMotor(int dir, int pwr, int pinPWM, int pinIN){
  float speed = map(pwr, -255, 255, 129, 255); 
  if(dir == 1){
    analogWrite(pinPWM,speed);
    analogWrite(pinIN,LOW);
  }
  else if(dir == -1){
    analogWrite(pinPWM,LOW);
    analogWrite(pinIN,speed);
  }
  else{
    analogWrite(pinPWM,LOW);
    analogWrite(pinIN,LOW);
  }
}