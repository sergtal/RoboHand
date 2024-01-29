//maincode
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define IN 5
#define PWM 6

volatile int counter = 0;   // счётчик
volatile bool encFlag = 0;  // флаг поворота

long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
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
  // set target position
  int target = 1;
  //int target = 50*sin(prevT/1e6);

  // PID constants
  float kp = 40; //20
  float kd = 0.25;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  // error
  int e = counter - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u)*4;
  if( pwr > 1024 ){
    pwr = 1024;
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

  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.println(counter);
}

void setMotor(int dir, int pwmVal, int pwm, int in){
  float speed = map(pwmVal, 0, 1024, 83, 255);
  
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