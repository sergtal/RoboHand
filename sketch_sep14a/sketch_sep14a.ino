#define IN 4
#define PWM 5
#define buttonPin 6    // the number of the pushbutton pin
#define ledPin 13      // the number of the LED pin
#define ENC_A 2       // пин энкодера
#define ENC_B 3       // пин энкодера
#define ENC_TYPE 1    // тип энкодера, 0 или 1

int buttonState = 0;         // variable for reading the pushbutton status
int lastButtonState = 0;  
volatile boolean state0, lastState, turnFlag;

volatile int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;


void setup() {
  Serial.begin(9600);     // запускаем монитор порта motor A  and IN1 IN2
  pinMode(A0, INPUT);     //считываем с А0 для 1 мотора
   // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  pinMode(IN, OUTPUT); //входы мотора
  pinMode(PWM, OUTPUT);
  pinMode(ledPin, OUTPUT); //контрольный свет
  attachInterrupt(0, speedEncoder, CHANGE); //вызов енкодера
}

void loop() {
  //Serial.println(pos);
  //buttonControl();
  //motorControl(analogRead(0),IN,PWM);
  //delay(50);

  // set target position
  int target = 120;

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
  float pwr = fabs(u);
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
void setMotor(int dir, int val, int pinPWM, int pinIN){
  analogWrite(pinPWM,val);
  if (dir == 1) {  
    digitalWrite(pinPWM,HIGH);
    digitalWrite(pinIN,LOW);
  } else if (dir == -1) { 
    digitalWrite(pinPWM,LOW);
    digitalWrite(pinIN,HIGH);
  } else {  
    digitalWrite(pinIN,LOW);
    digitalWrite(pinPWM,LOW);
  }
}
// принимает знач. 0-1023, пин IN и PWM
void motorControl(int val, byte pinIN, byte pinPWM) { //800 200 800 finger
  val = map(val, 0, 1023, -255, 255);

  if (val > 0) {  // вперёд
    analogWrite(pinPWM, val);
    digitalWrite(pinIN, 0);
  } else if (val < 0) {  // назад
    analogWrite(pinPWM, 255 + val);
    digitalWrite(pinIN, 1);
  } else {  // стоп
    digitalWrite(pinIN, 0);
    digitalWrite(pinPWM, 0);
  }

  Serial.println(val);
}
void buttonControl(){
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
   if (buttonState) {
          digitalWrite(ledPin, HIGH);  
          analogWrite(PWM, 220);
          digitalWrite(IN, 0);      
          delay(800);
          digitalWrite(IN, 0);
          digitalWrite(PWM, 0);
          delay(200);
          analogWrite(PWM, -120);
          digitalWrite(IN, 1);
          delay(800);
        } 
        else {
          digitalWrite(ledPin, LOW);
        }
    lastButtonState = buttonState;
  }
}
