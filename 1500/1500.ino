#define MOTOR2_IN 4
#define MOTOR2_PWM 5
#define buttonPin 6    // the number of the pushbutton pin
#define ledPin 13      // the number of the LED pin
#define ENC_A 2       // пин энкодера
#define ENC_B 3       // пин энкодера
#define ENC_TYPE 1    // тип энкодера, 0 или 1

int buttonState = 0;         // variable for reading the pushbutton status
int lastButtonState = 0;  
volatile int encCounter;
volatile boolean state0, lastState, turnFlag;

void setup() {
  Serial.begin(9600);     // запускаем монитор порта motor A  and IN1 IN2
  pinMode(A0, INPUT);     //считываем с А0 для 1 мотора
   // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(MOTOR2_IN, OUTPUT); //входы мотора
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(ledPin, OUTPUT); //контрольный свет
  attachInterrupt(0, speedEncoder, CHANGE);
}

void loop() {
  Serial.println(encCounter);
  buttonControl();
  motorControl(analogRead(0), MOTOR2_IN, MOTOR2_PWM);
  delay(50);
}

void speedEncoder() {
  state0 = digitalRead(ENC_A);
  if (state0 != lastState) {
  turnFlag = !turnFlag;
  if (turnFlag)
    encCounter += (digitalRead(ENC_B) != lastState) ? -1 : 1;
  lastState = state0;
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
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  
}
void buttonControl(){
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
   if (buttonState) {
          digitalWrite(ledPin, HIGH);  
          analogWrite(MOTOR2_PWM, 220);
          digitalWrite(MOTOR2_IN, 0);      
          delay(800);
          digitalWrite(MOTOR2_IN, 0);
          digitalWrite(MOTOR2_PWM, 0);
          delay(200);
          analogWrite(MOTOR2_PWM, -120);
          digitalWrite(MOTOR2_IN, 1);
          delay(800);
        } 
        else {
          digitalWrite(ledPin, LOW);
        }
    lastButtonState = buttonState;
  }
}
