volatile int counter = 0;   // счётчик
volatile bool encFlag = 0;  // флаг поворота

void setup() {
  Serial.begin(9600);
  attachInterrupt(0, encIsr, CHANGE);
  attachInterrupt(1, encIsr, CHANGE);
}

void loop() {
  if (encFlag) {
    Serial.println(counter);
    encFlag = 0;
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