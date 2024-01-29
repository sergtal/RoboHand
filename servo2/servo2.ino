const int motorPin = 5; // Пин, к которому подключен мотор
const int potPin = A0; // Пин, к которому подключен потенциометр
#include <EncButton.h>
EncButton<EB_TICK, 2, 3> enc;  // просто энкодер


void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  enc.setEncType(EB_HALFSTEP);
  pinMode(2, INPUT);  //D2 и D3 подтяжка пинов энкодера
  pinMode(3, INPUT);
  attachInterrupt(0, isr, CHANGE);  // D2
  attachInterrupt(1, isr, CHANGE);  // D3
}


void isr() {
  enc.tickISR();  // в прерывании вызываем тик ISR
}


void loop() {
  enc.tick();  // отработка
  // получаем пакеты по uart
  int potValue = analogRead(potPin); // Считываем значение с потенциометра
  int motorSpeed = map(potValue, 0, 1023, 0, 255); // Преобразуем значение в диапазон скорости мотора (0-255)

  analogWrite(motorPin, motorSpeed); // Устанавливаем скорость мотора

  //Serial.print(motorSpeed);
  //Serial.print(',');
  Serial.println(enc.counter, DEC);  // выводим заданную позицию
  
  delay(5); // Можно изменить задержку, чтобы управлять чувствительностью регулировки
}
