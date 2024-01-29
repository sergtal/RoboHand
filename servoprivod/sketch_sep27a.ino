#include <Servo.h> // подключение библиотеки Servo
Servo servo1;
const int pinServo=8; // Пин для подключения сервопривода
const int POT=0; // Аналоговый вход A0 для подключения потенциометра
int valpot = 0; // переменная для хранения значения потенциометра
int angleServo = 0; // переменная для хранения угла поворота сервы
void setup()
{
// подключить переменную servo к выводу pinServo
servo1.attach(pinServo);
pinMode(9,OUTPUT);
}
void loop()
{
valpot = analogRead(POT); // чтение данных потенциометра
// масштабируем значение к интервалу 0-180
angleServo=map(valpot,0,1023,0,360);
// поворот сервопривода на полученный угол
servo1.write(angleServo);
digitalWrite(9, HIGH);
delay(15); // пауза для ожидания поворота сервопривода
}
