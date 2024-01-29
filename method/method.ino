#include <PID_v1.h>

// Подключаем библиотеку PID
// https://github.com/br3ttb/Arduino-PID-Library

double Kp = 1.0; // Коэффициент пропорциональности
double Ki = 0.5; // Коэффциент интегральной составляющей
double Kd = 0.1; // Коэффициент дифенциальной составляющей

double setpoint = 50.0; // Задное значение
double input = 0.0; // Текущее значение
double output = 0.0; // Выходное значение

double error = 0.0; // Ошибка
double lastError = 0.0; // Предыдущая ошибка
double integral =0.0; // Интегральная сумма

unsigned long lastTime = 0; // Время последнего обновления
unsigned long sampleTime = 100; // Время между обновлениями

int motorPin1 = 4; // Пин 1 ШИМ-контроллера
int motorPin2 = 5; // Пин 2 ШИМ-контроллера
int encoderPinA = 2; // Пин A инкрементального энкодера
int encoderPinB = 3; // Пин B инкрементального энкодера

volatile int encoderPos = 0; // Текущее значение энкодера
volatile int lastEncoderPos = 0; // Предыдущее значение энкодера

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Создаем объект PID

void setup() {
  pinMode(motorPin1, OUTPUT); // Настраиваем пин 1 ШИМ-контроллера как выход
  pinMode(motorPin2, OUTPUT); // Настраиваем пин 2 ШИМ-контроллера как выход
  pinMode(encoderPinA, INPUT); // Настраиваем пин A энкодера как вход с подтякой
  pinMode(encoderPinB, INPUT); // Настраиваем пин B энкодера как вход с подтяжкой
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE); // Настраиваем прерывание на изменение пина A энкодера
  myPID.SetMode(AUTOMATIC); // Устанавливаем режим автоматического управления
  Serial.begin(9600); // Настраиваем монитор порта
}

void loop() {
  unsigned long now = millis(); // Текущее время
  unsigned long timeChange = (now - lastTime); // Время с момента последнего обновления

  if (timeChange >= sampleTime) {
    // Считываем текущее значение
    input = encoderPos;

    // Вычисляем ошибку
    error = setpoint - input;

    // Вычисляем интегральную сумму
    integral += (error * timeChange);

    // Вычисляем производную ошибки
    double dError = (error - lastError) / timeChange;

    // Вычисляем выходное значение
    myPID.Compute();

    // Ограничиваем выходное значение
    if (output > 255) {
      output = 255;
    } else if (output < 0) {
      output = 0;
    }

    // Устанавливаем скорость мотора
    if (output >= 0) {
      analogWrite(motorPin1, output);
      analogWrite(motorPin2, 0);
    } else {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, -output);
    }

    // Сохраняем текущую ошибку
    lastError = error;

    // Сохраняем время последнего обновления
    lastTime = now;

    // Выводим данные в монитор порта
    Serial.print("Input: ");
    Serial.print(input);
    Serial.print(" Output: ");
    Serial.print(output);
    Serial.print(" Setpoint: ");
    Serial.println(setpoint);
  }
}

void updateEncoder() {
  int valueA = digitalRead(encoderPinA);
  int valueB = digitalRead(encoderPinB);

  if (valueA == HIGH && valueB == LOW) {
    encoderPos++;
  } else if (valueA == LOW && valueB == HIGH) {
    encoderPos--;
  }

  if (encoderPos != lastEncoderPos) {
    lastEncoderPos = encoderPos;
  }
}
