void stopMotors() {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  Speed = 0;
}


void setSpeedMotors(int _speed){
  if (abs(_speed)<10) {
      stopMotors();
  } else {
      setSpeedMotorRight(_speed);
      setSpeedMotorLeft(_speed);
  }

}
void setSpeedMotorRight(int _speed) {
  int speedNorm = constrain(abs(_speed), 0, MOTOR_MAX);
  if (_speed > 0){
    digitalWrite(IN4,0);
    analogWrite(IN3, speedNorm);
  }  else {
      digitalWrite(IN4,1);
      analogWrite(IN3, 255 - speedNorm);
  }
}

void setSpeedMotorLeft(int _speed) {
  int speedNorm = constrain(abs(_speed), 0, MOTOR_MAX);
    if (_speed > 0){
    digitalWrite(IN2,0);
    analogWrite(IN1, speedNorm);
  }  else {
      digitalWrite(IN2,1);
      analogWrite(IN1, 255 - speedNorm);
  }
}
