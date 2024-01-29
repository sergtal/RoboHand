#define TO_DEG 57.2957f
#define TIME_GYRO 2000
#define FK 0.99 // коэффициент комплементарного фильтра

int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
float ax, ay, az;
float gyro_x_zero, gyro_y_zero, gyro_z_zero;
long int time_timer;  

void getGyroData() {
  if( time_timer < micros() ){
    time_timer = micros() + TIME_GYRO;
    accgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    gy = (gy_raw - gyro_y_zero)/131.0;
    ax = ax_raw/16384.0;
    az = az_raw/16384.0;


    // вычисляем угол наклона по акселерометру
    angle_ay = TO_DEG * atan2(-az,-ax);
    // вычисляем угол наклона по гироскопу
    angleY = angleY + gy * TIME_GYRO/1000000.0;
    // корректируем значение угла с помощью акселерометра
    angleY = angleY*FK + angle_ay*(1-FK);
    
    if ((angleY - ANGLE_ZERO>= ANGLE_CRASH) || (angleY - ANGLE_ZERO <= -ANGLE_CRASH)) {
      isCrash = true; 
      angleIntegral = 0;
    } else isCrash = false;  
  }
}

void calibrate_gyro() {
  int i = 0;
  int NUM_CALIBRATE = 100;
  while (i < NUM_CALIBRATE) {
    if( time_timer < micros() ){
      time_timer = micros() + TIME_GYRO;
      accgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
      gyro_x_zero += gx_raw;
      gyro_y_zero += gy_raw;
      gyro_z_zero += gz_raw;
      i++;
    }
  }
  gyro_x_zero /= NUM_CALIBRATE;
  gyro_y_zero /= NUM_CALIBRATE;
  gyro_z_zero /= NUM_CALIBRATE;
}




    
