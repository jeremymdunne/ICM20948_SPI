#include <Arduino.h>
#include <SPI.h> 
#include <ICM20948.h> 


ICM20948 imu; 
ICM20948::ICM20948_raw_data imu_data; 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PA0, OUTPUT); 
  digitalWrite(PA0, HIGH); 
  delay(2000);

  SPI.begin(); 
  Serial.println("Attempting connection"); 
  // attempt to connect 

  Serial.println(imu.begin(PA1));
}

void loop() {
  // put your main code here, to run repeatedly:
  long time = micros(); 
  imu.get_data(&imu_data); 
  Serial.println(String(micros()-time)); 
  Serial.println("Gyro: " + String(imu_data.gyro[0]) + "; " + String(imu_data.gyro[1]) + "; " + String(imu_data.gyro[2]));
  Serial.println("Accel: " + String(imu_data.accel[0]) + "; " + String(imu_data.accel[1]) + "; " + String(imu_data.accel[2]));
  Serial.println("Temp: " + String(imu_data.temperature)); 
  Serial.println("Mag: " + String(imu_data.mag[0]) + "; " + String(imu_data.mag[1]) + "; " + String(imu_data.mag[2]));
  delay(500); 
}