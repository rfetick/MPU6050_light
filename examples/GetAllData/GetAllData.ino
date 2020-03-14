/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

  if(millis() - timer > 1000){
    
    Serial.print(F("TEMPERATURE  : "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO   X : "));Serial.print(mpu.getAccX());
    Serial.print("\tY : ");Serial.print(mpu.getAccY());
    Serial.print("\tZ : ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO       X : "));Serial.print(mpu.getGyroX());
    Serial.print("\tY : ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ : ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE  X : "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY : ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE      X : "));Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=======================================================\n"));
    timer = millis();
    
  }

}
