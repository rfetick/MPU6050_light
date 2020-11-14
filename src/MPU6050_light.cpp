/* MPU6050_light library for Arduino
 * 
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light.h"
#include "Arduino.h"

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

byte MPU6050::begin(){
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  writeData(MPU6050_GYRO_CONFIG_REGISTER, MPU6050_GYRO_CONFIG);
  writeData(MPU6050_ACCEL_CONFIG_REGISTER, MPU6050_ACCEL_CONFIG);
  byte status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the last connection with status
  
  this->update();
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis(); // may cause issue if begin() is much before the first update()
  return status;
}

byte MPU6050::writeData(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  byte status = wire->endTransmission();
  return status;
}

// This method is not used internaly, maybe by user...
byte MPU6050::readData(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data =  wire->read();
  return data;
}

/* SETTER */

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) or (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff){
  setFilterGyroCoef(1.0-acc_coeff);
}

/* CALC OFFSET */

void MPU6050::calcGyroOffsets(){
  float xyz[3] = {0,0,0};
  int16_t b;
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(MPU6050_GYRO_OUT_REGISTER);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

	for(int j=0;j<3;j++){
		b  = wire->read() << 8;
		b |= wire->read();
		xyz[j] += ((float)b) / GYRO_LSB_2_DEGSEC;
	}
	
	delay(1);
  }
  gyroXoffset = xyz[0] / CALIB_OFFSET_NB_MES;
  gyroYoffset = xyz[1] / CALIB_OFFSET_NB_MES;
  gyroZoffset = xyz[2] / CALIB_OFFSET_NB_MES;
}

void MPU6050::calcAccOffsets(){
  float xyz[3] = {0,0,0};
  int16_t b;
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(MPU6050_ACCEL_OUT_REGISTER);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

	for(int j=0;j<3;j++){
		b  = wire->read() << 8;
		b |= wire->read();
		xyz[j] += ((float)b) / ACC_LSB_2_G;
	}
	
	delay(1);
  }
  
  accXoffset = xyz[0] / CALIB_OFFSET_NB_MES;
  accYoffset = xyz[1] / CALIB_OFFSET_NB_MES;
  accZoffset = xyz[2] / CALIB_OFFSET_NB_MES - 1.0;
}

/* UPDATE */

void MPU6050::update(){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(MPU6050_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for(int i=0;i<7;i++){
	rawData[i]  = wire->read() << 8;
    rawData[i] |= wire->read();
  }

  accX = ((float)rawData[0]) / ACC_LSB_2_G - accXoffset;
  accY = ((float)rawData[1]) / ACC_LSB_2_G - accYoffset;
  accZ = ((float)rawData[2]) / ACC_LSB_2_G - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / GYRO_LSB_2_DEGSEC - gyroXoffset;
  gyroY = ((float)rawData[5]) / GYRO_LSB_2_DEGSEC - gyroYoffset;
  gyroZ = ((float)rawData[6]) / GYRO_LSB_2_DEGSEC - gyroZoffset;
  
  float sgZ = (accZ>=0)-(accZ<0);
  angleAccX = atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG;
  angleAccY = - atan2(accX, sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  angleX = (filterGyroCoef*(angleX + gyroX*dt)) + ((1.0-filterGyroCoef)*angleAccX);
  angleY = (filterGyroCoef*(angleY + gyroY*dt)) + ((1.0-filterGyroCoef)*angleAccY);
  angleZ += gyroZ*dt;

}
