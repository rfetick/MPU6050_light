/* The register map is provided at
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * Mapping of the different gyro and accelero configurations:
 *
 * GYRO_CONFIG_[0,1,2,3] range = +- [250, 500,1000,2000] °/s
 *                       sensi =    [131,65.5,32.8,16.4] bit/(°/s)
 *
 * ACC_CONFIG_[0,1,2,3] range = +- [    2,   4,   8,  16] times the gravity (9.81m/s²)
 *                      sensi =    [16384,8192,4096,2048] bit/gravity
*/

#ifndef MPU6050_LIGHT_H
#define MPU6050_LIGHT_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

// --------------------------------------------
#define MPU6050_GYRO_CONFIG_1 // choose here to define _0 or _1
#define MPU6050_ACC_CONFIG_0  // choose here to define _0 or _1

#ifdef MPU6050_GYRO_CONFIG_0
  #define MPU6050_GYRO_CONFIG   0x00  // range = +-250 °/s
  #define GYRO_LSB_2_DEGSEC     131.0 // [bit/(°/s)] for gyro config 0
#endif
#ifdef MPU6050_GYRO_CONFIG_1
  #define MPU6050_GYRO_CONFIG   0x08  // range = +-500 °/s
  #define GYRO_LSB_2_DEGSEC     65.5  // [bit/(°/s)] for gyro config 1
#endif

#ifdef MPU6050_ACC_CONFIG_0
  #define MPU6050_ACCEL_CONFIG  0x00     // range = +- 2 g
  #define ACC_LSB_2_G           16384.0  // [bit/gravity] for accel config 0
#endif
#ifdef MPU6050_ACC_CONFIG_1
  #define MPU6050_ACCEL_CONFIG  0x08     // range = +- 4 g
  #define ACC_LSB_2_G           8192.0  // [bit/gravity] for accel config 1
#endif
// --------------------------------------------

#define RAD_2_DEG             57.29578 // [°/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

class MPU6050{
  public:
    // INIT and BASIC FUNCTIONS
	MPU6050(TwoWire &w);
    MPU6050(TwoWire &w, float gyro_coeff);
    byte begin();
	
	byte writeData(byte reg, byte data);
    byte readData(byte reg);
	
	void calcGyroOffsets();
	void calcAccOffsets();
	
	// MPU CONFIG SETTER
    void setGyroOffsets(float x, float y, float z);
	void setAccOffsets(float x, float y, float z);
	
	void setFilterGyroCoef(float gyro_coeff);
	void setFilterAccCoef(float acc_coeff);

	// MPU CONFIG GETTER
	float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };
	
	float getAccXoffset(){ return accXoffset; };
	float getAccYoffset(){ return accYoffset; };
	float getAccZoffset(){ return accZoffset; };
	
	float getFilterGyroCoef(){ return filterGyroCoef; };
	float getFilterAccCoef(){ return 1.0-filterGyroCoef; };
	
	// INLOOP GETTER
    float getTemp(){ return temp; };

    float getAccX(){ return accX; };
    float getAccY(){ return accY; };
    float getAccZ(){ return accZ; };

    float getGyroX(){ return gyroX; };
    float getGyroY(){ return gyroY; };
    float getGyroZ(){ return gyroZ; };
	
	float getAccAngleX(){ return angleAccX; };
    float getAccAngleY(){ return angleAccY; };

    float getAngleX(){ return angleX; };
    float getAngleY(){ return angleY; };
    float getAngleZ(){ return angleZ; };

	// INLOOP UPDATE
    void update();


  private:
    TwoWire *wire;
    float gyroXoffset, gyroYoffset, gyroZoffset;
	float accXoffset, accYoffset, accZoffset;
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    long preInterval;
    float filterGyroCoef; // complementary filter coefficient to balance gyro vs accelero data to get angle
};

#endif
