#ifndef MPU6050_LIGHT_H
#define MPU6050_LIGHT_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

#define GYRO_LSB_2_DEGSEC  65.5     // [bit/(°/s)]
#define ACC_LSB_2_G        16384.0  // [bit/gravity]
#define RAD_2_DEG          57.29578 // [°/rad]
#define GYRO_OFFSET_NB_MES 3000     //
#define TEMP_LSB_2_DEGREE  340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET    12412.0  //

#define DEFAULT_GYRO_COEFF 0.98

class MPU6050{
  public:
    MPU6050(TwoWire &w);
    MPU6050(TwoWire &w, float aC, float gC);
    void begin();
    void setGyroOffsets(float x, float y, float z);

    void writeMPU6050(byte reg, byte data);
    byte readMPU6050(byte reg);

    float getTemp(){ return temp; };

    float getAccX(){ return accX; };
    float getAccY(){ return accY; };
    float getAccZ(){ return accZ; };

    float getGyroX(){ return gyroX; };
    float getGyroY(){ return gyroY; };
    float getGyroZ(){ return gyroZ; };

    void calcGyroOffsets();

    float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };

    void update();

    float getAccAngleX(){ return angleAccX; };
    float getAccAngleY(){ return angleAccY; };

    float getAngleX(){ return angleX; };
    float getAngleY(){ return angleY; };
    float getAngleZ(){ return angleZ; };

  private:
    TwoWire *wire;
    float gyroXoffset, gyroYoffset, gyroZoffset;
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY, angleAccZ;
    float angleX, angleY, angleZ;
    long preInterval;
    float accCoef, gyroCoef;
};

#endif
