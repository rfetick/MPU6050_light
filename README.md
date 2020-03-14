# MPU6050_light

Lightweight, fast and simple library to communicate with the MPU6050

## Description

The library is made to retrieve accelero and gyro data from the MPU6050. This data is processed using a complementary filter to provide tilt angles on X and Y with respect to the horizontal rest position. The hypothesis for the validity of these angles are:
* small linear accelerations (the gravity is the dominant one)
* small loop delay between two calls to `update()` so the approximation `angle[t]=angle[t-1]+gyro*dt` is valid
* heading (angle Z) is valid for small X and Y angles

The default complementary filter is `0.98` for the gyro data and `0.02` for the acccelero data. This filter compensates for the gyro drift and for the relatively high accelero noise.

All data is available through getters.

| NAME           | DESCRIPTION                                              |
|----------------|----------------------------------------------------------|
| getAccX-Y-Z    | Acceleration on X, Y or Z [units of g=9.81m/s²]          |
| getGyroX-Y-Z   | Angular speed en X, Y or Z [deg/s]                       |
| getAccAngleX-Y | Angles computed from accelero data [deg]. Warning: noisy |
| getAngleX-Y-Z  | Angles computed with complementary filter [deg]          |
| getTemp        | Device temperature [°C]                                  |

## Examples

A minimal code to retrieve some MPU6050 data would be

```cpp
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
}

void loop() {
  mpu.update();
  float angle[3] = {mpu.getAngleX(),mpu.getAngleY(),mpu.getAngleZ()};
  float gyro[3] = {mpu.getGyroX(),mpu.getGyroY(),mpu.getGyroZ()};
  // process this data for your needs...
}
```

Note that ready-to-run examples are also provided in the dedicated `examples` folder. You may have a look at them in order to get started.

## Authors

[rfetick](https://github.com/rfetick) : modifications for better memory management, speed and efficiency.

[tockn](https://github.com/tockn) : initial author of the library (v1.5.2)
