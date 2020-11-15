# MPU6050_light ![bdg](https://img.shields.io/github/license/rfetick/MPU6050_light) ![bdg](https://img.shields.io/github/v/release/rfetick/MPU6050_light) ![bdg](https://img.shields.io/github/commits-since/rfetick/MPU6050_light/latest)

**Lightweight, fast and simple library to communicate with the MPU6050**

:arrows_counterclockwise: Your feedback is important. Any issue or suggestion can be reported to the `Issues` section


## Description

The library is made to retrieve accelero and gyro data from the MPU6050. This data is processed using a complementary filter to provide tilt angles on X and Y with respect to the horizontal rest position. The hypothesis for the validity of these angles are:
* small linear accelerations (the gravity is the dominant one)
* small loop delay between two calls to `update()` so the approximation `angle[t]=angle[t-1]+gyro*dt` is valid
* heading (angle Z) is valid for small X and Y angles

The default complementary filter is `0.98` for the gyro data and `0.02` for the acccelero data. This filter compensates for the gyro drift and for the relatively high accelero noise.

The important methods of the MPU6050 are the following ones.

| FUNCTION               | DESCRIPTION                                                |
|------------------------|------------------------------------------------------------|
| MPU6050(Wire)          | Constructor                                                |
| begin()                | Start communication with MPU6050                           |
| calcOffsets(bool,bool) | Compute offsets. Device must be stable meanwhile           |
| update()               | Update data. Must be called often to get consistent angles |

All data is available through the following getters. They all return a `float`.

| FUNCTION         | DESCRIPTION                                        | UNIT          |
|------------------|----------------------------------------------------|---------------|
| getAccX-Y-Z()    | Acceleration on X, Y or Z                          | g (=9.81m/s²) |
| getGyroX-Y-Z()   | Angular speed en X, Y or Z                         | deg/s         |
| getAccAngleX-Y() | Angles computed from accelero data. Warning: noisy | deg           |
| getAngleX-Y-Z()  | Angles computed with complementary filter          | deg           |
| getTemp()        | Device temperature                                 | °C            |

The gyro offsets can also be provided by the user if they are already known.

| FUNCTION                 | DESCRIPTION                                  | UNIT          |
|--------------------------|----------------------------------------------|---------------|
| setGyroOffsets(gx,gy,gz) | Set gyro offsets on X, Y or Z                | deg/s         |
| setAccOffsets(ax,ay,az)  | Set accelero offsets on X, Y or Z            | g (=9.81m/s²) |

## Examples

A minimal code to retrieve some MPU6050 data would be

```cpp
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();
}

void loop() {
  mpu.update();
  float angle[3] = {mpu.getAngleX(),mpu.getAngleY(),mpu.getAngleZ()};
  float gyro[3] = {mpu.getGyroX(),mpu.getGyroY(),mpu.getGyroZ()};
  // process this data for your needs...
}
```

Some ready-to-run examples are also provided in the dedicated `examples` folder. You may have a look at them in order to get started.

## License

See the LICENSE file

## Authors

[rfetick](https://github.com/rfetick) : modifications for better memory management, speed and efficiency.

[tockn](https://github.com/tockn) : initial author of the library (v1.5.2)
