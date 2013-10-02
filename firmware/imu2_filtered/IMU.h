/*
Copyright 2013 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// IMU header file (imu.h)

#include "DSP2833x_Device.h"
#include <string.h>

// Board specific calibration info
#include "cal_null.h"
//#include "../calibration/IMU10_20120604T1806.h"

#define CPU_CLOCK_SPEED 6.667L   // for a 150MHz CPU clock speed
#define DELAY_US(A) DSP28x_usDelay(((((long double) A * 1000.0L) /
        (long double)CPU_CLOCK_SPEED) - 9.0L) / 5.0L)
extern void DSP28x_usDelay(unsigned long Count);

// Pin definitions (A: 0-31, B: 32-63, C: 64-79)
#define SER_DI GPIO22
#define SER_RO GPIO23
#define SER_nRE GPIO24
#define SER_DE GPIO25
#define SER_HnF GPIO26

#define ACCEL3_CS GPIO29
#define ACCEL3_SPC GPIO56
#define ACCEL3_SDI GPIO54
#define ACCEL3_SDO GPIO55
#define ACCEL3_RDYINT GPIO0

#define ID_SDA GPIO32
#define ID_SCL GPIO33

#define MAG_RESET GPIO9

#define nLED0 GPIO70
#define nLED1 GPIO67

#define SPARE0 GPIO34
#define SPARE1 GPIO41
#define SPARE2 GPIO44
#define SPARE_MDX GPIO12
#define SPARE_MDR GPIO13
#define SPARE_MCLKX GPIO14

#define LED1_PATTERN 0xA6A6A6A6

// Used to set the output rate during debug mode.  This has no impact on the
// filter decimation rate.
#define OVERSAMPLE_RATE 4

typedef struct IMURawData_t {
  // Raw Sensor reading
  Uint16 accel_x;
  Uint16 accel_y;
  Uint16 accel_z;
  Uint16 gyro_x;
  Uint16 gyro_y;
  Uint16 gyro_z;
  Uint16 mag_x;
  Uint16 mag_y;
  Uint16 mag_z;

  // Temperatures
  Uint16 accel_x_t;
  Uint16 accel_y_t;
  Uint16 accel_z_t;
  Uint16 gyro_x_t;
  Uint16 gyro_y_t;
  Uint16 gyro_z_t;

  // Voltage Reference
  Uint16 ref_mon;

  // 3-Axis Readings
  int16 accel3_x;
  int16 accel3_y;
  int16 accel3_z;

  // Serial Number
  Uint64 serial_id;
} IMURawData;

extern IMURawData rawData;

typedef struct IMUConvData_t {

  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;

  float accel3_x;
  float accel3_y;
  float accel3_z;

  float ref_mon;
} IMUConvData;

extern IMUConvData convData;

typedef struct IMUFilterData_t {

  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;

} IMUFilterData;

extern IMUFilterData filterData;
extern IMUFilterData filterInputData;

extern char debug_mode;
