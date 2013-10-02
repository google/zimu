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

// Null calibration data file (cal_null.h)

// This header file can be replace with a board specific file for calibration.
// TODO: Add calibration information to flash independent of compliation.

// CGF = calibration gain factor (float)
// CBO = calibration bias offset (float)
// CTN = calibration temperature nominal (Uint16)
// CTS = calibration temperature sensitivity (float)

// LED blink code
#define LED0_PATTERN 0x33333333

// accel_x
#define AX_CGF		1.0
#define AX_CBO		0.0
#define AX_CTN		0
#define AX_CTS		0.0

// accel_y
#define AY_CGF		1.0
#define AY_CBO		0.0
#define AY_CTN		0
#define AY_CTS		0.0

// accel_z
#define AZ_CGF		1.0
#define AZ_CBO		0.0
#define AZ_CTN		0
#define AZ_CTS		0.0

// gyro_x
#define GX_CGF		1.0
#define GX_CBO		0.0
#define GX_CTN		0
#define GX_CTS		0.0

// gyro_y
#define GY_CGF		1.0
#define GY_CBO		0.0
#define GY_CTN		0
#define GY_CTS		0.0

// gyro_z
#define GZ_CGF		1.0
#define GZ_CBO		0.0
#define GZ_CTN		0
#define GZ_CTS		0.0

// mag_x (uses gyro_y_t)
#define MX_CGF		1.0
#define MX_CBO		0.0
#define MX_CTN	   0
#define MX_CTS		0.0

// mag_y (uses gyro_x_t)
#define MY_CGF		1.0
#define MY_CBO		0.0
#define MY_CTN	   0
#define MY_CTS		0.0

// mag_z (uses gyro_x_t)
#define MZ_CGF		1.0
#define MZ_CBO		0.0
#define MZ_CTN	   0
#define MZ_CTS		0.0

// accel3_x
#define A3X_CGF	1.0
#define A3X_CBO	0.0
#define A3X_CTN	0
#define A3X_CTS	0.0

// accel3_y
#define A3Y_CGF 	1.0
#define A3Y_CBO	0.0
#define A3Y_CTN	0
#define A3Y_CTS 	0.0

// accel3_z
#define A3Z_CGF	1.0
#define A3Z_CBO   0.0
#define A3Z_CTN   0
#define A3Z_CTS   0.0

