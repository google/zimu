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

// Ideal EU conversions

// accel_x, accel_y, and gyro_z are negative
// mag x is negative

// Accel is in g's
// Gyro is in degress/s
// Mag is in Gauss

// EUG = Engineering Unit Gain
// EUO = Engineering Unit Offset

#define MAX_COUNT 	4096	// 12-bit data

// Unsigned measurements
#define AX_EUG		-125.0/MAX_COUNT	// accel_x
#define AX_EUO		62.5
#define AY_EUG		-125.0/MAX_COUNT	// accel_y
#define AY_EUO 		62.5
#define AZ_EUG 		125.0/MAX_COUNT		// accel_z
#define AZ_EUO		-62.5
#define GX_EUG 		1285.8/MAX_COUNT	// gyro_x
#define GX_EUO		-642.9
#define GY_EUG 		1285.8/MAX_COUNT	// gyro_y
#define GY_EUO		-642.9
#define GZ_EUG 		-1285.8/MAX_COUNT	// gyro_z
#define GZ_EUO		642.9
#define MX_EUG 		-4.0/MAX_COUNT		// mag_x
#define MX_EUO		2.0
#define MY_EUG 		4.0/MAX_COUNT		// mag_y
#define MY_EUO		-2.0
#define MZ_EUG 		4.0/MAX_COUNT		// mag_z
#define MZ_EUO		-2.0
#define RM_EUG		6.0/MAX_COUNT		// ref_mon
#define RM_EUO		0

// Signed measurements
#define A3X_EUG		12.0/MAX_COUNT		// accel3_x
#define A3X_EUO		0.0
#define A3Y_EUG 	12.0/MAX_COUNT		// accel3_y
#define A3Y_EUO		0.0
#define A3Z_EUG 	12.0/MAX_COUNT		// accel3_z
#define A3Z_EUO		0.0


