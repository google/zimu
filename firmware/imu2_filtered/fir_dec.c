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

// Decimating FIR filter (fir_dec.c)
//
// This implements a decimating fir filter on the three gyro and acceleration
// channels of the IMU. Rather than storing a record of the incoming data to
// convolve the taps with, this implementation stores the internal state of
// muliple filters at various stages of completion.  When a new data point
// comes in, each state does a multiply and accumulate with the data point and
// the appropriate tap.  This implementation uses less memory, and is simpler
// for high decimation factors.  For decimation factors of 2 and less, it would
// be better to use the traditional implementation.  The main disadvantage is
// that each state has an associated tap pointer that needs to be updated.

// Performance:
// With num_taps = 16 and decimation_factor = 4,
//      this filter takes 15 us per input sample.
// With num_taps = 16 and decimation_factor = 1,
//      this filter takes 50 us per input sample.


#include "IMU.h"
#include "EU_info.h"

// The number of taps should be a multilple of the decimation factor
#define num_taps 16
#define decimation_factor 1
#define num_states num_taps/decimation_factor

extern float EUConv(Uint16 data, float gain, float offset);

// Filter taps
// 16 tap filter designed for an ouput rate of 250 Hz.
float taps[num_taps] = {
  -2.783314324915409e-003,
  -6.203002762049437e-003,
  -4.964116960763931e-003,
  1.047581154853106e-002,
  4.715123772621155e-002,
  1.019511222839356e-001,
  1.588822901248932e-001,
  1.954899877309799e-001,
  1.954899877309799e-001,
  1.588822901248932e-001,
  1.019511222839356e-001,
  4.715123772621155e-002,
  1.047581154853106e-002,
  -4.964116960763931e-003,
  -6.203002762049437e-003,
  -2.783314324915409e-003
};

// Store the intermediate states of the filter stages in accumulators
float accum_accel_x[num_states];
float accum_accel_y[num_states];
float accum_accel_z[num_states];
float accum_gyro_x[num_states];
float accum_gyro_y[num_states];
float accum_gyro_z[num_states];

// The tap pointer points to the current tap for the given accumulator.
Uint16 tap_ptr[num_states];

// Copies a filtered datapoint to the filterData structure.
#pragma CODE_SECTION(outputFilterData,"ramfuncs");
void outputFilterData(int16 state)
{
  // Correct for the dc error in the taps and copy values to filterData struct
  filterData.accel_x = accum_accel_x[state];
  filterData.accel_y = accum_accel_y[state];
  filterData.accel_z = accum_accel_z[state];
  filterData.gyro_x = accum_gyro_x[state];
  filterData.gyro_y = accum_gyro_y[state];
  filterData.gyro_z = accum_gyro_z[state];

  // Reset filter intermediate state to zero
  accum_accel_x[state] = 0;
  accum_accel_y[state] = 0;
  accum_accel_z[state] = 0;
  accum_gyro_x[state] = 0;
  accum_gyro_y[state] = 0;
  accum_gyro_z[state] = 0;
}

// Updates the filter with an input data point.  If the decimation cycle is
// complete, it outputs a datapoint and returns 1.  If the decimation cycle is
// not complete, it returns 0.
#pragma CODE_SECTION(updateFilter,"ramfuncs");
Uint16 updateFilter(void)
{
  int16 state;

  // stores the state that has completed a filter cycle
  int16 finished_state = -1;

  // Do convolution for each intermediate state
  for (state=0; state<num_states; state++) {

    // Save input data for debugging
    filterInputData.accel_x = EUConv(rawData.accel_x, AX_EUG, AX_EUO);
    filterInputData.accel_y = EUConv(rawData.accel_y, AY_EUG, AY_EUO);
    filterInputData.accel_z = EUConv(rawData.accel_z, AZ_EUG, AZ_EUO);
    filterInputData.gyro_x = EUConv(rawData.gyro_x, GX_EUG, GX_EUO);
    filterInputData.gyro_y = EUConv(rawData.gyro_y, GY_EUG, GY_EUO);
    filterInputData.gyro_z = EUConv(rawData.gyro_z, GZ_EUG, GZ_EUO);

    // Do MAC
    accum_accel_x[state] += filterInputData.accel_x * taps[tap_ptr[state]];
    accum_accel_y[state] +=  filterInputData.accel_y * taps[tap_ptr[state]];
    accum_accel_z[state] += filterInputData.accel_z * taps[tap_ptr[state]];
    accum_gyro_x[state] += filterInputData.gyro_x * taps[tap_ptr[state]];
    accum_gyro_y[state] += filterInputData.gyro_y * taps[tap_ptr[state]];
    accum_gyro_z[state] += filterInputData.gyro_z * taps[tap_ptr[state]];

    // increment tap pointer
    if (tap_ptr[state] == num_taps-1) {
      tap_ptr[state] = 0;

      // if the pointer rolls over, mark this as a finished datapoint
      finished_state = state;
    } else {
      tap_ptr[state]++;
    }
  }

  // Determine if this is the end of a decimation cycle
  if (finished_state != -1) {
    outputFilterData(finished_state);
    return 1;
  }
  return 0;
}

// Initialize filter
void initFilter(void)
{
  int16 state;

  // init tap pointers and filter accumlators
  for (state=0; state<num_states; state++) {

    tap_ptr[state] = state * decimation_factor;

    accum_accel_x[state] = 0;
    accum_accel_y[state] = 0;
    accum_accel_z[state] = 0;
    accum_gyro_x[state] = 0;
    accum_gyro_y[state] = 0;
    accum_gyro_z[state] = 0;
  }
}



