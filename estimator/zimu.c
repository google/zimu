// Copyright 2013 Google Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//     See the License for the specific language governing permissions and
//     limitations under the License.
#include "zimu.h"

#include <stdint.h>

#include "comp_filter.h"

SystemParams sys = {
  .ts = 0.004,
  .phys = {
    .g_g = {0.0, 0.0, 9.81},
    .mag_g = {0.2, 0.06, 0.4}
  }
};

const SystemParams *g_sys = &sys;

int32_t zimu_init = 0;


int main(void) {
  Vec3 acc, pqr, mag, wind, X_ned_est, V_ned_est;
  Vec3 X_ned_meas[COMP_FILTER_NUM_POS_VEC] = {{0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0}};
  Vec3 V_ned_meas[COMP_FILTER_NUM_VEL_VEC] = {{0.0, 0.0, 0.0},
                                              {0.0, 0.0, 0.0}};
  Mat3 dcm_ned2b;
  CompFilterParams pm = {
    .alpha_nom = 0.0,
    .beta_nom = 0.0,
    .acc_window = 1.0,
    .fc_att = 1.0,
    .k_bias = 1.0,
    .ks_att = {1.0, 1.0, 1.0},
    .err_fcs_att = {1.0, 1.0, 1.0},
    .q_0 = {1.0, 0.0, 0.0, 0.0},
    .pqr_bias_low = -0.1,
    .pqr_bias_high = 0.1,
    .fc_vel = 1.0,
    .fc_pos = 1.0,
    .ks_vel = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}},
    .ks_pos = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}}
  };

  RunCompFilter(&acc, &pqr, &mag, &wind, X_ned_meas, V_ned_meas, &pm,
                &dcm_ned2b, &X_ned_est, &V_ned_est);
  return 0;
}


double Saturate(double x, double low, double high) {
  return fmin(fmax(x, low), high);
}
