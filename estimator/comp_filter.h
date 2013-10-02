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
#ifndef COMP_FILTER_H_
#define COMP_FILTER_H_

#include "linalg.h"
#include "quaternion.h"

#define COMP_FILTER_NUM_ATT_VEC 3
typedef enum {
  COMP_FILTER_MAG_VEC = 0,
  COMP_FILTER_APP_WIND_VEC = 1,
  COMP_FILTER_GRAVITY_VEC = 2
} AttitudeVecType;

#define COMP_FILTER_NUM_VEL_VEC 2
#define COMP_FILTER_NUM_POS_VEC 3

typedef struct {
  double alpha_nom, beta_nom;
  double acc_window;

  // Attitude filter parameters
  double fc_att, k_bias;
  double ks_att[COMP_FILTER_NUM_ATT_VEC];
  double err_fcs_att[COMP_FILTER_NUM_ATT_VEC];
  Quat q_0;
  double pqr_bias_low, pqr_bias_high;

  // Velocity/position filter parameters
  double fc_vel, fc_pos;
  Vec3 ks_vel[COMP_FILTER_NUM_VEL_VEC];
  Vec3 ks_pos[COMP_FILTER_NUM_POS_VEC];
} CompFilterParams;


void RunCompFilter(const Vec3 *acc, const Vec3 *pqr, const Vec3 *mag,
                   const Vec3 *wind,
                   const Vec3 Xg_meas[COMP_FILTER_NUM_POS_VEC],
                   const Vec3 Vg_meas[COMP_FILTER_NUM_VEL_VEC],
                   const CompFilterParams *pm,
                   Mat3 *dcm_g2b, Vec3 *Xg_est, Vec3 *Vg_est);

void FilterAttitude(const Vec3 vis[COMP_FILTER_NUM_ATT_VEC],
                    const Vec3 vbs[COMP_FILTER_NUM_ATT_VEC],
                    double fc, double k_bias,
                    const double ks[COMP_FILTER_NUM_ATT_VEC],
                    const double err_fcs[COMP_FILTER_NUM_ATT_VEC],
                    const Vec3 *pqr, const CompFilterParams *pm,
                    Mat3 *dcm_i2b, Vec3 *bias_out);
const Vec3 *FilterVel(const Vec3 Vg_meas[COMP_FILTER_NUM_VEL_VEC],
                      double fc, const Vec3 ks[COMP_FILTER_NUM_VEL_VEC],
                      const Mat3 *dcm_g2b, const Vec3 *acc, const Vec3 *pqr,
                      Vec3 *Vg_est);
const Vec3 *FilterPos(const Vec3 Xg_meas[COMP_FILTER_NUM_POS_VEC],
                      double fc, const Vec3 ks[COMP_FILTER_NUM_POS_VEC],
                      const Vec3 *Vg_est, Vec3 *Xg_est);

double LPF(double u, double y, double fc, double ts, double *y_z1);
const Vec3 *LPFVec3(const Vec3 *u, const Vec3 *y, double fc, double ts,
                    Vec3 *y_out, Vec3 *y_z1);

const Vec3 *CalcApparentWind(const Vec3 *Vg, const Vec3 *wind,
                             double alpha, double beta,
                             Vec3 *Vg_app, Vec3 *Vb_app);
double CalcSpecificForce(const Vec3 *Ai, const Vec3 *acc, const Vec3 *g_g,
                         double acc_window, Vec3 *Fg, Vec3 *Fb);

#endif  // COMP_FILTER_H_
