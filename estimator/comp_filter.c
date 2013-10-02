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
//
//
// Description:
//   Adaptive complementary attitude/velocity/position filter
//
//   Weight each sensor's contribution to the state estimate based on
//   some additional measurement that represents how confident we are in
//   the measurement.
//
// Coordinate systems:
//   Body coordinates, denoted "b", are fixed to the aircraft
//   (typically x forward, y starboard, z down).  It is assumed that
//   the IMU measurements are returned in body coordinates.
//
//   Ground coordinates, denoted "g", are fixed to the ground (e.g. a
//   North-East-Down system.
//
//   Inertial coordinates, denoted "i", are a true inertial system
//   that is approximated by the ground system.
#include "comp_filter.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "linalg.h"
#include "zimu.h"
#include "quaternion.h"

void RunCompFilter(const Vec3 *acc, const Vec3 *pqr, const Vec3 *mag,
                   const Vec3 *wind,
                   const Vec3 Xg_meas[COMP_FILTER_NUM_POS_VEC],
                   const Vec3 Vg_meas[COMP_FILTER_NUM_VEL_VEC],
                   const CompFilterParams *pm,
                   Mat3 *dcm_g2b, Vec3 *Xg_est, Vec3 *Vg_est) {
  // Prepare attitude vectors
  Vec3 vis[COMP_FILTER_NUM_ATT_VEC], vbs[COMP_FILTER_NUM_ATT_VEC];
  CalcApparentWind(Vg_meas, wind, pm->alpha_nom, pm->beta_nom,
                   &vis[COMP_FILTER_APP_WIND_VEC],
                   &vbs[COMP_FILTER_APP_WIND_VEC]);
  CalcSpecificForce(&Vec3_0, acc, &g_sys->phys.g_g, pm->acc_window,
                    &vis[COMP_FILTER_GRAVITY_VEC],
                    &vbs[COMP_FILTER_GRAVITY_VEC]);
  vis[COMP_FILTER_MAG_VEC] = g_sys->phys.mag_g;
  vbs[COMP_FILTER_MAG_VEC] = *mag;

  Vec3 pqr_bias;
  FilterAttitude(vis, vbs, pm->fc_att, pm->k_bias, pm->ks_att, pm->err_fcs_att,
                 pqr, pm, dcm_g2b, &pqr_bias);
  FilterVel(Vg_meas, pm->fc_vel, pm->ks_vel, dcm_g2b, acc, pqr, Vg_est);
  FilterPos(Xg_meas, pm->fc_pos, pm->ks_pos, Vg_est, Xg_est);
}


// General form of Mahony's nonlinear complementary filter.
//
// Low frequency attitude information from vector measurements
// (gravity vector, magnetic field vector, apparent wind vector) are
// combined with high frequency attitude information from the gyros
// using first- (or optionally second-) order low/high pass filters.
//
// Inputs:
//   vis[]     -- Vectors in inertial frame
//   vbs[]     -- Vectors in body frame
//   fc        -- Cut-off frequency [Hz]
//   k_bias    -- Integral gain on vector error for bias estimation
//   ks[]      -- Weight for each vector measurement (must sum to 1)
//   err_fcs[] -- Cut-off frequency for first order LPF on vector errors
//   pqr       -- Angular rate vector
//
// Persistent state:
//   q         -- estimated attitude quaternion
//   bias      -- estimated gyro bias
//   ef_z1s[]  -- filtered vector errors
void FilterAttitude(const Vec3 vis[COMP_FILTER_NUM_ATT_VEC],
                    const Vec3 vbs[COMP_FILTER_NUM_ATT_VEC],
                    double fc, double k_bias,
                    const double ks[COMP_FILTER_NUM_ATT_VEC],
                    const double err_fcs[COMP_FILTER_NUM_ATT_VEC],
                    const Vec3 *pqr, const CompFilterParams *pm,
                    Mat3 *dcm_i2b, Vec3 *bias_out) {
  static Quat q;
  static Vec3 bias, ef_z1s[COMP_FILTER_NUM_ATT_VEC];
  if (zimu_init == ZIMU_INIT_START) {
    q = pm->q_0;
    bias = Vec3_0;
    for (int32_t i = 0; i < COMP_FILTER_NUM_ATT_VEC; ++i)
      ef_z1s[i] = Vec3_0;
  }

  Vec3 vi_norms[COMP_FILTER_NUM_ATT_VEC], vb_norms[COMP_FILTER_NUM_ATT_VEC];
  Vec3 vb_ests[COMP_FILTER_NUM_ATT_VEC];
  Vec3 es[COMP_FILTER_NUM_ATT_VEC], efs[COMP_FILTER_NUM_ATT_VEC];
  Vec3 w_est, w_err = Vec3_0;

  for (int32_t i = 0; i < COMP_FILTER_NUM_ATT_VEC; ++i) {
    // Normalize vectors (safely)
    Vec3Scale(&vis[i], 1.0/Vec3NormBound(&vis[i], 1.0e-6), &vi_norms[i]);
    Vec3Scale(&vbs[i], 1.0/Vec3NormBound(&vbs[i], 1.0e-6), &vb_norms[i]);

    // Estimated inertial vectors in body coordinates
    QuatRotate(&q, &vi_norms[i], &vb_ests[i]);

    // Angular rate error from vector measurements
    Vec3Cross(&vb_norms[i], &vb_ests[i], &es[i]);

    // Filter the vector errors (note: fc<=0 implies no filtering)
    if (err_fcs[i] > 0.0) {
      LPFVec3(&es[i], &ef_z1s[i], err_fcs[i], g_sys->ts, &efs[i], &ef_z1s[i]);
    } else {
      ef_z1s[i] = es[i];
      efs[i] = es[i];
    }

    // Angular rate error based on vectors alone
    Vec3Axpy(ks[i], &efs[i], &w_err, &w_err);
  }

  // Gyro bias integration
  Vec3LinComb(1.0, &bias, -k_bias*g_sys->ts, &w_err, &bias);
  bias.x = Saturate(bias.x, pm->pqr_bias_low, pm->pqr_bias_high);
  bias.y = Saturate(bias.y, pm->pqr_bias_low, pm->pqr_bias_high);
  bias.z = Saturate(bias.z, pm->pqr_bias_low, pm->pqr_bias_high);

  // Estimated angular rate (omega_ref)
  Vec3LinComb3(1.0, pqr, -1.0, &bias, 2.0*PI*fc, &w_err, &w_est);

  // Quaternion integration
  Quat dq, q_omega = {0.0, 0.5*w_est.x, 0.5*w_est.y, 0.5*w_est.z};
  QuatMultiply(&q, &q_omega, &dq);
  QuatLinComb(1.0, &q, g_sys->ts, &dq, &q);
  QuatNormalize(&q, &q);

  // Convert to DCM representation
  QuatToDCM(&q, dcm_i2b);

  *bias_out = bias;
}


// Complementary velocity filter
//
// Low-frequency velocity information from multiple velocity
// measurements (each with a weight, ks[i], that must sum to one) is
// combined with high frequency velocity information from the
// accelerometer using second-order low/high pass filters.  This
// filter is performed in the body frame because the accelerometer
// biases are in the body frame.
//
//   Vb = LPF(s) * (ks[0] * Vb_meas[0] + ks[1] * Vb_meas[1] + ...)
//        + HPF(s) * Ab_meas/s
//
//                       (2*pi*fc)^2
//   LPF(s) = ----------------------------------     HPF(s) = 1 - LPF(s)
//            s^2 + 4*pi*zeta*fc*s + (2*pi*fc)^2
//
// Inputs:
//   Vg_meas[]   -- Velocity measurements
//   fc          -- Cut-off frequency [Hz]
//   ks[]        -- Weight for each measurement (must sum to 1 for each axis!)
//   dcm_g2b     -- Estimated attitude
//   acc         -- Acceleration measured by IMU
//   pqr         -- Angular rate vector
//
// Persistent state:
//   Vb          -- Last velocity in body coord (used in 2nd order filter)
//   Vb_err_fs[] -- Low pass filter of Vb_meas[i] - Vb_est
const Vec3 *FilterVel(const Vec3 Vg_meas[COMP_FILTER_NUM_VEL_VEC],
                      double fc, const Vec3 ks[COMP_FILTER_NUM_VEL_VEC],
                      const Mat3 *dcm_g2b, const Vec3 *acc, const Vec3 *pqr,
                      Vec3 *Vg_est) {
  static Vec3 Vb, Vb_err_fs[COMP_FILTER_NUM_VEL_VEC];
  if (zimu_init == ZIMU_INIT_START) {
    Vb = Vec3_0;
    for (int32_t i = 0; i < COMP_FILTER_NUM_VEL_VEC; ++i)
      Vb_err_fs[i] = Vec3_0;
  }

  const double zeta = sqrt(2.0)/2.0;
  Vec3 g_b, dVb_hp, dVb_lp = Vec3_0, Vb_meas, tmp;

  // High frequency body velocity information comes from the
  // accelerometer and gyro
  Mat3Vec3Mult(dcm_g2b, &g_sys->phys.g_g, &g_b);
  Vec3Add3(acc, &g_b, Vec3Cross(&Vb, pqr, &tmp), &dVb_hp);

  // Low frequency body velocity information comes from the various
  // direct velocity measurements (e.g. GPS, etc...)
  for (int32_t i = 0; i < COMP_FILTER_NUM_VEL_VEC; ++i) {
    Mat3Vec3Mult(dcm_g2b, &Vg_meas[i], &Vb_meas);
    Vec3LinComb3(1.0, &Vb_meas, -1.0, &Vb, -1.0, &Vb_err_fs[i], &tmp);
    Vec3LinComb(1.0, &Vb_err_fs[i], 4.0*PI*fc*zeta*g_sys->ts, &tmp,
                &Vb_err_fs[i]);
    Vec3LinComb(1.0, &dVb_lp, PI*fc/zeta, Vec3Mult(&ks[i], &Vb_err_fs[i], &tmp),
                &dVb_lp);
  }

  // The high- and low-frequency velocity change information are
  // combined and integrated.
  Vec3LinComb3(1.0, &Vb, g_sys->ts, &dVb_lp, g_sys->ts, &dVb_hp, &Vb);
  Mat3TransVec3Mult(dcm_g2b, &Vb, Vg_est);
  return Vg_est;
}


// Complementary position filter
//
// Low-frequency position information from multiple position
// measurements (each with a weight, ks[i], that must sum to one) is
// combined with high frequency position information from a single
// velocity estimate using simple first-order low/high pass filters.
//
//   Xg = LPF(s) * (ks[0] * Xg_meas[0] + ks[1] * Xg_meas[1] + ...)
//        + HPF(s) * Vg_meas/s
//
//               2*pi*fc
//   LPF(s) =  -----------        HPF(s) = 1 - LPF(s)
//             s + 2*pi*fc
//
// Inputs:
//   Xg_meas[] -- Position measurements
//   fc        -- Cut-off frequency [Hz]
//   ks[]      -- Weight for each measurement (must sum to 1 for each axis!)
//   Vg_est    -- Estimated velocity
//
// Persistent state:
//   Xg        -- Last estimated position
const Vec3 *FilterPos(const Vec3 Xg_meas[COMP_FILTER_NUM_POS_VEC],
                      double fc, const Vec3 ks[COMP_FILTER_NUM_POS_VEC],
                      const Vec3 *Vg_est, Vec3 *Xg_est) {
  static Vec3 Xg;
  if (zimu_init == ZIMU_INIT_START) {
    Xg = Xg_meas[0];
  }

  Vec3 tmp, ks_dXg_tot = Vec3_0;
  for (int32_t i = 0; i < COMP_FILTER_NUM_POS_VEC; ++i) {
    Vec3Mult(&ks[i], Vec3Sub(&Xg_meas[i], &Xg, &tmp), &tmp);
    Vec3LinComb(1.0, &ks_dXg_tot, 2.0*PI*fc, &tmp, &ks_dXg_tot);
  }

  Vec3LinComb(1.0, &Xg, g_sys->ts, Vec3Add(&ks_dXg_tot, Vg_est, &tmp), &Xg);
  *Xg_est = Xg;
  return Xg_est;
}


// Returns the nominal apparent wind vector (assuming a given alpha
// and beta) in the body and ground coordinates.
const Vec3 *CalcApparentWind(const Vec3 *Vg, const Vec3 *wind,
                             double alpha, double beta,
                             Vec3 *Vg_app, Vec3 *Vb_app) {
  // Long term average apparent wind vector in body frame
  Vec3 Vb_app_nom = {cos(alpha)*cos(beta), cos(alpha)*sin(beta), sin(alpha)};
  Vec3Sub(Vg, wind, Vg_app);
  Vec3Scale(&Vb_app_nom, Vec3Norm(Vg_app), Vb_app);
  return Vg_app;
}


// Returns the expected acceleration in ground coordinates.  Also,
// returns a multiplier for how much to trust the "gravity vector"
// based on how fast the system is accelerating.
double CalcSpecificForce(const Vec3 *Ai, const Vec3 *acc, const Vec3 *g_g,
                         double acc_window, Vec3 *Fg, Vec3 *Fb) {
  double g = Vec3Norm(g_g);
  double acc_norm = Vec3Norm(acc);
  Vec3Sub(Ai, g_g, Fg);
  *Fb = *acc;
  return (acc_norm > (g - acc_window) && acc_norm < (g + acc_window))
      * (1.0 - fabs(acc_norm - g)/acc_window);
}


// Single-pole low pass filter
//
// Inputs:
//   u  -- Input value
//   y  -- Last output value
//   fc -- Cut-off frequency in Hz
//   ts -- Sample time
double LPF(double u, double y, double fc, double ts, double *y_z1) {
  double y_out = y + 2.0*PI*ts*fc*(u - y);
  *y_z1 = y_out;
  return y_out;
}


const Vec3 *LPFVec3(const Vec3 *u, const Vec3 *y, double fc, double ts,
                    Vec3 *y_out, Vec3 *y_z1) {
  y_out->x = LPF(u->x, y->x, fc, ts, &y_z1->x);
  y_out->y = LPF(u->y, y->y, fc, ts, &y_z1->y);
  y_out->z = LPF(u->z, y->z, fc, ts, &y_z1->z);
  return y_out;
}
