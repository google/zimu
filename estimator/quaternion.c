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
// The quaternion operations defined here work for quaternions expressed as
//     q = [q0 q_bar]^T,
// where q0 is the scalar part and
// q_bar = q1*i + q2*j + q3*k is the three-vector part of the quaternion.
#include "quaternion.h"
#include <math.h>
#include "linalg.h"

const Quat *QuatMultiply(const Quat *q_in, const Quat *r_in, Quat *q_out) {
  Quat q = *q_in, r = *r_in;
  q_out->q0 = q.q0*r.q0 - q.q1*r.q1 - q.q2*r.q2 - q.q3*r.q3;
  q_out->q1 = q.q0*r.q1 + q.q1*r.q0 + q.q2*r.q3 - q.q3*r.q2;
  q_out->q2 = q.q0*r.q2 + q.q2*r.q0 + q.q3*r.q1 - q.q1*r.q3;
  q_out->q3 = q.q0*r.q3 + q.q3*r.q0 + q.q1*r.q2 - q.q2*r.q1;
  return q_out;
}

const Vec3 *QuatRotate(const Quat *q, const Vec3 *v_in, Vec3 *v_out) {
  Mat3 dcm;
  QuatToDCM(q, &dcm);
  Mat3Vec3Mult(&dcm, v_in, v_out);
  return v_out;
}

const Mat3 *QuatToDCM(const Quat *q, Mat3 *dcm) {
  Quat q_norm;
  QuatNormalize(q, &q_norm);

  dcm->d[0][0] = q_norm.q0*q_norm.q0 + q_norm.q1*q_norm.q1
      - q_norm.q2*q_norm.q2 - q_norm.q3*q_norm.q3;
  dcm->d[0][1] = 2.0*(q_norm.q1*q_norm.q2 + q_norm.q0*q_norm.q3);
  dcm->d[0][2] = 2.0*(q_norm.q1*q_norm.q3 - q_norm.q0*q_norm.q2);
  dcm->d[1][0] = 2.0*(q_norm.q1*q_norm.q2 - q_norm.q0*q_norm.q3);
  dcm->d[1][1] = q_norm.q0*q_norm.q0 - q_norm.q1*q_norm.q1
      + q_norm.q2*q_norm.q2 - q_norm.q3*q_norm.q3;
  dcm->d[1][2] = 2.0*(q_norm.q2*q_norm.q3 + q_norm.q0*q_norm.q1);
  dcm->d[2][0] = 2.0*(q_norm.q1*q_norm.q3 + q_norm.q0*q_norm.q2);
  dcm->d[2][1] = 2.0*(q_norm.q2*q_norm.q3 - q_norm.q0*q_norm.q1);
  dcm->d[2][2] = q_norm.q0*q_norm.q0 - q_norm.q1*q_norm.q1
      - q_norm.q2*q_norm.q2 + q_norm.q3*q_norm.q3;

  return dcm;
}

const Quat *QuatNormalize(const Quat *q_in, Quat *q_out) {
  double quat_mod = fmax(QuatMod(q_in), 1.0e-9);
  return QuatScale(q_in, 1.0/quat_mod, q_out);
}

const Quat *QuatScale(const Quat *q_in, double scale, Quat *q_out) {
  q_out->q0 = scale * q_in->q0;
  q_out->q1 = scale * q_in->q1;
  q_out->q2 = scale * q_in->q2;
  q_out->q3 = scale * q_in->q3;
  return q_out;
}

const Quat *QuatLinComb(double cq, const Quat *q, double cr, const Quat *r,
                        Quat *q_out) {
  q_out->q0 = cq*q->q0 + cr*r->q0;
  q_out->q1 = cq*q->q1 + cr*r->q1;
  q_out->q2 = cq*q->q2 + cr*r->q2;
  q_out->q3 = cq*q->q3 + cr*r->q3;
  return q_out;
}

double QuatNorm(const Quat *q) {
  return q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3;
}

double QuatMod(const Quat *q) {
  return sqrt(QuatNorm(q));
}
