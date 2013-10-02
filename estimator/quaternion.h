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
#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "linalg.h"

typedef struct {
  double q0, q1, q2, q3;
} Quat;

const Quat *QuatMultiply(const Quat *q, const Quat *r, Quat *q_out);
const Vec3 *QuatRotate(const Quat *q, const Vec3 *v_in, Vec3 *v_out);
const Mat3 *QuatToDCM(const Quat *q, Mat3 *dcm);
const Quat *QuatNormalize(const Quat *q_in, Quat *q_out);
const Quat *QuatScale(const Quat *q_in, double scale, Quat *q_out);
const Quat *QuatLinComb(double cq, const Quat *q, double cr, const Quat *r,
                        Quat *q_out);
double QuatNorm(const Quat *q);
double QuatMod(const Quat *q);

#endif  // QUATERNION_H_
