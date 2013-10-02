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
#ifndef LINALG_H_
#define LINALG_H_

typedef enum {TRANS, NO_TRANS} TransposeType;

typedef struct Vec3 {
  double x, y, z;
} Vec3;

extern const Vec3 Vec3_0;
extern const Vec3 Vec3_1;

const Vec3 *Vec3Axpy(double a, const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
double Vec3Sdot(double a, const Vec3 *v0, const Vec3 *v1);
const Vec3 *Vec3Amul(double a, const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Scale(const Vec3 *v_in, double scale, Vec3 *v_out);
const Vec3 *Vec3Add(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Add3(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2,
                     Vec3 *v_out);
const Vec3 *Vec3Sub(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3LinComb(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                        Vec3 *v_out);
const Vec3 *Vec3LinComb3(double c0, const Vec3 *v0,
                         double c1, const Vec3 *v1,
                         double c2, const Vec3 *v2, Vec3 *v_out);
const Vec3 *Vec3Mult(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
const Vec3 *Vec3Cross(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out);
double Vec3Norm(const Vec3 *v);
double Vec3NormSquared(const Vec3 *v);
double Vec3NormBound(const Vec3 *v, double low);


typedef struct Mat3 {
  double d[3][3];
} Mat3;

extern const Mat3 Mat3_0;
extern const Mat3 Mat3_I;

const Vec3 *Mat3Vec3Axpby(const Mat3 *m, TransposeType t,
                          const Vec3 *v0, double a, const Vec3 *v1,
                          Vec3 *v_out);
const Vec3 *Mat3Vec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out);
const Vec3 *Mat3TransVec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out);

#endif  // LINALG_H_
