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
#include "linalg.h"

#include <assert.h>
#include <math.h>


const Vec3 Vec3_0 = {0.0, 0.0, 0.0};
const Vec3 Vec3_1 = {1.0, 1.0, 1.0};
const Mat3 Mat3_0 = {{{0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0}}};
const Mat3 Mat3_I = {{{1.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0},
                      {0.0, 0.0, 1.0}}};

// Vec3 -- operations on vectors with three elements

// v_out = a*v0 + v1
const Vec3 *Vec3Axpy(double a, const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  v_out->x = a * v0->x + v1->x;
  v_out->y = a * v0->y + v1->y;
  v_out->z = a * v0->z + v1->z;
  return v_out;
}

// v_out = a + v0^T * v1
double Vec3Sdot(double a, const Vec3 *v0, const Vec3 *v1) {
  return a + v0->x*v1->x + v0->y*v1->y + v0->z*v1->z;
}

// v_out = a*v0_x*v1_x + a*v0_y*v1_y + a*v0_z*v1_z
const Vec3 *Vec3Amul(double a, const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  v_out->x = a * v0->x*v1->x;
  v_out->y = a * v0->y*v1->y;
  v_out->z = a * v0->z*v1->z;
  return v_out;
}

// Scale all elements of a vector by a scalar.
const Vec3 *Vec3Scale(const Vec3 *v_in, double scale, Vec3 *v_out) {
  return Vec3Axpy(scale, v_in, &Vec3_0, v_out);
}

// v_out = v0 + v1
const Vec3 *Vec3Add(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  return Vec3Axpy(1.0, v0, v1, v_out);
}

// v_out = v0 + v1 + v2
const Vec3 *Vec3Add3(const Vec3 *v0, const Vec3 *v1, const Vec3 *v2,
                     Vec3 *v_out) {
  Vec3 vtmp;
  Vec3Axpy(1.0, v0, v1, &vtmp);
  return Vec3Axpy(1.0, &vtmp, v2, v_out);
}

// v_out = v0 - v1
const Vec3 *Vec3Sub(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  return Vec3Axpy(-1.0, v1, v0, v_out);
}

// v_out = c0 * v0 + c1 * v1
const Vec3 *Vec3LinComb(double c0, const Vec3 *v0, double c1, const Vec3 *v1,
                        Vec3 *v_out) {
  Vec3 vtmp;
  Vec3Axpy(c0, v0, &Vec3_0, &vtmp);
  return Vec3Axpy(c1, v1, &vtmp, v_out);
}

// v_out = c0 * v0 + c1 * v1 + c2 * v2
const Vec3 *Vec3LinComb3(double c0, const Vec3 *v0,
                         double c1, const Vec3 *v1,
                         double c2, const Vec3 *v2, Vec3 *v_out) {
  Vec3 vtmp;
  Vec3Axpy(c0, v0, &Vec3_0, &vtmp);
  Vec3Axpy(c1, v1, &vtmp, &vtmp);
  return Vec3Axpy(c2, v2, &vtmp, v_out);
}

const Vec3 *Vec3Mult(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  return Vec3Amul(1.0, v0, v1, v_out);
}

// Cross-product of two vectors
const Vec3 *Vec3Cross(const Vec3 *v0, const Vec3 *v1, Vec3 *v_out) {
  Vec3 vtmp = {v0->y*v1->z - v0->z*v1->y,
               v0->z*v1->x - v0->x*v1->z,
               v0->x*v1->y - v0->y*v1->x};
  *v_out = vtmp;
  return v_out;
}

double Vec3Norm(const Vec3 *v) {
  return sqrt(Vec3NormSquared(v));
}

double Vec3NormBound(const Vec3 *v, double low) {
  return fmax(low, Vec3Norm(v));
}

double Vec3NormSquared(const Vec3 *v) {
  return Vec3Sdot(0.0, v, v);
}


// Mat3 -- operations on 3x3 matrices

// v_out = m*v0 + a*v1
const Vec3 *Mat3Vec3Axpby(const Mat3 *m, TransposeType t, const Vec3 *v0,
                          double a, const Vec3 *v1, Vec3 *v_out) {
  Vec3 vtmp;
  if (t == TRANS) {
    vtmp.x = m->d[0][0] * v0->x + m->d[1][0] * v0->y + m->d[2][0] * v0->z;
    vtmp.y = m->d[0][1] * v0->x + m->d[1][1] * v0->y + m->d[2][1] * v0->z;
    vtmp.z = m->d[0][2] * v0->x + m->d[1][2] * v0->y + m->d[2][2] * v0->z;
  } else {
    vtmp.x = m->d[0][0] * v0->x + m->d[0][1] * v0->y + m->d[0][2] * v0->z;
    vtmp.y = m->d[1][0] * v0->x + m->d[1][1] * v0->y + m->d[1][2] * v0->z;
    vtmp.z = m->d[2][0] * v0->x + m->d[2][1] * v0->y + m->d[2][2] * v0->z;
  }

  v_out->x = vtmp.x + a*v1->x;
  v_out->y = vtmp.y + a*v1->y;
  v_out->z = vtmp.z + a*v1->z;
  return v_out;
}

const Vec3 *Mat3Vec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out) {
  return Mat3Vec3Axpby(m, NO_TRANS, v_in, 0.0, &Vec3_0, v_out);
}

const Vec3 *Mat3TransVec3Mult(const Mat3 *m, const Vec3 *v_in, Vec3 *v_out) {
  return Mat3Vec3Axpby(m, TRANS, v_in, 0.0, &Vec3_0, v_out);
}
