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
#ifndef ZIMU_H_
#define ZIMU_H_

#include <math.h>
#include <stdint.h>

#include "linalg.h"

#define PI 3.14159265358979323846

typedef enum {
  ZIMU_INIT_START = 0,
  ZIMU_INIT_DONE = 1
} ZIMUInitState;

typedef struct {
  Vec3 g_g, mag_g;
} PhysParams;

typedef struct {
  double ts;
  PhysParams phys;
} SystemParams;


extern int32_t zimu_init;
extern const SystemParams *g_sys;

double Saturate(double x, double low, double high);

#endif  // ZIMU_H_
