/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * out_of_tree_controller.c - App layer application of an out of tree controller.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "math3d.h"
#include "controller_lee.h"
#include "platform_defaults.h"
#include "physicalConstants.h"

static inline struct mat33 mlog(struct mat33 R) {
	float acosinput = (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1) / 2.0f;
	if (acosinput >= 1) {
		return mzero();
	} else if (acosinput <= -1) {
		struct vec omg;
		if (!(fabsf(1.0f + R.m[2][2]) < 1e-6f)) {
			omg = vdiv(mkvec(R.m[0][2], R.m[1][2], 1.0f + R.m[2][2]), sqrtf(2.0f * (1.0f + R.m[2][2])));
		} else if (!(fabsf(1.0f + R.m[1][1]) < 1e-6f)) {
			omg = vdiv(mkvec(R.m[0][1], 1.0f + R.m[1][1], R.m[2][1]), sqrtf(2.0f * (1.0f + R.m[1][1])));
		} else {
			omg = vdiv(mkvec(1.0f + R.m[0][0], R.m[1][0], R.m[2][0]), sqrtf(2.0f * (1.0f + R.m[0][0])));
		}
		return mcrossmat(vscl(M_PI_F, omg));
	} else {
		float theta = acosf(acosinput);
		return mscl(theta / 2.0f / sinf(theta), msub(R, mtranspose(R)));
	}
}

static inline struct vec mvee(struct mat33 R) {
	return mkvec(R.m[2][1], R.m[0][2], R.m[1][0]);
}

static controllerLee_t g_self2 = {
  .mass = CF_MASS,

  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {7.0, 7.0, 7.0}, // Kp in paper
  .Kpos_P_limit = 100,
  .Kpos_D = {4.0, 4.0, 4.0}, // Kv in paper
  .Kpos_D_limit = 100,
  .Kpos_I = {0.0, 0.0, 0.0}, // not in paper
  .Kpos_I_limit = 2,

  // Attitude PID
  .KR = {0.007, 0.007, 0.008},
  .Komega = {0.00115, 0.00115, 0.002},
  .KI = {0.03, 0.03, 0.03},

  // New stuff
  .R_des.m = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
  },
  .omega_r = {0.0f, 0.0f, 0.0f}
};

void controllerOutOfTreeInit() {
  controllerLeeInit(&g_self2);
}

bool controllerOutOfTreeTest() {
  return controllerLeeFirmwareTest();
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  
  controllerLee_t self = g_self2;
  float dt = (float)(1.0f/ATTITUDE_RATE);
  
  struct vec x = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct mat33 R = quat2rotmat(mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w));
  struct vec W = mkvec(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z);

  struct vec x_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
  struct vec b1_d = mcolumn(maxisangle(vbasis(2), radians(setpoint->attitude.yaw)), 0);

  struct vec ex = vsub(x, x_d);
  struct vec ev = vsub(v, v_d);
  
  struct vec F_d = vadd4(
    vneg(veltmul(self.Kpos_P, ex)),
    vneg(veltmul(self.Kpos_D, ev)),
    vscl(self.mass, a_d),
    vscl(self.mass*GRAVITY_MAGNITUDE, vbasis(2)));
  struct vec b3_d = vnormalize(F_d);
  struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
  struct mat33 R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  float f = 0.0f;
  struct vec M = vzero();

  if (vneq(mcolumn(self.R_des, 0), vbasis(0)) && veq(mcolumn(self.R_des, 1), vbasis(1)) && veq(mcolumn(self.R_des, 2), vbasis(2))) {
    struct vec W_d = mvee(mscl(1.0f/dt, mlog(mmul(mtranspose(self.R_des), R_d))));
    if (vneq(self.omega_r, vzero())) {
      struct vec W_d_dot = vdiv(vsub(W_d, self.omega_r), dt);

      struct vec eR = vscl(0.5f, mvee(msub(
        mmul(mtranspose(R_d), R),
        mmul(mtranspose(R), R_d))));
      struct vec eW = vsub(W, mvmul(mtranspose(R), mvmul(R_d, W_d)));

      f = vdot(F_d, mvmul(R, vbasis(2)));
      M = vadd4(
        vneg(veltmul(self.KR, eR)),
        vneg(veltmul(self.Komega, eW)),
        vcross(W, veltmul(self.J, W)),
        vneg(veltmul(self.J, vsub(
          vcross(W, mvmul(mtranspose(R), mvmul(R_d, W_d))),
          mvmul(mtranspose(R), mvmul(R_d, W_d_dot))))));
    }
    self.omega_r = W_d;
  }  
  self.R_des = R_d;

  control->controlMode = controlModeForceTorque;
  control->thrustSi = f;
  control->torque[0] = M.x;
  control->torque[1] = M.y;
  control->torque[2] = M.z;
}
