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
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

#include "controller.h"
#include "math3d.h"
#include "controller_lee.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "commander.h"
#include "power_distribution.h"
#include "crtp_commander_high_level.h"
#include "param.h"
#include "log.h"

void setDesiredAttitude(float thrust_g, float roll, float pitch, float yaw) {
  float f = thrust_g*GRAVITY_MAGNITUDE;
  float thrust = f / powerDistributionGetMaxThrust() * UINT16_MAX;

  setpoint_t setpoint;
  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.mode.z = modeVelocity;

  setpoint.thrust = thrust;
  setpoint.attitude.roll = roll;
  setpoint.attitude.pitch = pitch;
  setpoint.attitude.yaw = yaw;

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

void appMain() {
  while (1) {
    vTaskDelay(M2T(2000));
  }
}

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
  .mass = CF_MASS, // kg
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  .Kpos_P = {7.0, 7.0, 7.0},
  .Kpos_D = {4.0, 4.0, 4.0}, // 6 4 works better than 7 4 and 7 5 and 8 5

  .KR = {0.007, 0.007, 0.008},
  .Komega = {0.00115, 0.00115, 0.002},
};

void controllerLee2Init(controllerLee_t* self);
void controllerLee2(controllerLee_t* self, control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);

void controllerOutOfTreeInit() {
  controllerLee2Init(&g_self2);
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerLee2(&g_self2, control, setpoint, sensors, state, tick);
}

// Inner functions
void controllerLee2Init(controllerLee_t* self) {  
  self->Kpos_I_limit = 0;
  self->Kpos_I = vzero();
  
  self->R_des = mcolumns(vrepeat(NAN), vrepeat(NAN), vrepeat(NAN));
  self->omega_r = vrepeat(NAN);

  self->rpy = vzero();
  self->rpy_des = vzero();
}

void controllerLee2(controllerLee_t* self, control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  
  float dt = (float)(1.0f/ATTITUDE_RATE);

  // States
  struct vec x = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  self->rpy = quat2rpy(q);
  struct vec W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));

  float desiredYaw = 0;
  // if (setpoint->mode.yaw == modeVelocity) {
  //   desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  // } else if (setpoint->mode.yaw == modeAbs) {
  //   desiredYaw = radians(setpoint->attitude.yaw);
  // } else if (setpoint->mode.quat == modeAbs) {
  //   struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
  //   desiredYaw = quat2rpy(setpoint_quat).z;
  // }
  
  // Calculate f and R_d
  float f;
  struct mat33 R_d;
  if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs || setpoint->mode.z == modeAbs) {
    struct vec x_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);

    struct vec ex = vsub(x, x_d);
    struct vec ev = vsub(v, v_d);
    
    struct vec F_d = vscl(self->mass, vadd4(
      vneg(veltmul(self->Kpos_P, ex)),
      vneg(veltmul(self->Kpos_D, ev)),
      a_d,
      vscl(GRAVITY_MAGNITUDE, vbasis(2))));
    f = vdot(F_d, mvmul(R, vbasis(2)));

    struct vec b3_d = vnormalize(F_d);
    struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
    R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  } else {
    if (setpoint->mode.z == modeDisable && setpoint->thrust < 1000) {
      control->controlMode = controlModeForceTorque;
      control->thrustSi  = 0;
      control->torque[0] = 0;
      control->torque[1] = 0;
      control->torque[2] = 0;
      return;
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    f = setpoint->thrust / UINT16_MAX * max_thrust;

    R_d = quat2rotmat(rpy2quat(mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw)));
  }

  // Calculate M
  struct vec M = vzero();

  if (vneq(mcolumn(self->R_des, 0), vrepeat(NAN)) && vneq(mcolumn(self->R_des, 1), vrepeat(NAN)) && vneq(mcolumn(self->R_des, 2), vrepeat(NAN))) {
    struct vec W_d = mvee(mscl(1.0f/dt, mlog(mmul(mtranspose(self->R_des), R_d))));
    if (vneq(self->omega_r, vrepeat(NAN))) {
      struct vec W_d_dot = vdiv(vsub(W_d, self->omega_r), dt);

      struct vec eR = vscl(0.5f, mvee(msub(
        mmul(mtranspose(R_d), R),
        mmul(mtranspose(R), R_d))));
      struct vec eW = vsub(W, mvmul(mtranspose(R), mvmul(R_d, W_d)));

      self->rpy_des = eW;

      M = vadd4(
        vneg(veltmul(self->KR, eR)),
        vneg(veltmul(self->Komega, eW)),
        vcross(W, veltmul(self->J, W)),
        vneg(veltmul(self->J, vsub(
          vcross(W, mvmul(mtranspose(R), mvmul(R_d, W_d))),
          mvmul(mtranspose(R), mvmul(R_d, W_d_dot))))));
    }
    self->omega_r = W_d;
  }  
  self->R_des = R_d;
  self->rpy = mcolumn(self->R_des, 0);

  control->controlMode = controlModeForceTorque;
  control->thrustSi = f;
  control->torque[0] = M.x;
  control->torque[1] = M.y;
  control->torque[2] = M.z;

  self->Kpos_I_limit = f;
  self->Kpos_I.x = M.x;
  self->Kpos_I.y = M.y;
  self->Kpos_I.z = M.z;
}

PARAM_GROUP_START(ctrlLee2)

PARAM_ADD(PARAM_FLOAT, kx_x, &g_self2.Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, kx_y, &g_self2.Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, kx_z, &g_self2.Kpos_P.z)

PARAM_ADD(PARAM_FLOAT, kv_x, &g_self2.Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, kv_y, &g_self2.Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, kv_z, &g_self2.Kpos_D.z)

PARAM_ADD(PARAM_FLOAT, kR_x, &g_self2.KR.x)
PARAM_ADD(PARAM_FLOAT, kR_y, &g_self2.KR.y)
PARAM_ADD(PARAM_FLOAT, kR_z, &g_self2.KR.z)

PARAM_ADD(PARAM_FLOAT, kW_x, &g_self2.Komega.x)
PARAM_ADD(PARAM_FLOAT, kW_y, &g_self2.Komega.y)
PARAM_ADD(PARAM_FLOAT, kW_z, &g_self2.Komega.z)

PARAM_GROUP_STOP(ctrlLee2)

LOG_GROUP_START(ctrlLee2)

LOG_ADD(LOG_FLOAT, f, &g_self2.Kpos_I_limit)
LOG_ADD(LOG_FLOAT, M1, &g_self2.Kpos_I.x)
LOG_ADD(LOG_FLOAT, M2, &g_self2.Kpos_I.y)
LOG_ADD(LOG_FLOAT, M3, &g_self2.Kpos_I.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self2.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self2.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self2.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self2.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self2.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self2.rpy_des.z)

LOG_GROUP_STOP(ctrlLee2)