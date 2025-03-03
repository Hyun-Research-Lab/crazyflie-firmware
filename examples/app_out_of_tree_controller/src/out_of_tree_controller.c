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
#include <stdlib.h>
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
#include "radiolink.h"
#include "param.h"
#include "log.h"

#define FILTER_SIZE 50
#define NETWORK_RATE RATE_100_HZ

uint8_t disable_props = 0;
uint8_t enable_filters = 0;

float t = 0;

typedef struct controllerLee2_s {
  // Quadrotor parameters
  float m;
  struct vec J; // Inertia matrix (diagonal matrix); kg m^2

  // Gains
  float kx;
  float kv;
  float ki;
  float c1;
  float sigma;

  float kR;
  float kW;
  float kI;
  float c2;

  // Errors
  struct vec ex;
  struct vec ev;
  struct vec ei;

  struct vec eR;
  struct vec eW;
  struct vec eI;

  // Wrench
  float f;
  struct vec M;

  // Previous values
  struct mat33 R_d_prev;
  struct vec W_d_prev;

  struct vec W_d_raw[FILTER_SIZE];
  struct vec W_d_dot_raw[FILTER_SIZE];

  struct vec W_d;
  struct vec W_d_dot;

  // For leader-follower
  uint8_t node;
  uint8_t parent;
  
  struct vec F_d_bar;
  struct vec x;
  struct vec v;
  struct mat33 R;

  float kx_lf;
  float kv_lf;
  float ki_lf;
  float sigma_lf;

  struct vec ex_lf;
  struct vec ev_lf;
  struct vec ei_lf;

  float test_f;
} controllerLee2_t;

static controllerLee2_t g_self2 = {
  .node = 0, // 0 is leader, 1, 2, 3, ... are followers
  .parent = 0,
  
  .m = 0.036, // kg
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  .kx = 7.0,
  .kv = 4.0,
  .ki = 1.0,
  .c1 = 3.6,
  .sigma = 1.0,

  .kR = 0.007,
  .kW = 0.002,
  .kI = 0.0005,
  .c2 = 0.8,

  .kx_lf = 10.0,
  .kv_lf = 10.0,
  .ki_lf = 4.0,
  .sigma_lf = 1.0,
};

static inline struct mat33 vouter(struct vec a, struct vec b) {
  struct mat33 out;
  out.m[0][0] = a.x * b.x;
  out.m[0][1] = a.x * b.y;
  out.m[0][2] = a.x * b.z;
  out.m[1][0] = a.y * b.x;
  out.m[1][1] = a.y * b.y;
  out.m[1][2] = a.y * b.z;
  out.m[2][0] = a.z * b.x;
  out.m[2][1] = a.z * b.y;
  out.m[2][2] = a.z * b.z;
  return out;
}

static inline struct vec vclampscl2(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void p2pCB(P2PPacket* packet) {
  controllerLee2_t* self = &g_self2;

  // Copy time from leader
  if (packet->port == 0) {
    memcpy(&t, packet->data, sizeof(float));
  }
  
  // Only process packets from the parent
  if (packet->port != self->parent) {
    return;
  }
  
  float m_l;
  struct vec F_d_l_bar;
  struct vec x_l;
  struct vec v_l;

  // Get parent information
  memcpy(&m_l,         packet->data + sizeof(float),    sizeof(float));
  memcpy(&F_d_l_bar.x, packet->data + 2*sizeof(float),  sizeof(float));
  memcpy(&F_d_l_bar.y, packet->data + 3*sizeof(float),  sizeof(float));
  memcpy(&F_d_l_bar.z, packet->data + 4*sizeof(float),  sizeof(float));
  memcpy(&x_l.x,       packet->data + 5*sizeof(float),  sizeof(float));
  memcpy(&x_l.y,       packet->data + 6*sizeof(float),  sizeof(float));
  memcpy(&x_l.z,       packet->data + 7*sizeof(float),  sizeof(float));
  memcpy(&v_l.x,       packet->data + 8*sizeof(float),  sizeof(float));
  memcpy(&v_l.y,       packet->data + 9*sizeof(float),  sizeof(float));
  memcpy(&v_l.z,       packet->data + 10*sizeof(float), sizeof(float));

  if (vmag(F_d_l_bar) < 1e-6f) {
    self->ei = vzero();
    return;
  }

  // Where the magic happens
  struct vec re = vsub(self->x, x_l);
  struct vec re_dot = vsub(self->v, v_l);

  float theta =      M_PI_F/8.0f * cosf(M_PI_F/2.0f*t);
  float theta_dot =  M_PI_F/8.0f * -M_PI_F/2.0f*sinf(M_PI_F/2.0f*t);
  float theta_ddot = M_PI_F/8.0f * -M_PI_F/2.0f*M_PI_F/2.0f*cosf(M_PI_F/2.0f*t);
  
  // Desired values
  struct vec re_d;
  struct vec re_d_dot;
  struct vec re_d_ddot;
  struct vec b1_d;
  if (self->node == 1) {
    // re_d = vscl(vmag(re), vnormalize(mkvec(1, 0, 0)));
    // re_d_dot = vzero();
    // re_d_ddot = vzero();
    re_d =      vscl(vmag(re),                          mkvec(cosf(theta),  0, sinf(theta)));
    re_d_dot =  vscl(vmag(re)*theta_dot,                mkvec(-sinf(theta), 0, cosf(theta)));
    re_d_ddot = vadd(vscl(vmag(re)*theta_dot*theta_dot, mkvec(-cosf(theta), 0, -sinf(theta))),
                              vscl(vmag(re)*theta_ddot, mkvec(-sinf(theta), 0, cosf(theta))));
    b1_d = vbasis(0);
  } else if (self->node == 2) {
    // re_d = vscl(vmag(re), vnormalize(mkvec(-1, 0, 0)));
    // re_d_dot = vzero();
    // re_d_ddot = vzero();
    re_d =      vscl(vmag(re),                          mkvec(-cosf(theta), 0, sinf(theta)));
    re_d_dot =  vscl(vmag(re)*theta_dot,                mkvec(sinf(theta),  0, cosf(theta)));
    re_d_ddot = vadd(vscl(vmag(re)*theta_dot*theta_dot, mkvec(cosf(theta),  0, -sinf(theta))),
                              vscl(vmag(re)*theta_ddot, mkvec(sinf(theta),  0, cosf(theta))));
    b1_d = vbasis(0);
  }

  self->ex_lf = vsub(re, re_d);
  self->ev_lf = vsub(re_dot, re_d_dot);
  self->ei_lf = vadd(self->ei_lf, vscl(1.0f/NETWORK_RATE, self->ex_lf));
  self->ei_lf = vclampscl2(self->ei_lf, -self->sigma_lf, self->sigma_lf);

  struct mat33 P = msub(meye(), mscl(1.0f/vmag2(re), vouter(re, re)));
  struct vec u = mvmul(P, vadd4(
    vscl(-self->kx_lf, self->ex_lf),
    vscl(-self->kv_lf, self->ev_lf),
    vscl(-self->ki_lf, self->ei_lf),
    re_d_ddot));

  struct vec F_d_bar = vscl(self->m, vadd(vdiv(F_d_l_bar, m_l), u));
  struct vec F_d = vadd(F_d_bar, vscl(self->m*GRAVITY_MAGNITUDE, vbasis(2)));
  // self->F_d = vscl(self->m, u);

  // Send F_d to the controller
  float f = vdot(F_d, mvmul(self->R, vbasis(2)));
  self->test_f = vmag(re);
  float thrust = f * UINT16_MAX / powerDistributionGetMaxThrust();

  struct vec b3_d = vnormalize(F_d);
  struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
  struct mat33 R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  struct vec rpy_d = quat2rpy(mat2quat(R_d));
  float roll = degrees(rpy_d.x);
  float pitch = -degrees(rpy_d.y);
  float yaw = degrees(rpy_d.z);
  
  setpoint_t setpoint;
  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.mode.z = modeVelocity;
  setpoint.mode.yaw = modeAbs;

  setpoint.thrust = thrust;
  setpoint.attitude.roll = roll;
  setpoint.attitude.pitch = pitch;
  setpoint.attitude.yaw = yaw;

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

void appMain() {
  controllerLee2_t* self = &g_self2;

  if (self->node > 0) {
    p2pRegisterCB(p2pCB);
  }
  
  P2PPacket packet;
  packet.port = self->node;
  packet.size = 11*sizeof(float);
    
  while (1) {
    vTaskDelay(M2T(1000/NETWORK_RATE));
    if (self->node == 0) {
      t += 1.0f/NETWORK_RATE;
    }

    memcpy(packet.data,                    &t,               sizeof(float));
    memcpy(packet.data + sizeof(float),    &self->m,         sizeof(float));
    memcpy(packet.data + 2*sizeof(float),  &self->F_d_bar.x, sizeof(float));
    memcpy(packet.data + 3*sizeof(float),  &self->F_d_bar.y, sizeof(float));
    memcpy(packet.data + 4*sizeof(float),  &self->F_d_bar.z, sizeof(float));
    memcpy(packet.data + 5*sizeof(float),  &self->x.x,       sizeof(float));
    memcpy(packet.data + 6*sizeof(float),  &self->x.y,       sizeof(float));
    memcpy(packet.data + 7*sizeof(float),  &self->x.z,       sizeof(float));
    memcpy(packet.data + 8*sizeof(float),  &self->v.x,       sizeof(float));
    memcpy(packet.data + 9*sizeof(float),  &self->v.y,       sizeof(float));
    memcpy(packet.data + 10*sizeof(float), &self->v.z,       sizeof(float));

    radiolinkSendP2PPacketBroadcast(&packet);
  }
}

static inline struct mat33 mlog(struct mat33 R) {
	float acosinput = (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0f) / 2.0f;
	if (acosinput >= 1.0f) {
		return mzero();
	} else if (acosinput <= -1.0f) {
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

// static inline int compare(const void* a, const void* b) {
//    return (*(int*)a - *(int*)b);
// }

static inline struct vec filter(struct vec* arr, int size) {
  // float x[size];
  // float y[size];
  // float z[size];
  // for (int i = 0; i < size; i++) {
  //   x[i] = arr[i].x;
  //   y[i] = arr[i].y;
  //   z[i] = arr[i].z;
  // }

  // qsort(x, size, sizeof(float), compare);
  // qsort(y, size, sizeof(float), compare);
  // qsort(z, size, sizeof(float), compare);

  // return mkvec(x[size/2], y[size/2], z[size/2]);

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  for (int i = 0; i < size; i++) {
    x += arr[i].x;
    y += arr[i].y;
    z += arr[i].z;
  }
  x /= size;
  y /= size;
  z /= size;

  return mkvec(x, y, z);
}

static inline void resetFilterBuffers(controllerLee2_t* self) {
  for (int i = 0; i < FILTER_SIZE; i++) {
    self->W_d_raw[i] = vzero();
    self->W_d_dot_raw[i] = vzero();
  }
}

void controllerOutOfTreeInit() {
  controllerLee2_t* self = &g_self2;

  self->f = 0;
  self->M = vzero();
  
  self->ex = vzero();
  self->ev = vzero();
  self->ei = vzero();

  self->eR = vzero();
  self->eW = vzero();
  self->eI = vzero();

  self->R_d_prev = mcolumns(vrepeat(NAN), vrepeat(NAN), vrepeat(NAN));
  self->W_d_prev = vrepeat(NAN);

  self->W_d = vzero();
  self->W_d_dot = vzero();

  resetFilterBuffers(self);

  self->F_d_bar = vzero();
  self->x = vzero();
  self->v = vzero();
  self->R = meye();

  self->ex_lf = vzero();
  self->ev_lf = vzero();
  self->ei_lf = vzero();

  self->test_f = 0;
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerLee2_t* self = &g_self2;
  
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  
  float dt = (float)(1.0f/ATTITUDE_RATE);

  // States
  struct vec x = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  // self->rpy = quat2rpy(q);
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
  if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  }

  self->x = x;
  self->v = v;
  self->R = R;
  
  // Calculate f and R_d
  struct mat33 R_d;
  if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs || setpoint->mode.z == modeAbs) {
    struct vec x_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);

    self->ex = vsub(x, x_d);
    self->ev = vsub(v, v_d);
    self->ei = vadd(self->ei, vscl(dt, vadd(self->ev, vscl(self->c1, self->ex))));
    self->ei = vclampscl2(self->ei, -self->sigma, self->sigma);
    
    self->F_d_bar = vscl(self->m, vadd4(
      vscl(-self->kx, self->ex),
      vscl(-self->kv, self->ev),
      vscl(-self->ki, self->ei),
      a_d));
    self->F_d_bar = veltmul(mkvec(0.1f, 0.1f, 1.0f), self->F_d_bar);
    struct vec F_d = vadd(self->F_d_bar, vscl(self->m*GRAVITY_MAGNITUDE, vbasis(2)));
    self->f = vdot(F_d, mvmul(R, vbasis(2)));
    
    if (self->f < 0.01f) {
      self->ei = vzero();
      self->eI = vzero();
      resetFilterBuffers(self);
    }

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
      self->ei = vzero();
      self->eI = vzero();
      resetFilterBuffers(self);
      return;
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    self->f = setpoint->thrust / UINT16_MAX * max_thrust;

    R_d = quat2rotmat(rpy2quat(mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw)));

    struct vec b3 = mcolumn(R, 2);
    struct vec b3_d = mcolumn(R_d, 2);
    self->F_d_bar = vscl(self->f/vdot(b3_d, b3), b3_d);
  }

  // Calculate M
  if (vneq(mcolumn(self->R_d_prev, 0), vrepeat(NAN)) && vneq(mcolumn(self->R_d_prev, 1), vrepeat(NAN)) && vneq(mcolumn(self->R_d_prev, 2), vrepeat(NAN))) {
    if (enable_filters) {
      // Update W_d_raw buffer
      for (int i = 0; i < FILTER_SIZE - 1; i++) {
        self->W_d_raw[i] = self->W_d_raw[i + 1];
      }
      self->W_d_raw[FILTER_SIZE - 1] = vdiv(mvee(mlog(mmul(mtranspose(self->R_d_prev), R_d))), dt);
      self->W_d = filter(self->W_d_raw, FILTER_SIZE);
    } else {
      self->W_d = vzero();
    }

    if (vneq(self->W_d_prev, vrepeat(NAN))) {
      if (enable_filters) {
        // Update W_d_dot_raw buffer
        for (int i = 0; i < FILTER_SIZE - 1; i++) {
          self->W_d_dot_raw[i] = self->W_d_dot_raw[i + 1];
        }
        self->W_d_dot_raw[FILTER_SIZE - 1] = vdiv(vsub(self->W_d, self->W_d_prev), dt);
        self->W_d_dot = filter(self->W_d_dot_raw, FILTER_SIZE);
      } else {
        self->W_d_dot = vzero();
      }

      self->eR = vscl(0.5f, mvee(msub(
        mmul(mtranspose(R_d), R),
        mmul(mtranspose(R), R_d))));
      self->eW = vsub(W, mvmul(mtranspose(R), mvmul(R_d, self->W_d)));
      self->eI = vadd(self->eI, vscl(dt, vadd(self->eW, vscl(self->c2, self->eR))));

      self->M = vadd(vadd4(
        vneg(vscl(self->kR, self->eR)),
        vneg(vscl(self->kW, self->eW)),
        vneg(vscl(self->kI, self->eI)),
        vcross(mvmul(mtranspose(R), mvmul(R_d, self->W_d)), veltmul(self->J, mvmul(mtranspose(R), mvmul(R_d, self->W_d))))),
        veltmul(self->J, mvmul(mtranspose(R), mvmul(R_d, self->W_d_dot))));
    }
    self->W_d_prev = self->W_d;
  }
  self->R_d_prev = R_d;

  control->controlMode = controlModeForceTorque;
  control->thrustSi = self->f;
  control->torque[0] = self->M.x;
  control->torque[1] = self->M.y;
  control->torque[2] = self->M.z;
  if (disable_props) {
    control->thrustSi = 0.0f;
    control->torque[0] = 0.0f;
    control->torque[1] = 0.0f;
    control->torque[2] = 0.0f;
  }
}

PARAM_GROUP_START(ctrlLee2)

PARAM_ADD(PARAM_UINT8, node, &g_self2.node)
PARAM_ADD(PARAM_UINT8, parent, &g_self2.parent)

PARAM_ADD(PARAM_FLOAT, m, &g_self2.m)

PARAM_ADD(PARAM_FLOAT, kx, &g_self2.kx)
PARAM_ADD(PARAM_FLOAT, kv, &g_self2.kv)
PARAM_ADD(PARAM_FLOAT, kR, &g_self2.kR)
PARAM_ADD(PARAM_FLOAT, kW, &g_self2.kW)
// PARAM_ADD(PARAM_FLOAT, kI, &g_self2.kI)
// PARAM_ADD(PARAM_FLOAT, c2, &g_self2.c2)

PARAM_ADD(PARAM_UINT8, disable_props, &disable_props)

PARAM_ADD(PARAM_FLOAT, kx_lf, &g_self2.kx_lf)
PARAM_ADD(PARAM_FLOAT, kv_lf, &g_self2.kv_lf)
PARAM_ADD(PARAM_FLOAT, ki_lf, &g_self2.ki_lf)

PARAM_GROUP_STOP(ctrlLee2)

LOG_GROUP_START(ctrlLee2)

// Wrench
LOG_ADD(LOG_FLOAT, f, &g_self2.f)
LOG_ADD(LOG_FLOAT, M1, &g_self2.M.x)
LOG_ADD(LOG_FLOAT, M2, &g_self2.M.y)
LOG_ADD(LOG_FLOAT, M3, &g_self2.M.z)

// Errors
LOG_ADD(LOG_FLOAT, ex1, &g_self2.ex.x)
LOG_ADD(LOG_FLOAT, ex2, &g_self2.ex.y)
LOG_ADD(LOG_FLOAT, ex3, &g_self2.ex.z)

LOG_ADD(LOG_FLOAT, ev1, &g_self2.ev.x)
LOG_ADD(LOG_FLOAT, ev2, &g_self2.ev.y)
LOG_ADD(LOG_FLOAT, ev3, &g_self2.ev.z)

LOG_ADD(LOG_FLOAT, eR1, &g_self2.eR.x)
LOG_ADD(LOG_FLOAT, eR2, &g_self2.eR.y)
LOG_ADD(LOG_FLOAT, eR3, &g_self2.eR.z)

LOG_ADD(LOG_FLOAT, eW1, &g_self2.eW.x)
LOG_ADD(LOG_FLOAT, eW2, &g_self2.eW.y)
LOG_ADD(LOG_FLOAT, eW3, &g_self2.eW.z)

// LOG_ADD(LOG_FLOAT, eI1, &g_self2.eI.x)
// LOG_ADD(LOG_FLOAT, eI2, &g_self2.eI.y)
// LOG_ADD(LOG_FLOAT, eI3, &g_self2.eI.z)

LOG_ADD(LOG_FLOAT, W_d1, &g_self2.W_d.x)
LOG_ADD(LOG_FLOAT, W_d2, &g_self2.W_d.y)
LOG_ADD(LOG_FLOAT, W_d3, &g_self2.W_d.z)

LOG_ADD(LOG_FLOAT, W_d_dot1, &g_self2.W_d_dot.x)
LOG_ADD(LOG_FLOAT, W_d_dot2, &g_self2.W_d_dot.y)
LOG_ADD(LOG_FLOAT, W_d_dot3, &g_self2.W_d_dot.z)

LOG_ADD(LOG_FLOAT, F_d1, &g_self2.F_d_bar.x)
LOG_ADD(LOG_FLOAT, F_d2, &g_self2.F_d_bar.y)
LOG_ADD(LOG_FLOAT, F_d3, &g_self2.F_d_bar.z)

LOG_ADD(LOG_FLOAT, test_f, &g_self2.test_f)

LOG_ADD(LOG_FLOAT, ex_lf1, &g_self2.ex_lf.x)
LOG_ADD(LOG_FLOAT, ex_lf2, &g_self2.ex_lf.y)
LOG_ADD(LOG_FLOAT, ex_lf3, &g_self2.ex_lf.z)

LOG_ADD(LOG_FLOAT, ev_lf1, &g_self2.ev_lf.x)
LOG_ADD(LOG_FLOAT, ev_lf2, &g_self2.ev_lf.y)
LOG_ADD(LOG_FLOAT, ev_lf3, &g_self2.ev_lf.z)

LOG_ADD(LOG_FLOAT, ei_lf1, &g_self2.ei_lf.x)
LOG_ADD(LOG_FLOAT, ei_lf2, &g_self2.ei_lf.y)
LOG_ADD(LOG_FLOAT, ei_lf3, &g_self2.ei_lf.z)

LOG_GROUP_STOP(ctrlLee2)