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

#define DEBUG_MODULE "CRAZYSAR"
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
#include "main.h"

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
  bool is_root;
  
  struct vec F_d_bar;
  struct vec x;
  struct vec v;
  struct mat33 R;

  float kR_geo;
  float kv_geo;

  float eR_geo;
  float ev1_geo;
  float ev2_geo;

  // float kx_rob;
  // float kv_rob;
  // float ki_rob;
  // float sigma_rob;

  // struct vec ex_rob;
  // struct vec ev_rob;
  // struct vec ei_rob;

  float flap_freq;
  float flap_amp;
  float flap_phase;

  float l;
  float follower_yaw;

  struct vec rod;
} controllerLee2_t;

static controllerLee2_t g_self2 = {
  .node = 0, // node = parent is leader
  .parent = 0,
  .is_root = false,

  .m = CF_MASS, // kg
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  .kx = 7.0,
  .kv = 4.0,
  .ki = 1.0,
  .c1 = 3.6,
  .sigma = 1.0,

  .kR = 0.007,
  .kW = 0.002,
  .kI = 0.0005,
  .c2 = 1.6, // Increased this to help with attitude convergence, might need tuning

  .kR_geo = 5.0,
  .kv_geo = 5.0,

  // .kx_rob = 5.0,
  // .kv_rob = 5.0,
  // .ki_rob = 2.0,
  // .sigma_rob = 1.0,

  .flap_freq = 0.0f,
  .flap_amp = 0.0f,
  .flap_phase = 0.0f,

  .l = 0,
  .follower_yaw = 0.0f,
};

// static inline struct mat33 vouter(struct vec a, struct vec b) {
//   struct mat33 out;
//   out.m[0][0] = a.x * b.x;
//   out.m[0][1] = a.x * b.y;
//   out.m[0][2] = a.x * b.z;
//   out.m[1][0] = a.y * b.x;
//   out.m[1][1] = a.y * b.y;
//   out.m[1][2] = a.y * b.z;
//   out.m[2][0] = a.z * b.x;
//   out.m[2][1] = a.z * b.y;
//   out.m[2][2] = a.z * b.z;
//   return out;
// }

static inline struct vec vclampscl2(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void p2pCB(P2PPacket* packet) {
  controllerLee2_t* self = &g_self2;

  // Leader and root do not process any packets
  if (self->node == self->parent || self->is_root) {
    return;
  }

  // Copy time from node 1 (TODO)
  if (packet->port == 1) {
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
  float l = vmag(re);//0.3716;
  self->l = l;
  
  // Desired values
  struct vec re_d;
  struct vec re_d_dot;
  struct vec re_d_ddot;

  struct vec rod_normalized = vnormalize(self->rod);

  // TODO: There is a weird dumb issue here
  // if (self->flap_freq == 0 && self->flap_amp == 0) {
    re_d = vscl(l, rod_normalized);
    re_d_dot = vzero();
    re_d_ddot = vzero();

  // } else {
  //   float theta =      self->flap_amp * cosf(self->flap_freq*(t+self->flap_phase));
  //   float theta_dot =  self->flap_amp * -self->flap_freq*sinf(self->flap_freq*(t+self->flap_phase));
  //   float theta_ddot = self->flap_amp * -self->flap_freq*self->flap_freq*cosf(self->flap_freq*(t+self->flap_phase));

  //   re_d = vnormalize(mkvec(cosf(theta), 0, sinf(theta) + rod_normalized.z));
  //   re_d_dot = vscl(theta_dot, mkvec(-sinf(theta), 0, cosf(theta)));
  //   re_d_ddot = vadd(
  //     vscl(theta_dot * theta_dot, mkvec(-cosf(theta), 0, -sinf(theta))),
  //     vscl(theta_ddot, mkvec(-sinf(theta), 0, cosf(theta)))
  //   );

  //   float angle = atan2f(rod_normalized.y, rod_normalized.x);
  //   struct quat q_flap = qaxisangle(vbasis(2), angle);

  //   re_d = vscl(l, qvrot(q_flap, re_d));
  //   re_d_dot = vscl(l, qvrot(q_flap, re_d_dot));
  //   re_d_ddot = vscl(l, qvrot(q_flap, re_d_ddot));
  // }

  struct vec b1_d = mkvec(cosf(self->follower_yaw), sinf(self->follower_yaw), 0);

  // Geometric controller
  float beta = 2.7f;
  int n = 3;

  struct vec t1 = vnormalize(re);
  struct vec t3 = vnormalize(vcross(re, re_d));
  struct vec t2 = vcross(t3, t1);

  struct vec t1_d = vnormalize(re_d);
  struct vec t3_d = t3;
  struct vec t2_d = vcross(t3_d, t1_d);

  self->eR_geo = l*atan2f(vdot(t1, t2_d), vdot(t1, t1_d));
  self->ev1_geo = vdot(t2, re_dot) - vdot(t2_d, re_d_dot);
  self->ev2_geo = vdot(t3, re_dot) - vdot(t3_d, re_d_dot);
  float ev_norm = sqrtf(self->ev1_geo*self->ev1_geo + self->ev2_geo*self->ev2_geo);
  
  float u_m1 = -self->kR_geo*self->eR_geo - self->kv_geo*self->ev1_geo - beta*(n-1)*self->ev1_geo*ev_norm + vdot(t2_d, re_d_ddot);
  float u_m2 =                             -self->kv_geo*self->ev2_geo - beta*(n-1)*self->ev2_geo*ev_norm + vdot(t3_d, re_d_ddot);

  struct vec u = vadd(vscl(u_m1, t2), vscl(u_m2, t3));

  // Add a robustness term
  // self->ex_rob = vsub(re, re_d);
  // self->ev_rob = vsub(re_dot, re_d_dot);
  // self->ei_rob = vadd(self->ei_rob, vdiv(self->ex_rob, NETWORK_RATE));
  // self->ei_rob = vclampscl2(self->ei_rob, -self->sigma_rob, self->sigma_rob);

  // struct mat33 P_onto_re = mscl(1.0f/vdot(re, re), vouter(re, re));
  // struct vec u = vadd3(
  //   vscl(u_m1, t2),
  //   vscl(u_m2, t3),
  //   vadd(mvmul(P_onto_re, vadd3(
  //       vscl(-self->kx_rob, self->ex_rob),
  //       vscl(-self->kv_rob, self->ev_rob),
  //       re_d_ddot)),
  //     vscl(-self->ki_rob, self->ei_rob)));
  // // vscl(-self->kx_rob*self->ex_rob - self->kv_rob*self->ev_rob - self->ki_rob*self->ei_rob + vdot(re_d_ddot, t1), t1));

  // Disturbance observer
  disturbance_observer_step(&u, &re, &re_dot, &t1);

  self->F_d_bar = vscl(self->m, vadd(vdiv(F_d_l_bar, m_l), u));
  struct vec F_d = vadd(self->F_d_bar, vscl(self->m*GRAVITY_MAGNITUDE, vbasis(2)));

  // Send F_d to the controller
  float f = vdot(F_d, mvmul(self->R, vbasis(2)));
  float thrust = f * UINT16_MAX / powerDistributionGetMaxThrust();

  struct vec b3_d = vnormalize(F_d);
  struct vec b2_d = vnormalize(vcross(b3_d, b1_d));
  struct mat33 R_d = mcolumns(vcross(b2_d, b3_d), b2_d, b3_d);

  struct quat quat_d = mat2quat(R_d);

  setpoint_t setpoint;
  setpoint.mode.quat = modeAbs;

  setpoint.thrust = thrust;
  setpoint.attitudeQuaternion.x = quat_d.x;
  setpoint.attitudeQuaternion.y = quat_d.y;
  setpoint.attitudeQuaternion.z = quat_d.z;
  setpoint.attitudeQuaternion.w = quat_d.w;

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
}

void appMain() {
  controllerLee2_t* self = &g_self2;

  // Wait for the node and parent to be set
  while (self->node == 0 || self->parent == 0) {
    vTaskDelay(M2T(100));
  }

  // Register the callback
  p2pRegisterCB(p2pCB);
  
  P2PPacket packet;
  packet.size = 11*sizeof(float);

  logVarId_t logIdAccX = logGetVarId("acc", "x");
  logVarId_t logIdAccY = logGetVarId("acc", "y");
  logVarId_t logIdAccZ = logGetVarId("acc", "z");

  float accX_init = logGetFloat(logIdAccX);
  float accY_init = logGetFloat(logIdAccY);
  float accZ_init = logGetFloat(logIdAccZ);
  float acc_norm_init = sqrtf(accX_init*accX_init + accY_init*accY_init + accZ_init*accZ_init);

  while (1) {
    vTaskDelay(F2T(NETWORK_RATE));
    if (self->node == 1 && vmag(self->F_d_bar) > 1e-6f) {
      t += 1.0f/NETWORK_RATE;
    }

    packet.port = self->node;
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

    // If a follower is disabled...
    if (self->node != self->parent && disable_props) {
      float accX = logGetFloat(logIdAccX);
      float accY = logGetFloat(logIdAccY);
      float accZ = logGetFloat(logIdAccZ);
      float acc_norm = sqrtf(accX*accX + accY*accY + accZ*accZ);

      if (acc_norm - acc_norm_init > 0.05f) {
        disable_props = 0;
      }
    }
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

  self->eR_geo = 0;
  self->ev1_geo = 0;
  self->ev2_geo = 0;

  // self->ex_rob = vzero();
  // self->ev_rob = vzero();
  // self->ei_rob = vzero();
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

  // Position setpoint
  if (setpoint->mode.x == modeAbs && setpoint->mode.y == modeAbs && setpoint->mode.z == modeAbs) {
    struct vec x_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    if (setpoint->velocity_body) {
      v_d = mvmul(R, v_d); // Convert body frame velocity to world frame
    }
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
    // self->F_d_bar = veltmul(mkvec(0.1f, 0.1f, 1.0f), self->F_d_bar);
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

  // Velocity setpoint
  } else if (setpoint->mode.x == modeVelocity && setpoint->mode.y == modeVelocity) {
    struct vec v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    if (setpoint->velocity_body) {
      v_d = mvmul(R, v_d); // Convert body frame velocity to world frame
    }
    struct vec a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec b1_d = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);

    self->ev = vsub(v, v_d);
    self->ei = vadd(self->ei, vscl(dt, self->ev));
    // self->ei = vclampscl2(self->ei, -self->sigma, self->sigma);
    
    self->F_d_bar = vscl(self->m, vadd3(
      vscl(-self->kv, self->ev),
      vscl(-4.0f, self->ei),
      a_d));

    // Hover setpoint (velocity control in the x/y directions, position control in the z direction)
    if (setpoint->mode.z == modeAbs) {
      float x3 = state->position.z;
      float x3_d = setpoint->position.z;

      self->F_d_bar.z += -self->m*self->kx*(x3 - x3_d);
    }

    // self->F_d_bar = veltmul(mkvec(0.1f, 0.1f, 1.0f), self->F_d_bar);
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

  // Manual setpoint
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

    if (setpoint->mode.quat == modeAbs) {
      R_d = quat2rotmat(mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w));
    } else {
      R_d = quat2rotmat(rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
        -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
        desiredYaw)));
    }

    if (self->node == self->parent) {
      struct vec b3 = mcolumn(R, 2);
      struct vec b3_d = mcolumn(R_d, 2);
      struct vec F_d = vscl(self->f/vdot(b3_d, b3), b3_d);
      self->F_d_bar = vsub(F_d, vscl(self->m*GRAVITY_MAGNITUDE, vbasis(2)));
    }
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

PARAM_GROUP_START(crazysar)

PARAM_ADD(PARAM_UINT8, node, &g_self2.node)
PARAM_ADD(PARAM_UINT8, parent, &g_self2.parent)
PARAM_ADD(PARAM_UINT8, is_root, &g_self2.is_root)

PARAM_ADD(PARAM_FLOAT, m, &g_self2.m)

PARAM_ADD(PARAM_FLOAT, kx, &g_self2.kx)
PARAM_ADD(PARAM_FLOAT, kv, &g_self2.kv)
PARAM_ADD(PARAM_FLOAT, ki, &g_self2.ki)

PARAM_ADD(PARAM_FLOAT, kR, &g_self2.kR)
PARAM_ADD(PARAM_FLOAT, kW, &g_self2.kW)
PARAM_ADD(PARAM_FLOAT, kI, &g_self2.kI)

PARAM_ADD(PARAM_UINT8, disable_props, &disable_props)

PARAM_ADD(PARAM_FLOAT, kR_geo, &g_self2.kR_geo)
PARAM_ADD(PARAM_FLOAT, kv_geo, &g_self2.kv_geo)

// PARAM_ADD(PARAM_FLOAT, kx_rob, &g_self2.kx_rob)
// PARAM_ADD(PARAM_FLOAT, kv_rob, &g_self2.kv_rob)
// PARAM_ADD(PARAM_FLOAT, ki_rob, &g_self2.ki_rob)

PARAM_ADD(PARAM_FLOAT, follower_yaw, &g_self2.follower_yaw)

PARAM_ADD(PARAM_FLOAT, rod_x, &g_self2.rod.x)
PARAM_ADD(PARAM_FLOAT, rod_y, &g_self2.rod.y)
PARAM_ADD(PARAM_FLOAT, rod_z, &g_self2.rod.z)

PARAM_ADD(PARAM_FLOAT, flap_freq, &g_self2.flap_freq)
PARAM_ADD(PARAM_FLOAT, flap_amp, &g_self2.flap_amp)
PARAM_ADD(PARAM_FLOAT, flap_phase, &g_self2.flap_phase)

PARAM_GROUP_STOP(crazysar)

LOG_GROUP_START(crazysar)

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

LOG_ADD(LOG_FLOAT, ei1, &g_self2.ei.x)
LOG_ADD(LOG_FLOAT, ei2, &g_self2.ei.y)
LOG_ADD(LOG_FLOAT, ei3, &g_self2.ei.z)

LOG_ADD(LOG_FLOAT, eR1, &g_self2.eR.x)
LOG_ADD(LOG_FLOAT, eR2, &g_self2.eR.y)
LOG_ADD(LOG_FLOAT, eR3, &g_self2.eR.z)

LOG_ADD(LOG_FLOAT, eW1, &g_self2.eW.x)
LOG_ADD(LOG_FLOAT, eW2, &g_self2.eW.y)
LOG_ADD(LOG_FLOAT, eW3, &g_self2.eW.z)

LOG_ADD(LOG_FLOAT, eI1, &g_self2.eI.x)
LOG_ADD(LOG_FLOAT, eI2, &g_self2.eI.y)
LOG_ADD(LOG_FLOAT, eI3, &g_self2.eI.z)

LOG_ADD(LOG_FLOAT, W_d1, &g_self2.W_d.x)
LOG_ADD(LOG_FLOAT, W_d2, &g_self2.W_d.y)
LOG_ADD(LOG_FLOAT, W_d3, &g_self2.W_d.z)

LOG_ADD(LOG_FLOAT, W_d_dot1, &g_self2.W_d_dot.x)
LOG_ADD(LOG_FLOAT, W_d_dot2, &g_self2.W_d_dot.y)
LOG_ADD(LOG_FLOAT, W_d_dot3, &g_self2.W_d_dot.z)

LOG_ADD(LOG_FLOAT, F_d1, &g_self2.F_d_bar.x)
LOG_ADD(LOG_FLOAT, F_d2, &g_self2.F_d_bar.y)
LOG_ADD(LOG_FLOAT, F_d3, &g_self2.F_d_bar.z)

LOG_ADD(LOG_FLOAT, eR_geo, &g_self2.eR_geo)
LOG_ADD(LOG_FLOAT, ev1_geo, &g_self2.ev1_geo)
LOG_ADD(LOG_FLOAT, ev2_geo, &g_self2.ev2_geo)

// LOG_ADD(LOG_FLOAT, ex_rob, &g_self2.ex_rob)
// LOG_ADD(LOG_FLOAT, ev_rob, &g_self2.ev_rob)
// LOG_ADD(LOG_FLOAT, ei_rob, &g_self2.ei_rob)

LOG_ADD(LOG_FLOAT, t, &t)
LOG_ADD(LOG_FLOAT, l, &g_self2.l)

LOG_GROUP_STOP(crazysar)