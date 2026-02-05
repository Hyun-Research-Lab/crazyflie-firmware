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
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "commander.h"
#include "power_distribution.h"
#include "crtp_commander_high_level.h"
#include "radiolink.h"
#include "param.h"
#include "log.h"
#include "main.h"

// #define PID_ROBUSTNESS

static uint8_t disable_props = 0;
static uint8_t enable_filters = 0;

static float t = 0;

static struct vec target_position_root = { 0, 0, 0 };

// Quadrotor parameters
static float m = CF_MASS; // kg
static struct vec J = { 16.571710e-6f, 16.655602e-6f, 29.261652e-6f }; // kg m^2

// Gains
static float kx = 7.0f;
static float kv = 4.0f;
static float ki = 1.0f;
static float c1 = 3.6f;
static float sigma = 1.0f;

static float kR = 0.007f;
static float kW = 0.002f;
static float kI = 0.0005f;
static float c2 = 1.6f; // Increased this to help with attitude convergence, might need tuning

// Errors
static struct vec ex = { 0, 0, 0 };
static struct vec ev = { 0, 0, 0 };
static struct vec ei = { 0, 0, 0 };

static struct vec eR = { 0, 0, 0 };
static struct vec eW = { 0, 0, 0 };
static struct vec eI = { 0, 0, 0 };

// Wrench
static float f = 0;
static struct vec M = { 0, 0, 0 };

// Previous values
static struct mat33 R_d_prev = { .m = {
  { NAN, NAN, NAN },
  { NAN, NAN, NAN },
  { NAN, NAN, NAN }
} };
static struct vec W_d_prev = { NAN, NAN, NAN };

static struct vec W_d_raw[FILTER_SIZE];
static struct vec W_d_dot_raw[FILTER_SIZE];

static struct vec W_d = { 0, 0, 0 };
static struct vec W_d_dot = { 0, 0, 0 };

// For leader-follower
static uint8_t node = 0; // node = parent is leader
static uint8_t parent = 0;
static bool is_root = false;

static struct vec F_d_bar = { 0, 0, 0 };
static struct vec x = { 0, 0, 0 };
static struct vec v = { 0, 0, 0 };
static struct mat33 R = { .m = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 }
} };

static float kR_geo = 5.0f;
static float kv_geo = 5.0f;

static float eR_geo = 0.0f;
static float ev1_geo = 0.0f;
static float ev2_geo = 0.0f;

#ifdef PID_ROBUSTNESS
static float kx_rob = 5.0f;
static float kv_rob = 5.0f;
static float ki_rob = 2.0f;
static float sigma_rob = 1.0f;

static struct vec ex_rob = { 0, 0, 0 };
static struct vec ev_rob = { 0, 0, 0 };
static struct vec ei_rob = { 0, 0, 0 };
#endif

static float flap_freq = 0.0f;
static float flap_amp = 0.0f;
static float flap_phase = 0.0f;

static float l = 0.0f;
static float follower_yaw = 0.0f;

// Parent information
static float m_l = 0.0f;
static struct vec F_d_l_bar = { 0, 0, 0 };
static struct vec x_l;
static struct vec v_l;

static int8_t rod[3] = { 0, 1, 0 }; // default value
static struct vec re = { 0, 0, 0 };

static uint32_t config_params = 0;

static paramVarId_t paramIdLedBitmask;

static uint32_t counter = 0;

#ifdef PID_ROBUSTNESS
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
#endif

static inline struct vec vclampscl2(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void setLedBitmask() {
  if (node == parent) {
    paramSetInt(paramIdLedBitmask, LED_LEADER);
  } else if (is_root) {
    paramSetInt(paramIdLedBitmask, LED_ROOT);
  } else {
    paramSetInt(paramIdLedBitmask, LED_FOLLOWER);
  }
}

void p2pCB(P2PPacket* packet) {
  // Leader and root do not process any packets
  // Only process packets from the parent
  if (node == parent || is_root || packet->port != parent) {
    return;
  }

  // Reset the counter every time we get a packet from the parent
  counter = 0;

  // Get parent information
  memcpy(        &m_l, packet->data + 0 * sizeof(float),  sizeof(float));
  memcpy(&F_d_l_bar.x, packet->data + 1 * sizeof(float),  sizeof(float));
  memcpy(&F_d_l_bar.y, packet->data + 2 * sizeof(float),  sizeof(float));
  memcpy(&F_d_l_bar.z, packet->data + 3 * sizeof(float),  sizeof(float));
  memcpy(      &x_l.x, packet->data + 4 * sizeof(float),  sizeof(float));
  memcpy(      &x_l.y, packet->data + 5 * sizeof(float),  sizeof(float));
  memcpy(      &x_l.z, packet->data + 6 * sizeof(float),  sizeof(float));
  memcpy(      &v_l.x, packet->data + 7 * sizeof(float),  sizeof(float));
  memcpy(      &v_l.y, packet->data + 8 * sizeof(float),  sizeof(float));
  memcpy(      &v_l.z, packet->data + 9 * sizeof(float),  sizeof(float));
}

void setRootSetpoint() {
  if (veq(target_position_root, vzero())) {
    target_position_root = x;
  }

  setpoint_t setpoint = {0};
  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;

  setpoint.position.x = target_position_root.x;
  setpoint.position.y = target_position_root.y;
  setpoint.position.z = target_position_root.z;

  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
}

void setFollowerSetpoint() {
  if (vmag(F_d_l_bar) < 1e-6f) {
    ei = vzero();
    return;
  }

  // Where the magic happens
  re = vsub(x, x_l);
  struct vec re_dot = vsub(v, v_l);
  l = vmag(re);//0.3716;

  // // If the rod is broken, become root
  // if ((l < 0.25f || l > 0.55f) && !disable_props) {
  //   is_root = true;
  //   setLedBitmask();

  //   ei = vzero();
  //   return;
  // }

  // Desired values
  struct vec re_d;
  struct vec re_d_dot;
  struct vec re_d_ddot;

  struct vec rod_normalized = vnormalize(mkvec((float)rod[0], (float)rod[1], (float)rod[2]));

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

  struct vec b1_d = mkvec(cosf(follower_yaw), sinf(follower_yaw), 0);

  // Geometric controller
  float beta = 2.7f;
  int n = 3;

  struct vec t1 = vnormalize(re);
  struct vec t3 = vnormalize(vcross(re, re_d));
  struct vec t2 = vcross(t3, t1);

  struct vec t1_d = vnormalize(re_d);
  struct vec t3_d = t3;
  struct vec t2_d = vcross(t3_d, t1_d);

  eR_geo = l*atan2f(vdot(t1, t2_d), vdot(t1, t1_d));
  ev1_geo = vdot(t2, re_dot) - vdot(t2_d, re_d_dot);
  ev2_geo = vdot(t3, re_dot) - vdot(t3_d, re_d_dot);
  float ev_norm = sqrtf(ev1_geo*ev1_geo + ev2_geo*ev2_geo);
  
  float u_m1 = -kR_geo*eR_geo - kv_geo*ev1_geo - beta*(n-1)*ev1_geo*ev_norm + vdot(t2_d, re_d_ddot);
  float u_m2 =                 -kv_geo*ev2_geo - beta*(n-1)*ev2_geo*ev_norm + vdot(t3_d, re_d_ddot);

  struct vec u = vadd(vscl(u_m1, t2), vscl(u_m2, t3));

  if (!disable_props) {
#ifdef PID_ROBUSTNESS
    ex_rob = vsub(re, re_d);
    ev_rob = vsub(re_dot, re_d_dot);
    ei_rob = vadd(ei_rob, vdiv(ex_rob, CRAZYSAR_NETWORK_RATE));
    ei_rob = vclampscl2(ei_rob, -sigma_rob, sigma_rob);

    struct mat33 P_onto_re = mscl(1.0f/vdot(re, re), vouter(re, re));
    u = vadd3(
      u,
      mvmul(P_onto_re, vadd3(
        vscl(-kx_rob, ex_rob),
        vscl(-kv_rob, ev_rob),
        re_d_ddot
      )),
      vscl(-ki_rob, ei_rob)
    );
    // vscl(-self->kx_rob*self->ex_rob - self->kv_rob*self->ev_rob - self->ki_rob*self->ei_rob + vdot(re_d_ddot, t1), t1));

#else
    // Disturbance observer
    disturbance_observer_step(&u, &re, &re_dot, &t1);
#endif
  }

  F_d_bar = vscl(m, vadd(vdiv(F_d_l_bar, m_l), u));
  struct vec F_d = vadd(F_d_bar, vscl(m*GRAVITY_MAGNITUDE, vbasis(2)));

  // Send F_d to the controller
  float f = vdot(F_d, mvmul(R, vbasis(2)));
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
  paramIdLedBitmask = paramGetVarId("led", "bitmask");
  
  // Wait for the node and parent to be set
  while (node == 0 || parent == 0) {
    vTaskDelay(M2T(100));
  }

  // Register the callback
  p2pRegisterCB(p2pCB);
  
  P2PPacket packet;
  packet.size = 10 * sizeof(float);

  logVarId_t logIdAccX = logGetVarId("acc", "x");
  logVarId_t logIdAccY = logGetVarId("acc", "y");
  logVarId_t logIdAccZ = logGetVarId("acc", "z");

  float accX_init = logGetFloat(logIdAccX);
  float accY_init = logGetFloat(logIdAccY);
  float accZ_init = logGetFloat(logIdAccZ);
  float acc_norm_init = sqrtf(accX_init*accX_init + accY_init*accY_init + accZ_init*accZ_init);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, F2T(CRAZYSAR_NETWORK_RATE));

    // If a follower is disabled...
    if (node != parent && disable_props) {
      float accX = logGetFloat(logIdAccX);
      float accY = logGetFloat(logIdAccY);
      float accZ = logGetFloat(logIdAccZ);
      float acc_norm = sqrtf(accX*accX + accY*accY + accZ*accZ);

      if (acc_norm - acc_norm_init > 0.05f) {
        disable_props = 0;
      }
    }

    // If it has been a certain number of cycles since the last command from the parent, become root
    if (node == parent || is_root) {
      counter = 0;
    } else {
      counter++;
      // if (counter > 100 && t > 1.0f) {
      //   is_root = true;
      //   setLedBitmask();
      // }
    }

    if (node == parent) {
      eR_geo = 0.0f;
      ev1_geo = 0.0f;
      ev2_geo = 0.0f;

    } else if (is_root) {
      eR_geo = 0.0f;
      ev1_geo = 0.0f;
      ev2_geo = 0.0f;
      
      setRootSetpoint();

    } else {
      setFollowerSetpoint();
    }

    t += 1.0f / CRAZYSAR_NETWORK_RATE;

    packet.port = node;
    memcpy(packet.data + 0 * sizeof(float),         &m, sizeof(float));
    memcpy(packet.data + 1 * sizeof(float), &F_d_bar.x, sizeof(float));
    memcpy(packet.data + 2 * sizeof(float), &F_d_bar.y, sizeof(float));
    memcpy(packet.data + 3 * sizeof(float), &F_d_bar.z, sizeof(float));
    memcpy(packet.data + 4 * sizeof(float),       &x.x, sizeof(float));
    memcpy(packet.data + 5 * sizeof(float),       &x.y, sizeof(float));
    memcpy(packet.data + 6 * sizeof(float),       &x.z, sizeof(float));
    memcpy(packet.data + 7 * sizeof(float),       &v.x, sizeof(float));
    memcpy(packet.data + 8 * sizeof(float),       &v.y, sizeof(float));
    memcpy(packet.data + 9 * sizeof(float),       &v.z, sizeof(float));

    // vTaskDelay(M2T(node)); // Stagger transmissions based on node ID
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

static inline void resetFilterBuffers() {
  for (int i = 0; i < FILTER_SIZE; i++) {
    W_d_raw[i] = vzero();
    W_d_dot_raw[i] = vzero();
  }
}

void controllerOutOfTreeInit() {
  resetFilterBuffers();
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!RATE_DO_EXECUTE(CRAZYSAR_ATTITUDE_RATE, tick)) {
    return;
  }

  float dt = (float)(1.0f / CRAZYSAR_ATTITUDE_RATE);

  // States
  x = mkvec(state->position.x, state->position.y, state->position.z);
  v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  R = quat2rotmat(q);
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

    ex = vsub(x, x_d);
    ev = vsub(v, v_d);
    ei = vadd(ei, vscl(dt, vadd(ev, vscl(c1, ex))));
    ei = vclampscl2(ei, -sigma, sigma);
    
    F_d_bar = vscl(m, vadd4(
      vscl(-kx, ex),
      vscl(-kv, ev),
      vscl(-ki, ei),
      a_d));
    // self->F_d_bar = veltmul(mkvec(0.1f, 0.1f, 1.0f), self->F_d_bar);
    struct vec F_d = vadd(F_d_bar, vscl(m*GRAVITY_MAGNITUDE, vbasis(2)));
    f = vdot(F_d, mvmul(R, vbasis(2)));
    
    if (f < 0.01f) {
      ei = vzero();
      eI = vzero();
      resetFilterBuffers();
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

    ev = vsub(v, v_d);
    ei = vadd(ei, vscl(dt, ev));
    // ei = vclampscl2(ei, -sigma, sigma);
    
    F_d_bar = vscl(m, vadd3(
      vscl(-kv, ev),
      vscl(-4.0f, ei),
      a_d));

    // Hover setpoint (velocity control in the x/y directions, position control in the z direction)
    if (setpoint->mode.z == modeAbs) {
      float x3 = state->position.z;
      float x3_d = setpoint->position.z;

      F_d_bar.z += -m*kx*(x3 - x3_d);
    }

    // self->F_d_bar = veltmul(mkvec(0.1f, 0.1f, 1.0f), self->F_d_bar);
    struct vec F_d = vadd(F_d_bar, vscl(m*GRAVITY_MAGNITUDE, vbasis(2)));
    f = vdot(F_d, mvmul(R, vbasis(2)));
    
    if (f < 0.01f) {
      ei = vzero();
      eI = vzero();
      resetFilterBuffers();
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
      ei = vzero();
      eI = vzero();
      resetFilterBuffers();
      return;
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    f = setpoint->thrust / UINT16_MAX * max_thrust;

    if (setpoint->mode.quat == modeAbs) {
      R_d = quat2rotmat(mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w));
    } else {
      R_d = quat2rotmat(rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
        -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
        desiredYaw)));
    }

    if (node == parent) {
      struct vec b3 = mcolumn(R, 2);
      struct vec b3_d = mcolumn(R_d, 2);
      struct vec F_d = vscl(f/vdot(b3_d, b3), b3_d);
      F_d_bar = vsub(F_d, vscl(m*GRAVITY_MAGNITUDE, vbasis(2)));
    }
  }

  // Calculate M
  if (vneq(mcolumn(R_d_prev, 0), vrepeat(NAN)) && vneq(mcolumn(R_d_prev, 1), vrepeat(NAN)) && vneq(mcolumn(R_d_prev, 2), vrepeat(NAN))) {
    if (enable_filters) {
      // Update W_d_raw buffer
      for (int i = 0; i < FILTER_SIZE - 1; i++) {
        W_d_raw[i] = W_d_raw[i + 1];
      }
      W_d_raw[FILTER_SIZE - 1] = vdiv(mvee(mlog(mmul(mtranspose(R_d_prev), R_d))), dt);
      W_d = filter(W_d_raw, FILTER_SIZE);
    } else {
      W_d = vzero();
    }

    if (vneq(W_d_prev, vrepeat(NAN))) {
      if (enable_filters) {
        // Update W_d_dot_raw buffer
        for (int i = 0; i < FILTER_SIZE - 1; i++) {
          W_d_dot_raw[i] = W_d_dot_raw[i + 1];
        }
        W_d_dot_raw[FILTER_SIZE - 1] = vdiv(vsub(W_d, W_d_prev), dt);
        W_d_dot = filter(W_d_dot_raw, FILTER_SIZE);
      } else {
        W_d_dot = vzero();
      }

      eR = vscl(0.5f, mvee(msub(
        mmul(mtranspose(R_d), R),
        mmul(mtranspose(R), R_d))));
      eW = vsub(W, mvmul(mtranspose(R), mvmul(R_d, W_d)));
      eI = vadd(eI, vscl(dt, vadd(eW, vscl(c2, eR))));

      M = vadd(vadd4(
        vneg(vscl(kR, eR)),
        vneg(vscl(kW, eW)),
        vneg(vscl(kI, eI)),
        vcross(mvmul(mtranspose(R), mvmul(R_d, W_d)), veltmul(J, mvmul(mtranspose(R), mvmul(R_d, W_d))))),
        veltmul(J, mvmul(mtranspose(R), mvmul(R_d, W_d_dot))));
    }
    W_d_prev = W_d;
  }
  R_d_prev = R_d;

  control->controlMode = controlModeForceTorque;
  control->thrustSi = f;
  control->torque[0] = M.x;
  control->torque[1] = M.y;
  control->torque[2] = M.z;
  if (disable_props) {
    control->thrustSi = 0.0f;
    control->torque[0] = 0.0f;
    control->torque[1] = 0.0f;
    control->torque[2] = 0.0f;
  }
}

void decodeConfigParams() {
  node = config_params & 0x0F;
  parent = (config_params >> 4) & 0x0F;

  memcpy(rod, (int8_t*)&config_params + 1, 3*sizeof(int8_t));

  setLedBitmask();

  // DEBUG_PRINT("Config params decoded: node=%d, parent=%d, rod=(%d, %d, %d)\n", node, parent, rod[0], rod[1], rod[2]);
}

PARAM_GROUP_START(crazysar)

PARAM_ADD(PARAM_UINT8, node, &node)
PARAM_ADD(PARAM_UINT8, parent, &parent)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, is_root, &is_root, &setLedBitmask)

PARAM_ADD(PARAM_FLOAT, m, &m)

PARAM_ADD(PARAM_FLOAT, kx, &kx)
PARAM_ADD(PARAM_FLOAT, kv, &kv)
PARAM_ADD(PARAM_FLOAT, ki, &ki)

PARAM_ADD(PARAM_FLOAT, kR, &kR)
PARAM_ADD(PARAM_FLOAT, kW, &kW)
PARAM_ADD(PARAM_FLOAT, kI, &kI)

PARAM_ADD(PARAM_UINT8, disable_props, &disable_props)

PARAM_ADD(PARAM_FLOAT, kR_geo, &kR_geo)
PARAM_ADD(PARAM_FLOAT, kv_geo, &kv_geo)

#ifdef PID_ROBUSTNESS
PARAM_ADD(PARAM_FLOAT, kx_rob, &kx_rob)
PARAM_ADD(PARAM_FLOAT, kv_rob, &kv_rob)
PARAM_ADD(PARAM_FLOAT, ki_rob, &ki_rob)
#endif

PARAM_ADD(PARAM_FLOAT, follower_yaw, &follower_yaw)

PARAM_ADD(PARAM_FLOAT, flap_freq, &flap_freq)
PARAM_ADD(PARAM_FLOAT, flap_amp, &flap_amp)
PARAM_ADD(PARAM_FLOAT, flap_phase, &flap_phase)

PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, config_params, &config_params, &decodeConfigParams)

PARAM_GROUP_STOP(crazysar)

LOG_GROUP_START(crazysar)

// Wrench
LOG_ADD(LOG_FLOAT, f, &f)
LOG_ADD(LOG_FLOAT, M1, &M.x)
LOG_ADD(LOG_FLOAT, M2, &M.y)
LOG_ADD(LOG_FLOAT, M3, &M.z)

// Errors
LOG_ADD(LOG_FLOAT, ex1, &ex.x)
LOG_ADD(LOG_FLOAT, ex2, &ex.y)
LOG_ADD(LOG_FLOAT, ex3, &ex.z)

LOG_ADD(LOG_FLOAT, ev1, &ev.x)
LOG_ADD(LOG_FLOAT, ev2, &ev.y)
LOG_ADD(LOG_FLOAT, ev3, &ev.z)

LOG_ADD(LOG_FLOAT, ei1, &ei.x)
LOG_ADD(LOG_FLOAT, ei2, &ei.y)
LOG_ADD(LOG_FLOAT, ei3, &ei.z)

LOG_ADD(LOG_FLOAT, eR1, &eR.x)
LOG_ADD(LOG_FLOAT, eR2, &eR.y)
LOG_ADD(LOG_FLOAT, eR3, &eR.z)

LOG_ADD(LOG_FLOAT, eW1, &eW.x)
LOG_ADD(LOG_FLOAT, eW2, &eW.y)
LOG_ADD(LOG_FLOAT, eW3, &eW.z)

LOG_ADD(LOG_FLOAT, eI1, &eI.x)
LOG_ADD(LOG_FLOAT, eI2, &eI.y)
LOG_ADD(LOG_FLOAT, eI3, &eI.z)

LOG_ADD(LOG_FLOAT, W_d1, &W_d.x)
LOG_ADD(LOG_FLOAT, W_d2, &W_d.y)
LOG_ADD(LOG_FLOAT, W_d3, &W_d.z)

LOG_ADD(LOG_FLOAT, W_d_dot1, &W_d_dot.x)
LOG_ADD(LOG_FLOAT, W_d_dot2, &W_d_dot.y)
LOG_ADD(LOG_FLOAT, W_d_dot3, &W_d_dot.z)

LOG_ADD(LOG_FLOAT, F_d1, &F_d_bar.x)
LOG_ADD(LOG_FLOAT, F_d2, &F_d_bar.y)
LOG_ADD(LOG_FLOAT, F_d3, &F_d_bar.z)

LOG_ADD(LOG_FLOAT, eR_geo, &eR_geo)
LOG_ADD(LOG_FLOAT, ev1_geo, &ev1_geo)
LOG_ADD(LOG_FLOAT, ev2_geo, &ev2_geo)

#ifdef PID_ROBUSTNESS
LOG_ADD(LOG_FLOAT, ex_rob, &ex_rob)
LOG_ADD(LOG_FLOAT, ev_rob, &ev_rob)
LOG_ADD(LOG_FLOAT, ei_rob, &ei_rob)
#endif

LOG_ADD(LOG_FLOAT, t, &t)
LOG_ADD(LOG_FLOAT, l, &l)

LOG_ADD(LOG_INT8, rod1, &rod[0])
LOG_ADD(LOG_INT8, rod2, &rod[1])
LOG_ADD(LOG_INT8, rod3, &rod[2])

LOG_ADD(LOG_FLOAT, re1, &re.x)
LOG_ADD(LOG_FLOAT, re2, &re.y)
LOG_ADD(LOG_FLOAT, re3, &re.z)

LOG_ADD(LOG_UINT32, counter, &counter)

LOG_GROUP_STOP(crazysar)