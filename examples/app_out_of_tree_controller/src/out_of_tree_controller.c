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

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

#include "controller.h"
#include "controller_pid.h"
#include "controller_lqr.h"
#include "commander.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "power_distribution.h"
#include "gp_model_params.h"

#include "param.h"
#include "log.h"

typedef enum {
  NominalControllerTypeNone,
  NominalControllerTypePID,
  NominalControllerTypeLQR,
  NominalControllerTypeCount
} NominalControllerType;

typedef struct {
  void (*init)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
} NominalControllerFunctions;

static NominalControllerFunctions nominalControllerFunctions[] = {
  {.init = 0, .update = 0},
  {.init = controllerPidInit, .update = controllerPid},
  {.init = controllerLQRInit, .update = controllerLQR},
};

static NominalControllerType nominal_controller = NominalControllerTypePID;

// Model parameters from gp_model_params.c
extern const gp_model_params_t f_params;

float f_star = 0.0f;

uint8_t use_nominal = 0;

typedef struct data_s {
  union {
    struct {
      float vbz_plus;
      float vbz;
      float R33;
    };
    float translation[3];
  };
  union {
    struct {
      float Wx_plus;
      float Wy_plus;
      float Wz_plus;
      float Wx;
      float Wy;
      float Wz;
    };
    float rotation[6];
  };
} data_t;

data_t data;
control_t nominal_control = {0};

// struct mat33 A_block_vrpy = {
//   .m = { { 0.0f, 9.81e-2f, 0.0f },
//          { -9.81e-2f, 0.0f, 0.0f },
//          { -4.90452123e-08f, -4.90452123e-08f, 0.0f } }
// };
// struct mat33 A_block_WW = {
//   .m = { { 1.0f, -1.10080266e-09f, 1.11060286e-09f },
//          { 4.65498046e-10f, 1.0f, -4.86942337e-10f },
//          { -3.12320334e-10f, 3.09624941e-10f, 1.0f } }
// };
// struct mat33 B_block_WM = {
//   .m = { { 605.448577f, -28.785763f, -13.0908447f },
//          { -28.785763f, 605.786198f, -36.5617889f },
//          { -13.0908447f, -36.5617889f, 344.314849f } }
// };

// void linear_dynamics_model(data_t *data, const control_t* control, const setpoint_t* setpoint, const sensorData_t* sensors, const state_t* state, const float dt) {
//   // State
//   struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // velocity in the world frame (m/s)
//   struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w); // attitude quaternion
//   struct vec rpy = quat2rpy(q); // euler angles (rad)
//   struct mat33 R = quat2rotmat(q); // rotation matrix
//   struct vec W = mvmul(R, mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z))); // angular velocity in the world frame (rad/s)

//   // Control input
//   float f = control->thrustSi;
//   struct vec M = mkvec(control->torqueX, control->torqueY, control->torqueZ);

//   // System dynamics  
//   struct vec v_plus = vadd3(
//     v,
//     mvmul(A_block_vrpy, rpy),
//     mkvec(0.0f, 0.0f, 0.37037037f * (f - CF_MASS * GRAVITY_MAGNITUDE))
//   );

//   struct vec W_plus = vadd(
//     mvmul(A_block_WW, W),
//     mvmul(B_block_WM, M)
//   );

//   // Set data
//   data->vbz_plus = mvmul(mtranspose(R), v_plus).z;
//   data->vbz = mvmul(mtranspose(R), v).z;
//   data->R33 = R.m[2][2];

//   data->Wx = W.x;
//   data->Wy = W.y;
//   data->Wz = W.z;
//   data->Wx_plus = W_plus.x;
//   data->Wy_plus = W_plus.y;
//   data->Wz_plus = W_plus.z;
// }

float c_hat(const data_t* data, const gp_model_params_t* params) {
  float y_star = 0.0f;
  for (int sample_idx = 0; sample_idx < params->NUM_SAMPLES; sample_idx++) {
    // Kernel
    float sqdist = 0.0f;
    for (int data_idx = 0; data_idx < params->NUM_DIMS; data_idx++) {
      float diff = data->translation[data_idx] - params->X_train[sample_idx*params->NUM_DIMS + data_idx];
      sqdist += diff * diff / params->lengthscale_sq[data_idx];
    }
    float rbf_kernel = expf(-0.5f * sqdist);

    y_star += rbf_kernel * params->alpha_times_outputscale[sample_idx];
  }
  return y_star * params->y_std + params->y_mean;
}

void nonlinear_dynamics_model(data_t *data, const control_t* control, const sensorData_t* sensors, const state_t* state, const float dt) {
  // Diagonal of the inertia matrix
  struct vec J = mkvec(16.571710e-6f, 16.655602e-6f, 29.261652e-6f);

  // Current state
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // m/s
  struct mat33 R = quat2rotmat(mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w));
  struct vec W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the body frame (rad/s)
  float vbz = mvmul(mtranspose(R), v).z; // m/s
  
  // Control input
  float f = control->thrustSi;
  struct vec M = mkvec(control->torqueX, control->torqueY, control->torqueZ);

  // System dynamics
  float vbz_dot = f / CF_MASS - R.m[2][2] * GRAVITY_MAGNITUDE;
  struct vec W_dot = veltdiv(vsub(M, vcross(W, veltmul(J, W))), J);

  // Estimate the next state
  float vbz_next = vbz + vbz_dot * dt; // m/s
  struct vec W_next = vadd(W, vscl(dt, W_dot)); // rad/s

  // Set data
  data->vbz_plus = vbz_next;
  data->vbz = vbz;
  data->R33 = R.m[2][2];

  data->Wx = W.x;
  data->Wy = W.y;
  data->Wz = W.z;
  data->Wx_plus = W_next.x;
  data->Wy_plus = W_next.y;
  data->Wz_plus = W_next.z;
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(1000));
  }
}

void controllerOutOfTreeInit() {
  for (int i = 1; i < NominalControllerTypeCount; i++) {
    nominalControllerFunctions[i].init();
  }
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Calculate the nominal control (u_bar)
  nominalControllerFunctions[nominal_controller].update(&nominal_control, setpoint, sensors, state, tick);

  if (use_nominal) {
    *control = nominal_control;
    return;
  }

  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    return;
  }

  // Disable controller in manual mode if thrust is low
  if (setpoint->mode.z == modeDisable && setpoint->thrust < 1000.0f) {
    control->controlMode = controlModeForceTorque;
    control->thrustSi = 0.0f;
    control->torqueX = 0.0f;
    control->torqueY = 0.0f;
    control->torqueZ = 0.0f;
    return;
  }

  // Convert nominal control to controlModeForceTorque
  if (nominal_control.controlMode == controlModeLegacy) {
    const float arm = 0.707106781f * ARM_LENGTH;
    control_t temp_control = nominal_control;

    nominal_control.controlMode = controlModeForceTorque;
    nominal_control.thrustSi =  temp_control.thrust                 / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueX  =  temp_control.roll   / 2.0f * arm    / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueY  = -temp_control.pitch  / 2.0f * arm    / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueZ  = -temp_control.yaw    * THRUST2TORQUE / UINT16_MAX * powerDistributionGetMaxThrust();
  }

  // Estimate the next state after a given time if the nominal control is applied
  nonlinear_dynamics_model(&data, &nominal_control, sensors, state, SAMPLING_PERIOD);
  // TODO: Make the linear model a function of sampling period
  // linear_dynamics_model(&data, &nominal_control, setpoint, sensors, state, 0.01f);

  // c_hat function
  f_star = c_hat(&data, &f_params);

  control->controlMode = controlModeForceTorque;
  control->thrustSi = f_star / UINT16_MAX * powerDistributionGetMaxThrust();
  control->torqueX = nominal_control.torqueX;
  control->torqueY = nominal_control.torqueY;
  control->torqueZ = nominal_control.torqueZ;
}


PARAM_GROUP_START(ILBC)

PARAM_ADD(PARAM_UINT8, nominal_controller, &nominal_controller)
PARAM_ADD(PARAM_UINT8, use_nominal, &use_nominal)

PARAM_GROUP_STOP(ILBC)


LOG_GROUP_START(ILBC)

LOG_ADD(LOG_FLOAT, nominal_thrust, &nominal_control.thrust)
LOG_ADD(LOG_FLOAT, learned_thrust, &f_star)

LOG_ADD(LOG_FLOAT, vbz_plus, &data.vbz_plus)
LOG_ADD(LOG_FLOAT, vbz, &data.vbz)
LOG_ADD(LOG_FLOAT, R33, &data.R33)

LOG_GROUP_STOP(ILBC)