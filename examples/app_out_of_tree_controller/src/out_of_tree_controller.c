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
#include "out_of_tree_controller.h"

#include "param.h"
#include "log.h"

static NominalControllerType nominal_controller = NominalControllerTypePID;

static NominalControllerFunctions nominalControllerFunctions[] = {
  {.init = 0, .update = 0},
  {.init = controllerPidInit, .update = controllerPid},
  {.init = controllerLQRInit, .update = controllerLQR},
};

LearningType learning_type = LearningTypeDisable;

// Model parameters from gp_model_params.c
extern const gp_model_params_t thrust_params;
extern const gp_model_params_t torqueX_params;
extern const gp_model_params_t torqueY_params;
extern const gp_model_params_t torqueZ_params;

static float thrust_tilde = 0.0f;
static float torqueX_tilde = 0.0f;
static float torqueY_tilde = 0.0f;
static float torqueZ_tilde = 0.0f;

extern const float random_numbers[];
static int rand_idx = 0;

data_t data;
control_t nominal_control = {0};

const float A[144] = {
  0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 0.0f, 0.0f, 0.0f, 0.000490499999999993f, 0.0f, 0.0f, 1.6349999999999767e-06f, 0.0f, 
  0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 0.0f, -0.000490499999999993f, 0.0f, 0.0f, -1.6349999999999767e-06f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.0f, 0.0980999999999986f, 0.0f, 0.0f, 0.000490499999999993f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, -0.0980999999999986f, 0.0f, 0.0f, -0.000490499999999993f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 0.009999999999999858f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 0.0f, 
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.9999999999999858f, 
};
const float B[48] = {
  0.0f, -1.1766180606402188e-05f, 0.0002476151083430713f, -1.4944631228833127e-05f, 
  0.0f, -0.00024747710583175375f, 1.1766180606402183e-05f, 5.350882790773782e-06f, 
  0.001362397820163468f, 0.0f, 0.0f, 0.0f, 
  0.0f, -0.004706472242560876f, 0.09904604333722854f, -0.00597785249153325f, 
  0.0f, -0.0989908423327015f, 0.004706472242560873f, 0.0021403531163095126f, 
  0.27247956403269363f, 0.0f, 0.0f, 0.0f, 
  0.0f, 3.027242884792095f, -0.14392881475721325f, -0.06545422374035206f, 
  0.0f, -0.1439288147572133f, 3.0289309889060716f, -0.18280894469520648f, 
  0.0f, -0.06545422374035206f, -0.18280894469520648f, 1.7215742425368978f, 
  0.0f, 605.448576958419f, -28.785762951442653f, -13.090844748070415f, 
  0.0f, -28.785762951442663f, 605.7861977812144f, -36.56178893904129f, 
  0.0f, -13.090844748070415f, -36.56178893904129f, 344.31484850737957f, 
};
static float get_A(unsigned int row, unsigned int col) { return A[row*12 + col]; }
static float get_B(unsigned int row, unsigned int col) { return B[row*4 + col]; }

void linear_dynamics_model(data_t *data, const control_t* control, const setpoint_t* setpoint, const sensorData_t* sensors, const state_t* state) {
  // States
  full_state_t x;
  x.position = mkvec(state->position.x, state->position.y, state->position.z); // position in the world frame (m)
  x.velocity = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // velocity in the world frame (m/s)
  x.rpy = mkvec(radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw)); // euler angles (rad)
  x.angularVelocity = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the body frame (rad/s)

  struct mat33 R = quat2rotmat(mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w));

  // Equilibrium state and input
  full_state_t x_eq;
  x_eq.position = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  x_eq.velocity = vzero();
  x_eq.rpy = vzero();
  x_eq.angularVelocity = vzero();

  full_input_t u_eq;
  u_eq.thrust = CF_MASS * GRAVITY_MAGNITUDE;
  u_eq.torque = vzero();

  // Control input
  full_input_t u_bar;
  u_bar.thrust = control->thrust;
  u_bar.torque = mkvec(control->torqueX, control->torqueY, control->torqueZ);

  // System dynamics
  full_state_t x_plus;
  for (int state_idx = 0; state_idx < 12; state_idx++) {
    x_plus.full[state_idx] = x_eq.full[state_idx];
    for (int state_col = 0; state_col < 12; state_col++) {
      x_plus.full[state_idx] += get_A(state_idx, state_col) * (x.full[state_col] - x_eq.full[state_col]);
    }
    for (int input_idx = 0; input_idx < 4; input_idx++) {
      x_plus.full[state_idx] += get_B(state_idx, input_idx) * (u_bar.full[input_idx] - u_eq.full[input_idx]);
    }
  }

  // Set data
  data->vbz_plus = mvmul(mtranspose(R), x_plus.velocity).z;
  data->vbz = mvmul(mtranspose(R), x.velocity).z;
  data->R33 = R.m[2][2];

  data->W_plus = x_plus.angularVelocity;
  data->W = x.angularVelocity;
}

static float c_hat(const float* data, const gp_model_params_t* params) {
  float y_star = 0.0f;
  for (int sample_idx = 0; sample_idx < params->NUM_SAMPLES; sample_idx++) {
    // Kernel
    float sqdist = 0.0f;
    for (int data_idx = 0; data_idx < params->NUM_DIMS; data_idx++) {
      float diff = data[data_idx] - params->X_train[sample_idx*params->NUM_DIMS + data_idx];
      sqdist += diff * diff / params->lengthscale_sq;
    }
    float rbf_kernel = expf(-0.5f * sqdist);

    y_star += rbf_kernel * params->alpha_times_outputscale[sample_idx];
  }
  return y_star;
}

static void nonlinear_dynamics_model(data_t *data, const control_t* control, const sensorData_t* sensors, const state_t* state, const float dt) {
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

  data->W_plus = W_next;
  data->W = W;
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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t tick) {
  if (!RATE_DO_EXECUTE(ILBC_RATE, tick)) {
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

  // Calculate the nominal control (u_bar)
  nominalControllerFunctions[nominal_controller].update(&nominal_control, setpoint, sensors, state, tick);

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

  if (learning_type == LearningTypeDisable) {
    // Return unaltered nominal control
    thrust_tilde = 0.0f;
    torqueX_tilde = 0.0f;
    torqueY_tilde = 0.0f;
    torqueZ_tilde = 0.0f;
  } else if (learning_type == LearningTypeTraining) {
    // Add exploration noise to the nominal control
    thrust_tilde = nominal_control.thrust * random_numbers[4*rand_idx] * 0.5f;
    torqueX_tilde = nominal_control.torqueX * random_numbers[4*rand_idx + 1] * 0.5f;
    torqueY_tilde = nominal_control.torqueY * random_numbers[4*rand_idx + 2] * 0.5f;
    torqueZ_tilde = nominal_control.torqueZ * random_numbers[4*rand_idx + 3] * 0.5f;

    if (RATE_DO_EXECUTE(10, tick)) {
      if (++rand_idx >= sizeof((float*)random_numbers) / sizeof(random_numbers[0]) / 4) {
        rand_idx = 0;
      }
    }
  } else {
    // Estimate the next state after a given time if the nominal control is applied
    if (learning_type == LearningTypeLinearModel) {
      linear_dynamics_model(&data, &nominal_control, setpoint, sensors, state);
    } else if (learning_type == LearningTypeNonlinearModel) {
      nonlinear_dynamics_model(&data, &nominal_control, sensors, state, 1.0f/ILBC_RATE);
    }

    // c_hat function
    thrust_tilde = c_hat(data.translation, &thrust_params);
    torqueX_tilde = c_hat(data.rotation, &torqueX_params);
    torqueY_tilde = c_hat(data.rotation, &torqueY_params);
    torqueZ_tilde = c_hat(data.rotation, &torqueZ_params);
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSi = nominal_control.thrustSi + thrust_tilde;
  control->torqueX = nominal_control.torqueX + torqueX_tilde;
  control->torqueY = nominal_control.torqueY + torqueY_tilde;
  control->torqueZ = nominal_control.torqueZ + torqueZ_tilde;
}


PARAM_GROUP_START(ILBC)

PARAM_ADD(PARAM_UINT8, nominal_controller, &nominal_controller)
PARAM_ADD(PARAM_UINT8, learning_type, &learning_type)

PARAM_GROUP_STOP(ILBC)


LOG_GROUP_START(ILBC)

LOG_ADD(LOG_UINT8, nominal_controller, &nominal_controller)
LOG_ADD(LOG_FLOAT, nominal_thrust, &nominal_control.thrustSi)
LOG_ADD(LOG_FLOAT, nominal_torqueX, &nominal_control.torqueX)
LOG_ADD(LOG_FLOAT, nominal_torqueY, &nominal_control.torqueY)
LOG_ADD(LOG_FLOAT, nominal_torqueZ, &nominal_control.torqueZ)

LOG_ADD(LOG_FLOAT, thrust_tilde, &thrust_tilde)
LOG_ADD(LOG_FLOAT, torqueX_tilde, &torqueX_tilde)
LOG_ADD(LOG_FLOAT, torqueY_tilde, &torqueY_tilde)
LOG_ADD(LOG_FLOAT, torqueZ_tilde, &torqueZ_tilde)

LOG_ADD(LOG_FLOAT, vbz_plus, &data.vbz_plus)
LOG_ADD(LOG_FLOAT, vbz, &data.vbz)
LOG_ADD(LOG_FLOAT, R33, &data.R33)

LOG_GROUP_STOP(ILBC)