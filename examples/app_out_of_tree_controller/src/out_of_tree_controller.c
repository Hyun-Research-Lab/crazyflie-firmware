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
#define DEBUG_MODULE "ILBC"
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
#include "arm_math.h"

#include "param.h"
#include "log.h"

#define ARM (0.707106781f * ARM_LENGTH)

static NominalControllerType nominal_controller = NominalControllerTypePID;

static NominalControllerFunctions nominalControllerFunctions[] = {
  {.init = 0, .update = 0},
  {.init = controllerPidInit, .update = controllerPid},
  {.init = controllerLQRInit, .update = controllerLQR},
};

LearningType learning_type = LearningTypeDisable;

// Model parameters from gp_model_params.c
extern gp_thrust_params_t thrust_params;
extern gp_torque_params_t torque_params;

// Array of random numbers from random_numbers.c
extern const int random_numbers_size;
extern const float random_numbers[];
static int rand_idx = 0;

static data_t data;
static control_t nominal_control = {0};
static control_t full_control = {0};

const arm_matrix_instance_f32 A = { 12, 12, (float32_t[]){
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
} };
const arm_matrix_instance_f32 B = { 12, 4, (float32_t[]){
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
} };

arm_matrix_instance_f32 T = { 4, 4, (float32_t[]){
  1.0f, 1.0f, 1.0f, 1.0f,
  -ARM, -ARM,  ARM,  ARM,
  -ARM,  ARM,  ARM, -ARM,
  -THRUST2TORQUE, THRUST2TORQUE, -THRUST2TORQUE, THRUST2TORQUE,
} };

void linear_dynamics_model(data_t *data, const control_t* control, const setpoint_t* setpoint, const sensorData_t* sensors, const state_t* state) {
  // States
  arm_matrix_instance_f32 x = { 12, 1, (float32_t[]){
    state->position.x, state->position.y, state->position.z, // position in the world frame (m)
    state->velocity.x, state->velocity.y, state->velocity.z, // velocity in the world frame (m/s)
    radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw), // euler angles (rad)
    radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z), // angular velocity in the body frame (rad/s)
  } };

  struct mat33 R = quat2rotmat(mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w));

  // Equilibrium state and input
  arm_matrix_instance_f32 x_eq = { 12, 1, (float32_t[]){
    setpoint->position.x, setpoint->position.y, setpoint->position.z,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
  } };
  arm_matrix_instance_f32 u_eq = { 4, 1, (float32_t[]){
    CF_MASS * GRAVITY_MAGNITUDE,
    0.0f,
    0.0f,
    0.0f
  } };

  // Control input
  arm_matrix_instance_f32 u_bar = { 4, 1, (float32_t[]){
    control->thrustSi,
    control->torqueX,
    control->torqueY,
    control->torqueZ
  } };

  // System dynamics
  float x_sub_data[12];
  arm_matrix_instance_f32 x_sub = { 12, 1, x_sub_data };

  float u_sub_data[4];
  arm_matrix_instance_f32 u_sub = { 4, 1, u_sub_data };

  float Ax_data[12];
  arm_matrix_instance_f32 Ax = { 12, 1, Ax_data };

  float Bu_data[12];
  arm_matrix_instance_f32 Bu = { 4, 1, Bu_data };

  float AxBu_data[12];
  arm_matrix_instance_f32 AxBu = { 12, 1, AxBu_data };

  float x_plus_data[12];
  arm_matrix_instance_f32 x_plus = { 12, 1, x_plus_data };

  arm_mat_sub_f32(&x, &x_eq, &x_sub);
  arm_mat_sub_f32(&u_bar, &u_eq, &u_sub);

  arm_mat_mult_f32(&A, &x_sub, &Ax);
  arm_mat_mult_f32(&B, &u_sub, &Bu);

  arm_mat_add_f32(&Ax, &Bu, &AxBu);
  arm_mat_add_f32(&x_eq, &AxBu, &x_plus);

  // Set data
  struct vec velocity_plus = mkvec(x_plus.pData[3], x_plus.pData[4], x_plus.pData[5]);
  struct vec velocity = mkvec(x.pData[3], x.pData[4], x.pData[5]);

  data->vbz_plus = mvmul(mtranspose(R), velocity_plus).z;
  data->vbz = mvmul(mtranspose(R), velocity).z;
  data->R33 = R.m[2][2];

  data->W_plus.x = x_plus.pData[10];
  data->W_plus.y = x_plus.pData[11];
  data->W_plus.z = x_plus.pData[12];
  data->W.x = x.pData[10];
  data->W.y = x.pData[11];
  data->W.z = x.pData[12];
}

static void gp_predict_thrust(const float* data, const gp_thrust_params_t* params, float* thrust) {
  float y_star = 0.0f;

  for (int sample_idx = 0; sample_idx < GP_MODEL_NUM_SAMPLES; sample_idx++) {
    // Kernel
    float sqdist = 0.0f;
    for (int data_idx = 0; data_idx < GP_MODEL_THRUST_DATA_DIM; data_idx++) {
      float diff = data[data_idx] - params->X_train[sample_idx * GP_MODEL_THRUST_DATA_DIM + data_idx];
      sqdist += (diff * diff) * params->neg_gamma[data_idx];
    }
    float rbf_kernel = expf(sqdist);

    y_star += rbf_kernel * params->alpha_times_outputscale[sample_idx];
  }
  *thrust = y_star;
}

static void gp_predict_torque(const float* data, const gp_torque_params_t* params, float* torque) {
  float y_starX = 0.0f;
  float y_starY = 0.0f;
  float y_starZ = 0.0f;

  for (int sample_idx = 0; sample_idx < GP_MODEL_NUM_SAMPLES; sample_idx++) {
    // Kernel
    float sqdistX = 0.0f;
    float sqdistY = 0.0f;
    float sqdistZ = 0.0f;
    for (int data_idx = 0; data_idx < GP_MODEL_TORQUE_DATA_DIM; data_idx++) {
      float diff = data[data_idx] - params->X_train[sample_idx * GP_MODEL_TORQUE_DATA_DIM + data_idx];
      sqdistX += (diff * diff) * params->neg_gammaX[data_idx];
      sqdistY += (diff * diff) * params->neg_gammaY[data_idx];
      sqdistZ += (diff * diff) * params->neg_gammaZ[data_idx];
    }
    float rbf_kernelX = expf(sqdistX);
    float rbf_kernelY = expf(sqdistY);
    float rbf_kernelZ = expf(sqdistZ);

    y_starX += rbf_kernelX * params->alpha_times_outputscaleX[sample_idx];
    y_starY += rbf_kernelY * params->alpha_times_outputscaleY[sample_idx];
    y_starZ += rbf_kernelZ * params->alpha_times_outputscaleZ[sample_idx];
  }
  torque[0] = y_starX;
  torque[1] = y_starY;
  torque[2] = y_starZ;
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
    control_t temp_control = nominal_control;

    nominal_control.controlMode = controlModeForceTorque;
    nominal_control.thrustSi =  temp_control.thrust                 / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueX  =  temp_control.roll   / 2.0f * ARM    / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueY  = -temp_control.pitch  / 2.0f * ARM    / UINT16_MAX * powerDistributionGetMaxThrust();
    nominal_control.torqueZ  = -temp_control.yaw    * THRUST2TORQUE / UINT16_MAX * powerDistributionGetMaxThrust();
  }

  if (learning_type == LearningTypeDisable) {
    // Return unaltered nominal control
    full_control.thrustSi = nominal_control.thrustSi;
    full_control.torqueX = nominal_control.torqueX;
    full_control.torqueY = nominal_control.torqueY;
    full_control.torqueZ = nominal_control.torqueZ;
  } else if (learning_type == LearningTypeTraining) {
    // Add exploration noise to the nominal control
    full_control.thrustSi = nominal_control.thrustSi + nominal_control.thrustSi * random_numbers[rand_idx];
    full_control.torqueX = nominal_control.torqueX + nominal_control.torqueX * random_numbers[rand_idx];
    full_control.torqueY = nominal_control.torqueY + nominal_control.torqueY * random_numbers[rand_idx];
    full_control.torqueZ = nominal_control.torqueZ + nominal_control.torqueZ * random_numbers[rand_idx];

    if (RATE_DO_EXECUTE(10, tick)) {
      if (++rand_idx >= random_numbers_size) {
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
    gp_predict_thrust(data.translation, &thrust_params, &full_control.thrustSi);
    gp_predict_torque(data.rotation, &torque_params, full_control.torque);

    // Disable learning if close to the equilibrium point
    float x[12] = {
      state->position.x, state->position.y, state->position.z,
      state->velocity.x, state->velocity.y, state->velocity.z,
      radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw),
      radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z),
    };
    float x_eq[12] = {
      setpoint->position.x, setpoint->position.y, setpoint->position.z,
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f,
    };

    if (arm_euclidean_distance_f32(x, x_eq, 12) < 0.25f) {
      learning_type = LearningTypeDisable;
      DEBUG_PRINT("Disabling learning\n");
    }
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSi = full_control.thrustSi;
  control->torqueX = full_control.torqueX;
  control->torqueY = full_control.torqueY;
  control->torqueZ = full_control.torqueZ;
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

LOG_ADD(LOG_FLOAT, full_thrust, &full_control.thrustSi)
LOG_ADD(LOG_FLOAT, full_torqueX, &full_control.torqueX)
LOG_ADD(LOG_FLOAT, full_torqueY, &full_control.torqueY)
LOG_ADD(LOG_FLOAT, full_torqueZ, &full_control.torqueZ)

LOG_ADD(LOG_FLOAT, vbz_plus, &data.vbz_plus)
LOG_ADD(LOG_FLOAT, vbz, &data.vbz)
LOG_ADD(LOG_FLOAT, R33, &data.R33)

LOG_GROUP_STOP(ILBC)