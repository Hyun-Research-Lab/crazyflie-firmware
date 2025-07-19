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
#include "commander.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"
#include "power_distribution.h"

#include "param.h"
#include "log.h"

// Model parameters from gp_model_params.c
extern const unsigned int N;
extern const unsigned int D;

extern const float X_train[];
extern const float y_mean;
extern const float y_std;
extern const float alpha[];
extern const float lengthscale;
extern const float outputscale;
extern const float noise;

void model(const state_t* state, const sensorData_t* sensors, control_t* control, float dt, state_t* next_state, sensorData_t* next_sensors) {
  // Diagonal of the inertia matrix
  struct vec J = mkvec(16.571710e-6f, 16.655602e-6f, 29.261652e-6f);
  struct vec J_inv = mkvec(1.0f/16.571710e-6f, 1.0f/16.655602e-6f, 1.0f/29.261652e-6f);

  // Current state
  struct vec x = mkvec(state->position.x, state->position.y, state->position.z); // m
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // m/s
  struct mat33 R = quat2rotmat(mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w));
  struct vec W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // rad/s

  // Control input
  float f = 0.0f; // N
  struct vec M = vzero(); // Nm
  
  if (control->controlMode == controlModeLegacy) {
    const float arm = 0.707106781f * ARM_LENGTH;
    f = control->thrust / UINT16_MAX * powerDistributionGetMaxThrust();
    M = mkvec(
      control->roll * 2.0f * STABILIZER_NR_OF_MOTORS * arm,
      -control->pitch * 2.0f * STABILIZER_NR_OF_MOTORS * arm,
      -control->yaw * STABILIZER_NR_OF_MOTORS * THRUST2TORQUE);
  
  } else if (control->controlMode == controlModeForceTorque) {
    f = control->thrustSi;
    M = mkvec(control->torqueX, control->torqueY, control->torqueZ);
  }

  // System dynamics
  struct vec x_dot = v;
  struct vec v_dot = vsub(vscl(GRAVITY_MAGNITUDE, vbasis(2)), vscl(f/CF_MASS, mvmul(R, vbasis(2))));
  struct mat33 R_dot = mmul(R, mcrossmat(W));
  struct vec W_dot = veltmul(J_inv, vsub(M, vcross(W, veltmul(J, W))));

  // Estimate the next state
  struct vec x_next = vadd(x, vscl(dt, x_dot)); // m
  struct vec v_next = vadd(v, vscl(dt, v_dot)); // m/s
  struct mat33 R_next = madd(R, mscl(dt, R_dot));
  struct vec rpy_next = quat2rpy(mat2quat(R_next)); // rad
  struct vec W_next = vadd(W, vscl(dt, W_dot)); // rad/s

  // Set the next state and sensors
  next_state->position.x = x_next.x;
  next_state->position.y = x_next.y;
  next_state->position.z = x_next.z;
  next_state->velocity.x = v_next.x;
  next_state->velocity.y = v_next.y;
  next_state->velocity.z = v_next.z;
  next_state->attitude.roll = degrees(rpy_next.x);
  next_state->attitude.pitch = degrees(rpy_next.y);
  next_state->attitude.yaw = degrees(rpy_next.z);
  next_sensors->gyro.x = degrees(W_next.x);
  next_sensors->gyro.y = degrees(W_next.y);
  next_sensors->gyro.z = degrees(W_next.z);
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  // vTaskDelay(M2T(5000)); // Wait 5 seconds before starting

  while(1) {
    // // Send a manual sepoint
    // setpoint_t setpoint;
    // setpoint.mode.roll = modeAbs;
    // setpoint.mode.pitch = modeAbs;
    // setpoint.mode.yaw = modeVelocity;
    
    // setpoint.attitude.roll = 0.0f;
    // setpoint.attitude.pitch = 0.0f;
    // setpoint.attitudeRate.yaw = 0.0f;
    // setpoint.thrust = 1000.0f;

    // commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);

    vTaskDelay(M2T(100));
  }
}

void controllerOutOfTreeInit() {
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    return;
  }
  
  // Calculate the nominal control (u_bar) using the PID controller
  control_t nominal_control;
  controllerPid(&nominal_control, setpoint, sensors, state, 0); // Set stabilizerStep to 0 so the controller is always run when called

  // Estimate the next state after a given time if the nominal control is applied
  state_t next_state;
  sensorData_t next_sensors;
  model(state, sensors, &nominal_control, 0.1f, &next_state, &next_sensors);

  // Compile the current and next states into a vector z
  float z[] = {
    next_state.position.x,
    next_state.position.y,
    next_state.position.z,
    next_state.velocity.x,
    next_state.velocity.y,
    next_state.velocity.z,
    next_state.attitude.roll,
    next_state.attitude.pitch,
    next_state.attitude.yaw,
    next_sensors.gyro.x,
    next_sensors.gyro.y,
    next_sensors.gyro.z,
    
    state->position.x,
    state->position.y,
    state->position.z,
    state->velocity.x,
    state->velocity.y,
    state->velocity.z,
    state->attitude.roll,
    state->attitude.pitch,
    state->attitude.yaw,
    sensors->gyro.x,
    sensors->gyro.y,
    sensors->gyro.z,
  };

  // c_hat function
  float f_star = 0.0f;
  for (int i = 0; i < N; i++) {
    // Kernel
    float sqdist = 0.0f;
    for (int j = 0; j < D; j++) {
      float diff = z[j] - X_train[i*D + j];
      sqdist += diff * diff;
    }
    float k = outputscale * expf(-0.5f * sqdist / (lengthscale * lengthscale));
    
    f_star += k * alpha[i];
  }

  control->thrust = f_star * y_std + y_mean;
  control->roll = nominal_control.roll;
  control->pitch = nominal_control.pitch;
  control->yaw = nominal_control.yaw;
}


// PARAM_GROUP_START(hamin)

// PARAM_ADD(PARAM_UINT8, start, &start)

// PARAM_GROUP_STOP(hamin)


// LOG_GROUP_START(hamin)

// LOG_ADD(LOG_FLOAT, thrust, &thrust_hamin)

// LOG_GROUP_STOP(hamin)