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

extern const unsigned int N;
extern const unsigned int D;

extern const float X_train[];
extern const float y_mean;
extern const float y_std;
extern const float alpha[];
extern const float lengthscale;
extern const float outputscale;
extern const float noise;

unsigned int X_train_idx = 0;

float thrust_hamin = 0.0f;

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  // vTaskDelay(M2T(5000)); // Wait 5 seconds before starting

  while(1) {
    // Send a manual sepoint
    setpoint_t setpoint;
    setpoint.mode.roll = modeAbs;
    setpoint.mode.pitch = modeAbs;
    setpoint.mode.yaw = modeAbs;
    
    setpoint.attitude.roll = 0.0f;
    setpoint.attitude.pitch = 0.0f;
    setpoint.attitude.yaw = 0.0f;
    setpoint.thrust = 1000.0f;

    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);

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
  controllerPid(control, setpoint, sensors, state, tick);
  control->thrust = thrust_hamin;
  
  if (!RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
    return;
  }
  
  if (RATE_DO_EXECUTE(10, tick)) {
    X_train_idx++;
  }

  float f_star = 0.0f;
  for (int i = 0; i < N; i++) {
    // Kernel
    float sqdist = 0.0f;
    for (int j = 0; j < D; j++) {
      float diff = X_train[X_train_idx*D + j] - X_train[i*D + j];
      sqdist += diff * diff;
    }
    float k = outputscale * expf(-0.5f * sqdist / (lengthscale * lengthscale));
    
    f_star += k * alpha[i];
  }

  thrust_hamin = f_star * y_std + y_mean;
  control->thrust = thrust_hamin;
}
