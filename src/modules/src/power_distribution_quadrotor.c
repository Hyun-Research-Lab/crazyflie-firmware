/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */


#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
    #error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

#define MEETS_THRUST_CONSTRAINTS(x) (x >= 0 && x <= THRUST_MAX)

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

const float arm = 0.707106781f * ARM_LENGTH;
// static float T_inv[16] = {
//   0.25f, -1.0f/(4.0f*arm), -1.0f/(4.0f*arm), -1.0f/(4.0f*THRUST2TORQUE),
//   0.25f, -1.0f/(4.0f*arm),  1.0f/(4.0f*arm),  1.0f/(4.0f*THRUST2TORQUE),
//   0.25f,  1.0f/(4.0f*arm),  1.0f/(4.0f*arm), -1.0f/(4.0f*THRUST2TORQUE),
//   0.25f,  1.0f/(4.0f*arm), -1.0f/(4.0f*arm),  1.0f/(4.0f*THRUST2TORQUE),
// };
static float Q[16] = {
  THRUST2TORQUE*THRUST2TORQUE + 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE, THRUST2TORQUE*THRUST2TORQUE - 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE,
  1 - THRUST2TORQUE*THRUST2TORQUE,             THRUST2TORQUE*THRUST2TORQUE + 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE, THRUST2TORQUE*THRUST2TORQUE - 2*arm*arm + 1,
  THRUST2TORQUE*THRUST2TORQUE - 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE, THRUST2TORQUE*THRUST2TORQUE + 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE,
  1 - THRUST2TORQUE*THRUST2TORQUE,             THRUST2TORQUE*THRUST2TORQUE - 2*arm*arm + 1,             1 - THRUST2TORQUE*THRUST2TORQUE, THRUST2TORQUE*THRUST2TORQUE + 2*arm*arm + 1,
};

static void solveLinearSystemPlusVector(float A[16], float b[4], float v[4], float out[4]) {
  float x[4];
  
  float augmented[4*5]; // Augmented matrix as a flat array
  int i, j, k;

  // Fill the augmented matrix
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      augmented[i*5 + j] = A[i*4 + j]; // Copy A
    }
    augmented[i*5 + 4] = b[i]; // Add b as the last column
  }

  // Gaussian elimination
  for (i = 0; i < 4; i++) {
    // Pivot normalization
    float pivot = augmented[i*5 + i];
    for (j = i; j <= 4; j++) { // Include the augmented column
      augmented[i*5 + j] /= pivot;
    }

    // Eliminate below
    for (k = i + 1; k < 4; k++) {
      float ratio = augmented[k*5 + i];
      for (j = i; j <= 4; j++) {
        augmented[k*5 + j] -= ratio * augmented[i*5 + j];
      }
    }
  }

  // Back-substitution
  for (i = 4 - 1; i >= 0; i--) {
    x[i] = augmented[i*5 + 4]; // Start with the augmented column value
    for (j = i + 1; j < 4; j++) {
      x[i] -= augmented[i*5 + j] * x[j];
    }
  }

  for (i = 0; i < 4; i++) {
    out[i] = x[i] + v[i];
  }
}

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
    DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
  }
  #endif
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  float F_star[STABILIZER_NR_OF_MOTORS];
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * ARM_LENGTH;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / THRUST2TORQUE;

  F_star[0] = thrustPart - rollPart - pitchPart - yawPart;
  F_star[1] = thrustPart - rollPart + pitchPart + yawPart;
  F_star[2] = thrustPart + rollPart + pitchPart - yawPart;
  F_star[3] = thrustPart + rollPart - pitchPart + yawPart;

  float A[16];
  float b[4];

  for (int i = 0; i < 4; i++) {
    // If the thrust is negative, set b[i] (same as v[i]) to -F_star
    if (F_star[i] < 0) {
      b[i] = -F_star[i];
    }
    // If the thrust is above max, set b[i] (same as v[i]) to THRUST_MAX - F_star
    else if (F_star[i] > THRUST_MAX) {
      b[i] = THRUST_MAX - F_star[i];
    }
    // If the thrust is within bounds, set b[i] to 0
    else {
      b[i] = 0;
    }
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // If both thrusts meet constraints values of A are from Q
      if (MEETS_THRUST_CONSTRAINTS(F_star[i]) && MEETS_THRUST_CONSTRAINTS(F_star[j])) {
        A[i*4 + j] = Q[i*4 + j];
      // If not, fill in A with ones and zeros
      } else if (i == j) {
        A[i*4 + j] = 1;
      } else {
        A[i*4 + j] = 0;
      }
      // If the row thrust meets constraints but the column thrust does not, add to b
      if (MEETS_THRUST_CONSTRAINTS(F_star[i]) && !MEETS_THRUST_CONSTRAINTS(F_star[j])) {
        b[i] -= Q[i*4 + j]*b[j];
      }
    }
  }
  
  solveLinearSystemPlusVector(A, b, F_star, motorForces);

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    motorThrustUncapped->list[motorIndex] = motorForce / THRUST_MAX * UINT16_MAX;
  }
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;
  bool isCapped = false;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, powerDistributionGetIdleThrust());
  }

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
  int32_t thrust = idleThrust;
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
  #endif
  return thrust;
}

float powerDistributionGetMaxThrust() {
  return STABILIZER_NR_OF_MOTORS * THRUST_MAX;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)