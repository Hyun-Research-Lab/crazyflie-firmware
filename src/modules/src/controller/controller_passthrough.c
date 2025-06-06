

#include "controller_passthrough.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"


static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;

void controllerPassthroughInit(void)
{
}

bool controllerPassthroughTest(void)
{
  return true;
}

// static int passthroughCount = 0;
void controllerPassthrough(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;
  control->thrust = setpoint->thrust / 65000.0f; // normalized 0 to 1
  control->roll = setpoint->attitude.roll;
  control->pitch = setpoint->attitude.pitch;

  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;

  // if (passthroughCount % 1000 == 0) {
  //   DEBUG_PRINT("setpoint->thrust: %f\n", (double) setpoint->thrust);
  // }
  // passthroughCount++;
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
// LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
// /**
//  * @brief Gyro roll measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
// /**
//  * @brief Gyro pitch measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
// /**
//  * @brief Yaw  measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
// /**
//  * @brief Acceleration in the zaxis in G-force
//  */
// LOG_ADD(LOG_FLOAT, accelz, &accelz)
// /**
//  * @brief Thrust command without (tilt)compensation
//  */
// LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
// /**
//  * @brief Desired roll setpoint
//  */
// LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
// /**
//  * @brief Desired pitch setpoint
//  */
// LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
// /**
//  * @brief Desired yaw setpoint
//  */
// LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
// /**
//  * @brief Desired roll rate setpoint
//  */
// LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
// /**
//  * @brief Desired pitch rate setpoint
//  */
// LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
// /**
//  * @brief Desired yaw rate setpoint
//  */
// LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)
