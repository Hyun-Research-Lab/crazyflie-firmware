#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"

const float K[48] = { 4.28062473e-09, -4.28043903e-09,  2.59075826e+00,  3.06871923e-09, -3.06841276e-09,  9.13659945e-01, -1.25374961e-07, -1.25373867e-07, -2.85853471e-14, -8.21319300e-10, -8.21308685e-10, -9.01826749e-16,
                      2.52935768e-04, -5.04522788e-03, -5.23699505e-12,  2.00539212e-04, -4.00009042e-03, -2.51199406e-12,  1.28851079e-02,  6.45977819e-04,  2.23572311e-04,  1.78214320e-03,  8.93454103e-05,  7.40622337e-05,
                      5.07076797e-03, -2.52935767e-04, -5.26213635e-12,  4.02033979e-03, -2.00539212e-04, -2.52405472e-12,  6.45977818e-04,  1.29503354e-02,  5.60333415e-04,  8.93454127e-05,  1.79116483e-03,  1.85620222e-04,
                      5.48058985e-04, -2.18674815e-04, -7.58023087e-13,  4.34526631e-04, -1.73375557e-04, -3.63524596e-13,  5.58478212e-04,  1.39969933e-03,  9.10811637e-03,  7.72433375e-05,  1.93592951e-04,  3.01722182e-03 };
static float get_K(unsigned int input_idx, unsigned int state_idx) { return K[input_idx*12 + state_idx]; }

void controllerLQR(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  // Disable controller in manual mode if thrust is low
  if (setpoint->mode.z == modeDisable && setpoint->thrust < 1000.0f) {
    control->controlMode = controlModeLegacy;
    control->thrust = 0.0f;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
    return;
  }

  // States
  struct vec p = mkvec(state->position.x, state->position.y, state->position.z); // position in the world frame (m)
  struct vec v = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // velocity in the world frame (m/s)
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w); // attitude quaternion
  struct vec rpy = quat2rpy(q); // euler angles (rad)
  struct mat33 R = quat2rotmat(q); // rotation matrix
  struct vec W = mvmul(R, mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z))); // angular velocity in the world frame (rad/s)
  float x[12] = { p.x, p.y, p.z,
                  v.x, v.y, v.z,
                  rpy.x, rpy.y, rpy.z,
                  W.x, W.y, W.z };

  // Equilibrium state and input
  float x_eq[12] = { setpoint->position.x, setpoint->position.y, setpoint->position.z,  // position (m)
                     0.0f, 0.0f, 0.0f,  // velocity (m/s)
                     0.0f, 0.0f, 0.0f,  // attitude (rad)
                     0.0f, 0.0f, 0.0f}; // angular velocity (rad/s)
  float u_eq[4] = {CF_MASS * GRAVITY_MAGNITUDE, 0.0f, 0.0f, 0.0f}; // thrust (N), torque (Nm)

  // Controller
  float u_bar[4] = {0};
  for (int input_idx = 0; input_idx < 4; input_idx++) {
    u_bar[input_idx] = u_eq[input_idx];
    for (int state_idx = 0; state_idx < 12; state_idx++) {
      u_bar[input_idx] -= get_K(input_idx, state_idx) * (x[state_idx] - x_eq[state_idx]);
    }
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSi = u_bar[0];
  control->torqueX = u_bar[1];
  control->torqueY = u_bar[2];
  control->torqueZ = u_bar[3];
}