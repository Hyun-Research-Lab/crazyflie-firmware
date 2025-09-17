#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"

// const float K[48] = { 4.28062473e-09, -4.28043903e-09,  2.59075826e+00,  3.06871923e-09, -3.06841276e-09,  9.13659945e-01, -1.25374961e-07, -1.25373867e-07, -2.85853471e-14, -8.21319300e-10, -8.21308685e-10, -9.01826749e-16,
//                       2.52935768e-04, -5.04522788e-03, -5.23699505e-12,  2.00539212e-04, -4.00009042e-03, -2.51199406e-12,  1.28851079e-02,  6.45977819e-04,  2.23572311e-04,  1.78214320e-03,  8.93454103e-05,  7.40622337e-05,
//                       5.07076797e-03, -2.52935767e-04, -5.26213635e-12,  4.02033979e-03, -2.00539212e-04, -2.52405472e-12,  6.45977818e-04,  1.29503354e-02,  5.60333415e-04,  8.93454127e-05,  1.79116483e-03,  1.85620222e-04,
//                       5.48058985e-04, -2.18674815e-04, -7.58023087e-13,  4.34526631e-04, -1.73375557e-04, -3.63524596e-13,  5.58478212e-04,  1.39969933e-03,  9.10811637e-03,  7.72433375e-05,  1.93592951e-04,  3.01722182e-03 };
// const float K[48] = { 1.68396578e-09, -1.68380066e-09,  8.27721853e-01,  2.53307990e-09, -2.53291830e-09,  8.58443914e-01, -1.23035744e-07, -1.23035393e-07,  3.44585277e-13, -8.15861264e-10, -8.15857945e-10,  2.82239041e-15,
//                       8.05048787e-05, -1.60580474e-03, -2.31709200e-12,  1.23678733e-04,
//                       -2.46697953e-03, -2.51278700e-12,  1.05924359e-02,  5.31037723e-04,
//                       2.23572286e-04,  1.76068667e-03,  8.82697093e-05,  7.40622257e-05,
//                       1.61393369e-03, -8.05048787e-05, -2.32787688e-12,  2.47946795e-03,
//                       -1.23678733e-04, -2.52488172e-12,  5.31037722e-04,  1.06460573e-02,
//                       5.60333360e-04,  8.82697117e-05,  1.76959968e-03,  1.85620204e-04,
//                       1.74437288e-04, -6.96002492e-05, -3.33800607e-13,  2.67986043e-04,
//                       -1.06926080e-04, -3.63600357e-13,  4.59107142e-04,  1.15064822e-03,
//                       9.10811636e-03,  7.63133483e-05,  1.91262145e-04,  3.01722182e-03 };
// const float K[48] = { 5.78524257e-09, -5.78576629e-09,  1.91055910e+00,  5.17271016e-09,
//   -5.17350151e-09,  9.36945096e-01, -1.52192092e-07, -1.52195754e-07,
//   -2.48380817e-13, -1.07678531e-09, -1.07682120e-09, -3.36200188e-15,
//    1.79294798e-04, -3.57633528e-03, -7.97243531e-12,  1.69522446e-04,
//   -3.38140910e-03, -4.49528153e-12,  1.20075681e-02,  6.01983514e-04,
//    2.23572302e-04,  1.77396030e-03,  8.89351690e-05,  7.40622307e-05,
//    3.59443951e-03, -1.79294798e-04, -8.01292962e-12,  3.39852657e-03,
//   -1.69522445e-04, -4.51715794e-12,  6.01983513e-04,  1.20683532e-02,
//    5.60333394e-04,  8.89351714e-05,  1.78294051e-03,  1.85620215e-04,
//    3.88494408e-04, -1.55008758e-04, -1.15440431e-12,  3.67319787e-04,
//   -1.46560114e-04, -6.50587494e-13,  5.20443088e-04,  1.30437290e-03,
//    9.10811636e-03,  7.68886665e-05,  1.92704049e-04,  3.01722182e-03 };
const float K[48] = { 2.89465744e-09, -2.89488948e-09,  8.59458703e-01,  4.38084580e-09,
  -4.38095320e-09,  8.97119168e-01, -1.50151865e-07, -1.50152076e-07,
   3.48853462e-13, -1.07001148e-09, -1.07001353e-09,  2.60691514e-15,
   8.05048787e-05, -1.60580474e-03, -4.16304842e-12,  1.23678733e-04,
  -2.46697953e-03, -4.47475501e-12,  1.05924359e-02,  5.31037723e-04,
   2.23572286e-04,  1.76068667e-03,  8.82697093e-05,  7.40622257e-05,
   1.61393369e-03, -8.05048787e-05, -4.18273169e-12,  2.47946795e-03,
  -1.23678733e-04, -4.49633147e-12,  5.31037722e-04,  1.06460573e-02,
   5.60333360e-04,  8.82697117e-05,  1.76959968e-03,  1.85620204e-04,
   1.74437288e-04, -6.96002492e-05, -6.01667444e-13,  2.67986043e-04,
  -1.06926080e-04, -6.47534310e-13,  4.59107142e-04,  1.15064822e-03,
   9.10811636e-03,  7.63133483e-05,  1.91262145e-04,  3.01722182e-03 };
static float get_K(unsigned int input_idx, unsigned int state_idx) { return K[input_idx*12 + state_idx]; }

void controllerLQRInit() {}

void controllerLQR(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
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

  // States
  full_state_t x;
  x.position = mkvec(state->position.x, state->position.y, state->position.z); // position in the world frame (m)
  x.velocity = mkvec(state->velocity.x, state->velocity.y, state->velocity.z); // velocity in the world frame (m/s)
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w); // attitude quaternion
  x.rpy = quat2rpy(q); // euler angles (rad)
  x.W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the world frame (rad/s)

  // Equilibrium state and input
  full_state_t x_eq;
  x_eq.position = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  x_eq.velocity = vzero();
  x_eq.rpy = vzero();
  x_eq.W = vzero();

  full_input_t u_eq;
  u_eq.thrust = CF_MASS * GRAVITY_MAGNITUDE;
  u_eq.torque = vzero();

  // Controller
  full_input_t u_bar;
  for (int input_idx = 0; input_idx < 4; input_idx++) {
    u_bar.u[input_idx] = u_eq.u[input_idx];
    for (int state_idx = 0; state_idx < 12; state_idx++) {
      u_bar.u[input_idx] -= get_K(input_idx, state_idx) * (x.x[state_idx] - x_eq.x[state_idx]);
    }
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSi = u_bar.thrust;
  control->torqueX = u_bar.torque.x;
  control->torqueY = u_bar.torque.y;
  control->torqueZ = u_bar.torque.z;
}