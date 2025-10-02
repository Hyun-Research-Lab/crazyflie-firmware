#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"
#include "out_of_tree_controller.h"
#include "log.h"

// const float K[48] = {
//     0.0f,    0.0f, 1.0f,   0.0f,    0.0f, 0.5f,   0.0f,     0.0f,    0.0f,   0.0f,   0.0f,   0.0f,
//     0.0f, -0.002f, 0.0f,   0.0f, -0.001f, 0.0f, 0.007f,     0.0f,    0.0f, 0.002f,   0.0f,   0.0f,
//   0.002f,    0.0f, 0.0f, 0.001f,    0.0f, 0.0f,   0.0f,   0.007f,    0.0f,   0.0f, 0.002f,   0.0f,
//     0.0f,    0.0f, 0.0f,   0.0f,    0.0f, 0.0f,   0.0f,     0.0f,  0.007f,   0.0f,   0.0f, 0.002f,
// };

#if ILBC_RATE == RATE_100_HZ
const float K[48] = {
  0.0f, 0.0f, 2.7196169877713547f, 0.0f, 0.0f, 0.9691499073520665f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  3.122713974508982e-05f, -0.0022683542442440705f, 0.0f, 2.5383181104853395e-05f, -0.0017169243882593096f, 0.0f, 0.005261657308419439f, 8.542894192999338e-05f, 7.945968543477448e-06f, 0.000828476145604941f, 1.542095653125121e-05f, 8.657152894184676e-06f, 
  0.0022679602883009524f, -3.1214124082947316e-05f, 0.0f, 0.0017168550139871263f, -2.5372920164173952e-05f, 0.0f, 8.539639338027319e-05f, 0.005262486382881406f, 2.088168508677165e-05f, 1.5415914094423216e-05f, 0.0008288877441147072f, 2.266375005903019e-05f, 
  1.8741464236534142e-05f, -6.00968452866516e-06f, 0.0f, 1.811160680704069e-05f, -6.130927550647393e-06f, 0.0f, 2.615973591895014e-05f, 7.374701507306406e-05f, 0.0008386931213142749f, 6.221138108895685e-06f, 1.6799568890572023e-05f, 0.0008674606267882784f, 
};
#elif ILBC_RATE == RATE_500_HZ
const float K[48] = {
  0.0f, 0.0f, 0.9721677827919112f, 0.0f, 0.0f, 1.0082000362818653f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  8.411882765917068e-06f, -0.0029563710576889636f, 0.0f, 8.672257777822518e-06f, -0.0022236289002057147f, 0.0f, 0.006753521089422444f, 3.654154668024719e-05f, 4.859185057561486e-06f, 0.001047441708786272f, 8.284460919536656e-06f, 5.574444956366007e-06f, 
  0.0029560858675756175f, -8.407902233678245e-06f, 0.0f, 0.0022236526058212833f, -8.669067666012692e-06f, 0.0f, 3.653123652286108e-05f, 0.006754669508160135f, 1.2470506189743747e-05f, 8.282837274198974e-06f, 0.001047905592447648f, 1.426306616810874e-05f, 
  -1.818784768106032e-05f, 7.689472087480011e-06f, 0.0f, -9.055975783309182e-06f, 3.949859694674742e-06f, 0.0f, -3.4848333116099564e-06f, -6.121444352984732e-06f, 0.0009652034458521737f, 1.8787610769166347e-06f, 5.1019829883089546e-06f, 0.000994032967650575f, 
};
#endif

static float get_K(unsigned int input_idx, unsigned int state_idx) { return K[input_idx*12 + state_idx]; }

void controllerLQRInit() {}

void controllerLQR(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t tick) {
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
  x.rpy = mkvec(radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw)); // euler angles (rad)
  x.angularVelocity = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the body frame (rad/s)

  // Equilibrium state and input
  full_state_t x_eq;
  x_eq.position = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  x_eq.velocity = vzero();
  x_eq.rpy = vzero();
  x_eq.angularVelocity = vzero();

  full_input_t u_eq;
  u_eq.thrust = CF_MASS * GRAVITY_MAGNITUDE;
  u_eq.torque = vzero();

  // Controller
  full_input_t u_bar;
  for (int input_idx = 0; input_idx < 4; input_idx++) {
    u_bar.full[input_idx] = u_eq.full[input_idx];
    for (int state_idx = 0; state_idx < 12; state_idx++) {
      u_bar.full[input_idx] -= get_K(input_idx, state_idx) * (x.full[state_idx] - x_eq.full[state_idx]);
    }
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSi = u_bar.thrust;
  control->torqueX = u_bar.torque.x;
  control->torqueY = u_bar.torque.y;
  control->torqueZ = u_bar.torque.z;
}
