#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"

/*
All 1s, 20 Hz
const float K[48] = {
  0.0f, 0.0f, 0.49369755490022854f, 0.0f, 0.0f, 0.5266010913858661f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  1.4512957571502181e-05f, -0.0002894834510801597f, 0.0f, 2.1800634609584715e-05f, -0.00043484747051105056f, 0.0f, 0.001784059025576896f, 8.944198234555553e-05f, 1.4015152154223512e-05f, 0.0003742729991675993f, 1.8763796180747026e-05f, 1.471591019761516e-05f, 
  0.00029094891962380974f, -1.4512957563900716e-05f, 0.0f, 0.0004370488237206825f, -2.1800634594184826e-05f, 0.0f, 8.944198226166003e-05f, 0.0017930905709261623f, 3.512577419846815e-05f, 1.8763796176232642e-05f, 0.0003761677029917671f, 3.688206392906393e-05f, 
  3.144677849702511e-05f, -1.2547236032847649e-05f, 0.0f, 4.723776880938442e-05f, -1.8847826840509666e-05f, 0.0f, 7.73274299596053e-05f, 0.00019380352329135108f, 0.0005709588660211638f, 1.62223171470091e-05f, 4.06575286676404e-05f, 0.0005995068189655324f, 
};

All 1s, 100 Hz
const float K[48] = {
  0.0f, 0.0f, 0.8594602026483931f, 0.0f, 0.0f, 0.8928130452220121f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  8.085585079641215e-05f, -0.0016128054853015697f, 0.0f, 0.00011817476876552316f, -0.0023571937609426167f, 0.0f, 0.008987666903611163f, 0.0004505847609541413f, 7.146912488153921e-05f, 0.001701720215888892f, 8.53135010860801e-05f, 7.218382832899973e-05f, 
  0.0016209698756128944f, -8.085585067123068e-05f, 0.0f, 0.0023691264159964305f, -0.0001181747686215901f, 0.0f, 0.0004505847604633531f, 0.009033164555838417f, 0.0001791211919998362f, 8.531350099206127e-05f, 0.0017103347237096315f, 0.00018091243268482138f, 
  0.00017519774136262953f, -6.990366843890981e-05f, 0.0f, 0.0002560600550000604f, -0.00010216762602767939f, 0.0f, 0.00038955170437694227f, 0.0009763232520309939f, 0.002911581959193498f, 7.375754869046831e-05f, 0.0001848566106837698f, 0.0029406980295845287f, 
};
*/
const float K[48] = {
    0.0f,    0.0f, 1.0f,   0.0f,    0.0f, 0.5f,   0.0f,     0.0f,    0.0f,   0.0f,   0.0f,   0.0f,
    0.0f, -0.002f, 0.0f,   0.0f, -0.001f, 0.0f, 0.007f,     0.0f,    0.0f, 0.002f,   0.0f,   0.0f,
  0.002f,    0.0f, 0.0f, 0.001f,    0.0f, 0.0f,   0.0f,   0.007f,    0.0f,   0.0f, 0.002f,   0.0f,
    0.0f,    0.0f, 0.0f,   0.0f,    0.0f, 0.0f,   0.0f,     0.0f,  0.007f,   0.0f,   0.0f, 0.002f,
};
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