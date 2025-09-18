#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"

const float K[48] = {
  1.3371064528883417e-09f, -1.3371473876606874e-09f, 0.859458703203067f, 2.183297202808388e-09f, -2.183824796084697e-09f, 0.8971191677389632f, -1.572495411913522e-07f, -1.572548610277113e-07f, 1.6234187042300207e-12f, -1.1388571001544107e-09f, -1.1389106258212596e-09f, -5.4767926999965455e-15f, 
  3.6423661826602975e-05f, -0.0007265626334209173f, -8.551799100347369e-13f, 6.192880184035225e-05f, -0.00123532720342326f, -9.7410912177052e-13f, 0.006677817170164238f, 0.00033476917245462865f, 7.143521092910777e-05f, 0.0017227244298286314f, 8.636289132119145e-05f, 7.2507975209333e-05f, 
  0.000730238707917512f, -3.6423689261903554e-05f, -8.607732202260314e-13f, 0.0012415774139097693f, -6.192884821614762e-05f, -9.788719703289652e-13f, 0.0003347694200940674f, 0.006711604203569502f, 0.00017903927650030361f, 8.636295641764503e-05f, 0.0017314408105141123f, 0.00018172792592442415f, 
  7.890186026400053e-05f, -3.1481627493312904e-05f, -1.2243397361442263e-13f, 0.00013415194705976324f, -5.352626316724572e-05f, -1.4087694760152065e-13f, 0.0002893484212854772f, 0.0007251889008683337f, 0.002910731225628973f, 7.464577041578766e-05f, 0.00018708337799343288f, 0.002954436933754575f, 
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
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w); // attitude quaternion
  x.rpy = quat2rpy(q); // euler angles (rad)
  x.W = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)); // angular velocity in the body frame (rad/s)

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