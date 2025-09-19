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
  x.rpy = mkvec(radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw)); // euler angles (rad)
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