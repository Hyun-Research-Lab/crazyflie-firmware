#include "stabilizer_types.h"
#include "math3d.h"
#include "platform_defaults.h"
#include "physicalConstants.h"
#include "controller_lqr.h"
#include "out_of_tree_controller.h"
#include "log.h"
#include "arm_math.h"

// const float K[48] = {
//      0.0f,     0.0f, 0.25f,     0.0f,      0.0f, 0.125f,     0.0f,     0.0f,     0.0f,   0.0f,   0.0f,   0.0f,
//      0.0f, -0.0005f,   0.0f,     0.0f, -0.0005f,    0.0f, 0.002f,     0.0f,     0.0f, 0.001f,   0.0f,   0.0f,
//   0.0005f,     0.0f,   0.0f, 0.0005f,      0.0f,    0.0f,     0.0f, 0.002f,     0.0f,   0.0f, 0.001f,   0.0f,
//      0.0f,     0.0f,   0.0f,     0.0f,      0.0f,    0.0f,     0.0f,     0.0f, 0.002f,   0.0f,   0.0f, 0.001f,
// };

#if ILBC_RATE == RATE_100_HZ
// const float K[48] = {
//   0.0f, 0.0f, 2.7196169877713547f, 0.0f, 0.0f, 0.9691499073520665f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
//   3.122713974508982e-05f, -0.0022683542442440705f, 0.0f, 2.5383181104853395e-05f, -0.0017169243882593096f, 0.0f, 0.005261657308419439f, 8.542894192999338e-05f, 7.945968543477448e-06f, 0.000828476145604941f, 1.542095653125121e-05f, 8.657152894184676e-06f, 
//   0.0022679602883009524f, -3.1214124082947316e-05f, 0.0f, 0.0017168550139871263f, -2.5372920164173952e-05f, 0.0f, 8.539639338027319e-05f, 0.005262486382881406f, 2.088168508677165e-05f, 1.5415914094423216e-05f, 0.0008288877441147072f, 2.266375005903019e-05f, 
//   1.8741464236534142e-05f, -6.00968452866516e-06f, 0.0f, 1.811160680704069e-05f, -6.130927550647393e-06f, 0.0f, 2.615973591895014e-05f, 7.374701507306406e-05f, 0.0008386931213142749f, 6.221138108895685e-06f, 1.6799568890572023e-05f, 0.0008674606267882784f, 
// };
// const float K[48] = {
//   0.0f, 0.0f, 0.8277236752931778f, 0.0f, 0.0f, 0.8542971152407296f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
//   9.959970806188733e-06f, -0.0007223481913481967f, 0.0f, 1.5235628575563026e-05f, -0.0010644116593725808f, 0.0f, 0.004150169152142651f, 6.53137008492469e-05f, 7.319663039845244e-06f, 0.0008110153746733469f, 1.4385241443555208e-05f, 8.03069733303867e-06f, 
//   0.0007222586650590812f, -9.956705872510805e-06f, 0.0f, 0.0010643539194244006f, -1.5230727390195524e-05f, 0.0f, 6.529385862950506e-05f, 0.004150741134081278f, 1.9359743411096387e-05f, 1.4381425263302817e-05f, 0.0008113582635514779f, 2.1141474239831456e-05f, 
//   7.736331762007975e-06f, -2.620942124777308e-06f, 0.0f, 1.2693272642339905e-05f, -4.384262715614914e-06f, 0.0f, 2.2757762540132182e-05f, 6.350442074458825e-05f, 0.0008386769038538535f, 6.1603561979510365e-06f, 1.663437397485836e-05f, 0.0008674444036481889f, 
// };
const arm_matrix_instance_f32 K = { 12, 12, (float32_t[]){
  0.0f, 0.0f, 0.3067375200159817f, 0.0f, 0.0f, 0.16116022058628687f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  3.122713974508982e-05f, -0.0022683542442440705f, 0.0f, 2.5383181104853395e-05f, -0.0017169243882593096f, 0.0f, 0.005261657308419439f, 8.542894192999338e-05f, 7.945968543477448e-06f, 0.000828476145604941f, 1.542095653125121e-05f, 8.657152894184676e-06f, 
  0.0022679602883009524f, -3.1214124082947316e-05f, 0.0f, 0.0017168550139871263f, -2.5372920164173952e-05f, 0.0f, 8.539639338027319e-05f, 0.005262486382881406f, 2.088168508677165e-05f, 1.5415914094423216e-05f, 0.0008288877441147072f, 2.266375005903019e-05f, 
  1.8741464236534142e-05f, -6.00968452866516e-06f, 0.0f, 1.811160680704069e-05f, -6.130927550647393e-06f, 0.0f, 2.615973591895014e-05f, 7.374701507306406e-05f, 0.0008386931213142749f, 6.221138108895685e-06f, 1.6799568890572023e-05f, 0.0008674606267882784f, 
} };
#elif ILBC_RATE == RATE_500_HZ
const float K[48] = {
  0.0f, 0.0f, 0.9721677827919112f, 0.0f, 0.0f, 1.0082000362818653f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
  8.411882765917068e-06f, -0.0029563710576889636f, 0.0f, 8.672257777822518e-06f, -0.0022236289002057147f, 0.0f, 0.006753521089422444f, 3.654154668024719e-05f, 4.859185057561486e-06f, 0.001047441708786272f, 8.284460919536656e-06f, 5.574444956366007e-06f, 
  0.0029560858675756175f, -8.407902233678245e-06f, 0.0f, 0.0022236526058212833f, -8.669067666012692e-06f, 0.0f, 3.653123652286108e-05f, 0.006754669508160135f, 1.2470506189743747e-05f, 8.282837274198974e-06f, 0.001047905592447648f, 1.426306616810874e-05f, 
  -1.818784768106032e-05f, 7.689472087480011e-06f, 0.0f, -9.055975783309182e-06f, 3.949859694674742e-06f, 0.0f, -3.4848333116099564e-06f, -6.121444352984732e-06f, 0.0009652034458521737f, 1.8787610769166347e-06f, 5.1019829883089546e-06f, 0.000994032967650575f, 
};
#endif

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
  arm_matrix_instance_f32 x = { 12, 1, (float32_t[]){
    state->position.x, state->position.y, state->position.z, // position in the world frame (m)
    state->velocity.x, state->velocity.y, state->velocity.z, // velocity in the world frame (m/s)
    radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw), // euler angles (rad)
    radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z), // angular velocity in the body frame (rad/s)
  } };

  // Equilibrium state and input
  arm_matrix_instance_f32 x_eq = { 12, 1, (float32_t[]){
    setpoint->position.x, setpoint->position.y, setpoint->position.z,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
  } };
  arm_matrix_instance_f32 u_eq = { 4, 1, (float32_t[]){
    CF_MASS * GRAVITY_MAGNITUDE,
    0.0f,
    0.0f,
    0.0f
  } };

  // Controller
  float x_sub_data[12];
  arm_matrix_instance_f32 x_sub = { 12, 1, x_sub_data };

  float Kx_sub_data[4];
  arm_matrix_instance_f32 Kx_sub = { 4, 1, Kx_sub_data };

  float u_bar_data[4];
  arm_matrix_instance_f32 u_bar = { 4, 1, u_bar_data };

  arm_mat_sub_f32(&x, &x_eq, &x_sub);
  arm_mat_mult_f32(&K, &x_sub, &Kx_sub);
  arm_mat_sub_f32(&u_eq, &Kx_sub, &u_bar);

  control->controlMode = controlModeForceTorque;
  control->thrustSi = u_bar.pData[0];
  control->torqueX = u_bar.pData[1];
  control->torqueY = u_bar.pData[2];
  control->torqueZ = u_bar.pData[3];
}
