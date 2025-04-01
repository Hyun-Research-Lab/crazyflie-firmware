#include <math.h>
#include <string.h>

#include "math3d.h"
#include "controller_lqr.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

static controllerLQR_t g_self = {
  .k1 = {3.53111077e-03, -4.92211298e-02, -3.02485384e-20,
    7.06552086e-01,  5.16808629e-02, -1.10221213e-01,
    7.04655342e-03, -9.75340536e-02, -4.53728076e-20,
    2.36722617e-01,  1.78872061e-02, -1.15099361e-01},
  
  .k2 = {-3.55151647e-03, -4.92205836e-02,  1.51243958e-20,
    7.06544898e-01, -5.19758074e-02,  1.10219219e-01,
    -7.08713751e-03, -9.75330014e-02,  4.53731874e-20,
    2.36720521e-01, -1.79860848e-02,  1.15097368e-01},

  .k3 = {-6.01750738e-01,  1.20554783e-04,  7.05278161e-01,
    -1.65228193e-03, -8.37037400e+00, -1.00174833e-01,
    -1.17855575e+00,  2.35107424e-04,  8.54844175e-01,
    -5.19023055e-04, -2.72183977e+00, -1.01031736e-01},


  .k4 = {6.01750738e-01, -1.20554783e-04,  7.05278161e-01,
    1.65228193e-03,  8.37037400e+00,  1.00174833e-01,
    1.17855575e+00, -2.35107424e-04,  8.54844175e-01,
    5.19023055e-04,  2.72183977e+00,  1.01031736e-01}

};

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLQRReset(controllerLQR_t* self)
{
}

void controllerLQRInit(controllerLQR_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  controllerLQRReset(self);
}

bool controllerLQRTest(controllerLQR_t* self)
{
  return true;
}

static float px, py, pz;
static float vx, vy, vz;
static float roll, pitch, yaw;
static float wx, wy, wz;

static float m1_out;
static float m4_out;
static float s1_out;
static float s2_out;

static float setpoint_px = 0;
static float setpoint_py = 0;
static float setpoint_pz = 1.0;

static int lqr_count = 0;
void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(RATE_1000_HZ, tick)) {
    return;
  }

  lqr_count ++;

  // uint64_t startTime = usecTimestamp();
  // float dt = (float)(1.0f/1000.0f);
  control->controlMode = controlModeLQR;

  // control->motorLeft_N = 3.3/2;
  // control->motorRight_N = 3.3/2;
  // control->servoLeft_deg = 0;
  // control->servoRight_deg = 0;

  float x[12] = {state->position.x, state->position.y, state->position.z,
                 radians(state->attitude.pitch), radians(state->attitude.roll), radians(state->attitude.yaw),
                 state->velocity.x, state->velocity.y, state->velocity.z,
                 radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)};

  x[9] = 0;
  x[10] = 0;
  x[11] = 0;

  // float xd[12] = {setpoint->position.x, setpoint->position.y, setpoint->position.z, 
  //                 0, 0, 0, 
  //                 0, 0, 0, 
  //                 0, 0, 0};

  float xd[12] = {setpoint_px, setpoint_py, setpoint_pz,
    0, 0, 0, 
    0, 0, 0, 
    0, 0, 0};
    
  roll = state->attitude.roll;
  pitch = state->attitude.pitch;
  yaw = state->attitude.yaw;

  wx = sensors->gyro.x;
  wy = sensors->gyro.y;
  wz = sensors->gyro.z;

  px = state->position.x;
  py = state->position.y;
  pz = state->position.z;

  vx = state->velocity.x;
  vy = state->velocity.y;
  vz = state->velocity.z;

  // do the matrix multiplication
  float tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k4[i] * (x[i] - xd[i]);
  }
  control->motorLeft_N = tmp + 9.81f*0.320f/2.0f;
  m1_out = control->motorLeft_N;
  // control->motorLeft_N = 0.0;
  // control->motorLeft_N = 3.3/2;

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k3[i] * (x[i] - xd[i]);
  }
  control->motorRight_N = tmp + 9.81f*0.320f/2.0f;
  m4_out = control->motorRight_N;
  // control->motorRight_N = 0.0f;
  // control->motorRight_N = 3.3/2;

  // ros2 launch crazyflie_examples launch.py script:=hello_world backend:=cflib

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k2[i] * (x[i] - xd[i]);
  }
  // control->servoLeft_deg = (lqr_count / 10) % 90 - 45; // degrees(tmp);
  control->servoLeft_deg = -degrees(tmp);
  s1_out = -degrees(tmp);

  tmp = 0;
  for (int i = 0; i < 12; i++) {
    tmp += -self->k1[i] * (x[i] - xd[i]);
  }
  // control->servoRight_deg = (lqr_count / 10) % 90 - 45; // degrees(tmp);
  control->servoRight_deg = degrees(tmp);
  s2_out = degrees(tmp);

  // struct vec dessnap = vzero();
  // Address inconsistency in firmware where we need to compute our own desired yaw angle
  // Rate-controlled YAW is moving YAW angle setpoint
  // float desiredYaw = 0; //rad
  // if (setpoint->mode.yaw == modeVelocity) {
  //   desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  // } else if (setpoint->mode.yaw == modeAbs) {
  //   desiredYaw = radians(setpoint->attitude.yaw);
  // } else if (setpoint->mode.quat == modeAbs) {
  //   struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
  //   self->rpy_des = quat2rpy(setpoint_quat);
  //   desiredYaw = self->rpy_des.z;
  // }

  // // Position controller
  // if (   setpoint->mode.x == modeAbs
  //     || setpoint->mode.y == modeAbs
  //     || setpoint->mode.z == modeAbs) {
  //   struct vec pos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  //   struct vec vel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  //   struct vec acc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);
  //   struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  //   struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  //   // errors
  //   struct vec pos_e = vclampscl(vsub(pos_d, statePos), -self->Kpos_P_limit, self->Kpos_P_limit);
  //   struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -self->Kpos_D_limit, self->Kpos_D_limit);
  //   self->i_error_pos = vadd(self->i_error_pos, vscl(dt, pos_e));
  //   self->p_error = pos_e;
  //   self->v_error = vel_e;

  //   struct vec F_d = vadd4(
  //     acc_d,
  //     veltmul(self->Kpos_D, vel_e),
  //     veltmul(self->Kpos_P, pos_e),
  //     veltmul(self->Kpos_I, self->i_error_pos));

  //   struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  //   struct mat33 R = quat2rotmat(q);
  //   struct vec z  = vbasis(2);
  //   control->thrustSi = self->mass*vdot(F_d , mvmul(R, z));
  //   self->thrustSi = control->thrustSi;
  //   // Reset the accumulated error while on the ground
  //   if (control->thrustSi < 0.01f) {
  //     controllerLQRReset(self);
  //   }

  //   // Compute Desired Rotation matrix
  //   float normFd = control->thrustSi;

  //   struct vec xdes = vbasis(0);
  //   struct vec ydes = vbasis(1);
  //   struct vec zdes = vbasis(2);
   
  //   if (normFd > 0) {
  //     zdes = vnormalize(F_d);
  //   } 
  //   struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
  //   struct vec zcrossx = vcross(zdes, xcdes);
  //   float normZX = vmag(zcrossx);

  //   if (normZX > 0) {
  //     ydes = vnormalize(zcrossx);
  //   } 
  //   xdes = vcross(ydes, zdes);
    
  //   self->R_des = mcolumns(xdes, ydes, zdes);

  // } else {
  //   if (setpoint->mode.z == modeDisable) {
  //     if (setpoint->thrust < 1000) {
  //         control->controlMode = controlModeForceTorque;
  //         control->thrustSi  = 0;
  //         control->torque[0] = 0;
  //         control->torque[1] = 0;
  //         control->torque[2] = 0;
  //         controllerLQRReset(self);
  //         return;
  //     }
  //   }
  //   const float max_thrust = powerDistributionGetMaxThrust(); // N
  //   control->thrustSi = setpoint->thrust / UINT16_MAX * max_thrust;

  //   struct quat q = rpy2quat(mkvec(
  //       radians(setpoint->attitude.roll),
  //       -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
  //       desiredYaw));
  //   self->R_des = quat2rotmat(q);
  // }

  // // Attitude controller

  // // current rotation [R]
  // struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  // self->rpy = quat2rpy(q);
  // struct mat33 R = quat2rotmat(q);

  // // desired rotation [Rdes]
  // struct quat q_des = mat2quat(self->R_des);
  // self->rpy_des = quat2rpy(q_des);

  // // rotation error
  // struct mat33 eRM = msub(mmul(mtranspose(self->R_des), R), mmul(mtranspose(R), self->R_des));

  // struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // // angular velocity
  // self->omega = mkvec(
  //   radians(sensors->gyro.x),
  //   radians(sensors->gyro.y),
  //   radians(sensors->gyro.z));

  // // Compute desired omega
  // struct vec xdes = mcolumn(self->R_des, 0);
  // struct vec ydes = mcolumn(self->R_des, 1);
  // struct vec zdes = mcolumn(self->R_des, 2);
  // struct vec hw = vzero();
  // // Desired Jerk and snap for now are zeros vector
  // struct vec desJerk = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);

  // if (control->thrustSi != 0) {
  //   struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
  //   hw = vscl(self->mass/control->thrustSi, tmp);
  // }
  // struct vec z_w = mkvec(0,0,1); 
  // float desiredYawRate = radians(setpoint->attitudeRate.yaw) * vdot(zdes,z_w);
  // struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
  
  // self->omega_r = mvmul(mmul(mtranspose(R), self->R_des), omega_des);

  // struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // // Integral part on angle
  // self->i_error_att = vadd(self->i_error_att, vscl(dt, eR));

  // // compute moments
  // // M = -kR eR - kw ew + w x Jw - J(w x wr)
  // self->u = vadd4(
  //   vneg(veltmul(self->KR, eR)),
  //   vneg(veltmul(self->Komega, omega_error)),
  //   vneg(veltmul(self->KI, self->i_error_att)),
  //   vcross(self->omega, veltmul(self->J, self->omega)));

  // control->controlMode = controlModeForceTorque;
  // control->torque[0] = self->u.x;
  // control->torque[1] = self->u.y;
  // control->torque[2] = self->u.z;

  // ticks = usecTimestamp() - startTime;
}


void controllerLQRFirmwareInit(void)
{
  controllerLQRInit(&g_self);
}

bool controllerLQRFirmwareTest(void)
{
  return true;
}

void controllerLQRFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLQR(&g_self, control, setpoint, sensors, state, tick);
}

#include "log.h"
#include "param.h"
LOG_GROUP_START(ctrlLQR)

LOG_ADD(LOG_FLOAT, px, &px)
LOG_ADD(LOG_FLOAT, py, &py)
LOG_ADD(LOG_FLOAT, pz, &pz)
LOG_ADD(LOG_FLOAT, vx, &vx)
LOG_ADD(LOG_FLOAT, vy, &vy)
LOG_ADD(LOG_FLOAT, vz, &vz)
LOG_ADD(LOG_FLOAT, wx, &wx)
LOG_ADD(LOG_FLOAT, wy, &wy)
LOG_ADD(LOG_FLOAT, wz, &wz)
LOG_ADD(LOG_FLOAT, roll, &roll)
LOG_ADD(LOG_FLOAT, pitch, &pitch)
LOG_ADD(LOG_FLOAT, yaw, &yaw)

LOG_ADD(LOG_FLOAT, m1_out, &m1_out)
LOG_ADD(LOG_FLOAT, m4_out, &m4_out)
LOG_ADD(LOG_FLOAT, s1_out, &s1_out)
LOG_ADD(LOG_FLOAT, s2_out, &s2_out)

// LOG_ADD(LOG_FLOAT,Kpos_Px, &g_self.Kpos_P.x)
// LOG_ADD(LOG_FLOAT,Kpos_Py, &g_self.Kpos_P.y)
// LOG_ADD(LOG_FLOAT,Kpos_Pz, &g_self.Kpos_P.z)
// LOG_ADD(LOG_FLOAT,Kpos_Dx, &g_self.Kpos_D.x)
// LOG_ADD(LOG_FLOAT,Kpos_Dy, &g_self.Kpos_D.y)
// LOG_ADD(LOG_FLOAT,Kpos_Dz, &g_self.Kpos_D.z)

LOG_GROUP_STOP(ctrlLQR)

PARAM_GROUP_START(ctrlLQRParam)
PARAM_ADD(PARAM_FLOAT, setpoint_px, &setpoint_px)
PARAM_ADD(PARAM_FLOAT, setpoint_py, &setpoint_py)
PARAM_ADD(PARAM_FLOAT, setpoint_pz, &setpoint_pz)
PARAM_GROUP_STOP(ctrlLQRParam)