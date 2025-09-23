#ifndef __CONTROLLER_LQR__
#define __CONTROLLER_LQR__

#include "math3d.h"
#include "stabilizer_types.h"

#define ADD_NOISE_LQR

typedef union full_state_s {
  struct {
    struct vec position;
    struct vec velocity;
    struct vec rpy;
    struct vec angularVelocity;
  };
  float full[12];
} full_state_t;

typedef union full_input_s {
  struct {
    float thrust;  // N
    struct vec torque; // Nm
  };
  float full[4];
} full_input_t;

void controllerLQRInit();
void controllerLQR(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t tick);

#endif /* __CONTROLLER_LQR__ */