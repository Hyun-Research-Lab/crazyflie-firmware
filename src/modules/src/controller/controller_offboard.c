#include "controller_offboard.h"

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOffboardInit() {
}

bool controllerOffboardTest() {
  return true;
}

void controllerOffboard(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  control->controlMode = controlModeForce;
  control->normalizedForces[0] = setpoint->position.x;
  control->normalizedForces[1] = setpoint->position.y;
  control->normalizedForces[2] = setpoint->position.z;
  control->normalizedForces[3] = setpoint->attitude.yaw;
}
