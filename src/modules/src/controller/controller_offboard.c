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
  control->controlMode = controlModeForceTorque;
  control->thrustSi = setpoint->position.x;
  control->torqueX = setpoint->position.z;
  control->torqueY = -setpoint->position.y;
  control->torqueZ = setpoint->attitude.yaw;
}
