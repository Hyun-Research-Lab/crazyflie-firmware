#ifndef __OUT_OF_TREE_CONTROLLER_H__
#define __OUT_OF_TREE_CONTROLLER_H__

#include "stabilizer_types.h"

typedef enum {
  NominalControllerTypeNone,
  NominalControllerTypePID,
  NominalControllerTypeLQR,
  NominalControllerTypeCount
} NominalControllerType;

typedef struct {
  void (*init)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
} NominalControllerFunctions;

typedef struct data_s {
  union {
    struct {
      float vbz_plus;
      float vbz;
      float R33;
    };
    float translation[3];
  };
  union {
    struct {
      float Wx_plus;
      float Wy_plus;
      float Wz_plus;
      float Wx;
      float Wy;
      float Wz;
    };
    float rotation[6];
  };
} data_t;

void controllerOutOfTreeInit();
bool controllerOutOfTreeTest();
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);

#endif // __OUT_OF_TREE_CONTROLLER_H__