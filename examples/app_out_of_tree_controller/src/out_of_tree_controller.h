#ifndef __OUT_OF_TREE_CONTROLLER_H__
#define __OUT_OF_TREE_CONTROLLER_H__

#include "stabilizer_types.h"

#define ILBC_RATE RATE_500_HZ

typedef enum {
  NominalControllerTypeNone,
  NominalControllerTypePID,
  NominalControllerTypeLQR,
  NominalControllerTypeCount
} NominalControllerType;

typedef struct {
  void (*init)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t tick);
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
      struct vec W_plus;
      struct vec W;
    };
    float rotation[6];
  };
} data_t;

void controllerOutOfTreeInit();
bool controllerOutOfTreeTest();
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);

#endif // __OUT_OF_TREE_CONTROLLER_H__