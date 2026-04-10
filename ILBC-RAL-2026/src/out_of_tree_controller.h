#ifndef OUT_OF_TREE_CONTROLLER_H
#define OUT_OF_TREE_CONTROLLER_H

#include "stabilizer_types.h"

#define ILBC_RATE RATE_100_HZ

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

typedef enum {
  LearningTypeDisable,
  LearningTypeLinearModel,
  LearningTypeNonlinearModel,
  LearningTypeTraining,
} LearningType;

void controllerOutOfTreeInit();
bool controllerOutOfTreeTest();
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t tick);

#endif // OUT_OF_TREE_CONTROLLER_H