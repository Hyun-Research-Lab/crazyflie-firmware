#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

#define LQR_NUM_STATES 12

#ifndef LQR_NUM_STATES
#error "LQR_NUM_STATES must be defined to 6 or 12 in the build configuration"
#endif

#if ((LQR_NUM_STATES != 12) && (LQR_NUM_STATES != 6))
#error "LQR_NUM_STATES must be either 6 or 12"
#endif

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLQR_s {
       // rows of the K matrix
    // u = -K * x
    float k1[LQR_NUM_STATES];
    float k2[LQR_NUM_STATES];
    float k3[LQR_NUM_STATES];
    float k4[LQR_NUM_STATES];

    // Logging variables
    struct vec rpy;
    struct vec rpy_des;
    struct mat33 R_des;
    struct vec omega;
    struct vec omega_r;
    struct vec u;
} controllerLQR_t;


void controllerLQRInit(controllerLQR_t* self);
void controllerLQRReset(controllerLQR_t* self);
void controllerLQR(controllerLQR_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

void controllerLQRFirmwareInit(void);
bool controllerLQRFirmwareTest(void);
void controllerLQRFirmware(control_t *control, const setpoint_t *setpoint,
                                        const sensorData_t *sensors,
                                        const state_t *state,
                                        const uint32_t tick);

#endif //__CONTROLLER_LQR_H__
