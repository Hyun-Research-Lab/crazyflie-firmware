#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

#define LQR_NUM_STATES 12

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLQR_s {
       // rows of the K matrix
    // u = -K * x
    float k1[LQR_NUM_STATES];
    float k2[LQR_NUM_STATES];
    float k3[LQR_NUM_STATES];
    float k4[LQR_NUM_STATES];

    float mass; // bicopter mass in kg

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
