/**
 *  _   _                     _           _
 * | | | |                   | |         | |
 * | |_| |_   _ _   _ _ __   | |     __ _| |__
 * |  _  | | | | | | | '_ \  | |    / _` | '_ \
 * | | | | |_| | |_| | | | | | |___| (_| | |_) |
 * \_| |_/\__, |\__,_|_| |_| \_____/\__,_|_.__/
 *         __/ |
 *        |___/
 *
 * @file power_distribution_bicopter.c
 * @author Logan Dihel
 * @brief Tells the motors what to do given the thrust and torque controller outputs
 * @details This file was modified from power_distribution_quadrotor.c
 */
#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"
#include "bicopterdeck.h"
#include "pm.h"
#include "math3d.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
#error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

#if defined(CONFIG_PROPELLER_WINDANCER) && defined(CONFIG_BATTERY_550)
static float pwmToThrustA = 0.04415f;
static float pwmToThrustB = 0.04359f;
#elif defined(CONFIG_PROPELLER_HURRICANE) && defined(CONFIG_BATTERY_550)
static float pwmToThrustA = 0.06261942f;
static float pwmToThrustB = 0.22547572f;
#elif defined(CONFIG_PROPELLER_HURRICANE) && defined(CONFIG_BATTERY_1550)
static float pwmToThrustA = 0.05163731f;
static float pwmToThrustB = 0.32107592f;
#else
#error "Unsupported propeller and battery configuration"
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

int powerDistributionMotorType(uint32_t id)
{
    return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
    return 0;
}

void powerDistributionInit(void)
{
#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
        DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
    }
#endif
}

bool powerDistributionTest(void)
{
    bool pass = true;
    return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
        return minThrust;
    }

    return thrust;
}

// logging
static float m1_pwm;
static float m4_pwm;

// trim parameters
static float pwmAdjust1 = 1.0f;
static float pwmAdjust4 = 1.0f;

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
    // DSHOT
    // control->thrust is in range [0, 1]
    // motorThrustUncapped->motors.m1 is in range [0, UINT16_MAX]
    motorThrustUncapped->motors.m1 = control->thrust * UINT16_MAX; // left
    motorThrustUncapped->motors.m4 = control->thrust * UINT16_MAX; // right

    // pitch and roll are already in degrees
    s_servo1_angle = control->pitch;
    s_servo2_angle = control->roll;
}

// define Apinv matrix as a bunch of arrays
static float a1[6] = {0.00000000f, 0.10868491f, 0.00000000f,
    0.98804466f, 0.00000000f, 0.00000000f};

static float a2[6] = {0.71414565f, -0.00000000f, -7.74666470f,
    -0.00000000f, 0.00000000f, 0.00000000f};

static float a3[6] = {0.00000000f, 0.00000000f, 0.00000000f,
    0.00000000f, 0.00000000f, 1.00000000f};

static float a4[6] = {7.74666470f, -0.00000000f, 0.71414565f,
    -0.00000000f, 0.00000000f, 0.00000000f};
static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {

    // v = Apinv * [torqueX; torqueY; torqueZ; 0; 0; thrustSi]
    

    float v1 = a1[0] * control->torqueX + a1[1] * control->torqueY + a1[2] * control->torqueZ + a1[5] * control->thrustSi;
    float v2 = a2[0] * control->torqueX + a2[1] * control->torqueY + a2[2] * control->torqueZ + a2[5] * control->thrustSi;
    float v3 = a3[0] * control->torqueX + a3[1] * control->torqueY + a3[2] * control->torqueZ + a3[5] * control->thrustSi;
    float v4 = a4[0] * control->torqueX + a4[1] * control->torqueY + a4[2] * control->torqueZ + a4[5] * control->thrustSi;

    // calculate the actual bicopter inputs
    float force_left = 0.25f * sqrtf((v1+v2)*(v1+v2) + (v3+v4)*(v3+v4));
    float force_right = 0.25f * sqrtf((v1-v2)*(v1-v2) + (v3-v4)*(v3-v4));

    double uncapped_left = (double) degrees(asinf((v1+v2)/(2.0f*force_left)));
    double uncapped_right = (double) degrees(asinf((v3+v4)/(2.0f*force_right)));
    
    // set the servo angle
    s_servo1_angle = fmax(-15.0, fmin(15.0, uncapped_left));
    s_servo2_angle = fmax(-15.0, fmin(15.0, uncapped_right));

    // set the PWM values for the BLDC motors
    #if defined(CONFIG_BICOPTER_NAME_MELONCOPTER)
    float m1_force = force_right;
    float m4_force = force_left;
    #elif defined(CONFIG_BICOPTER_NAME_REDCOPTER)
    float m1_force = force_left;
    float m4_force = force_right;
    #endif

    float y1 = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * m1_force)) / (2.0f * pwmToThrustA);
    float y4 = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * m4_force)) / (2.0f * pwmToThrustA);
    
    #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
    float vBatt = pmGetBatteryVoltage();
    #else
    float vBatt = 14.8f; // 4S battery nominal voltage
    #endif

    float pwm1 = y1 / vBatt;
    float pwm4 = y4 / vBatt;

    // maximum pwm value is 1.0
    // pwmAdjust is a parameter we can set to scale up or down all thrusts
    m1_pwm = fmin(pwm1 * pwmAdjust1, 1.0f);
    m4_pwm = fmin(pwm4 * pwmAdjust4, 1.0f);

    motorThrustUncapped->motors.m1 = m1_pwm * UINT16_MAX; // left motor
    motorThrustUncapped->motors.m4 = m4_pwm * UINT16_MAX; // right motor
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // Not implemented yet
}

static void powerDistributionWrench(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // Not implemented yet
}

static void powerDistributionLQR(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
    // get the desired force to be produced by each motor
    #if defined(CONFIG_BICOPTER_NAME_MELONCOPTER)
    float m1_force = control->motorRight_N;
    float m4_force = control->motorLeft_N;
    #elif defined(CONFIG_BICOPTER_NAME_REDCOPTER)
    float m1_force = control->motorLeft_N;
    float m4_force = control->motorRight_N;
    #endif

    // set the servo angles in degrees
    // max = 15 deg, min = -15 deg
    s_servo1_angle = fmax(-15, fmin(15, control->servoLeft_deg));
    s_servo2_angle = fmax(-15, fmin(15, control->servoRight_deg));
    
    // given the desired force, get the DSHOT value to send to the motors.
    // motorThrustUncapped->motors.m1 is in range [0, UINT16_MAX] which is sent as a DSHOT value

    // Force (N) = pwmToThrustA * Veff^2 + pwmToThrustB * Veff

    float y1 = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * m1_force)) / (2.0f * pwmToThrustA);
    float y4 = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * m4_force)) / (2.0f * pwmToThrustA);
    
    #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
    float vBatt = pmGetBatteryVoltage();
    #else
    float vBatt = 14.8f; // 4S battery nominal voltage
    #endif

    float pwm1 = y1 / vBatt;
    float pwm4 = y4 / vBatt;

    // maximum pwm value is 1.0
    // pwmAdjust is a parameter we can set to scale up or down all thrusts
    m1_pwm = fmin(pwm1 * pwmAdjust1, 1.0f);
    m4_pwm = fmin(pwm4 * pwmAdjust4, 1.0f);

    motorThrustUncapped->motors.m1 = m1_pwm * UINT16_MAX; // left motor
    motorThrustUncapped->motors.m4 = m4_pwm * UINT16_MAX; // right motor
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
        powerDistributionLegacy(control, motorThrustUncapped);
        break;
    case controlModeForceTorque:
        powerDistributionForceTorque(control, motorThrustUncapped);
        break;
    case controlModeForce:
        powerDistributionForce(control, motorThrustUncapped);
        break;
    case controlModeWrench:
        powerDistributionWrench(control, motorThrustUncapped);
        break;
    case controlModeLQR:
        powerDistributionLQR(control, motorThrustUncapped);
        break;
    default:
        // Nothing here
        break;
    }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
    const int32_t maxAllowedThrust = UINT16_MAX;
    bool isCapped = false;

    // Find highest thrust
    int32_t highestThrustFound = 0;
    for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
    {
        const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
        if (thrust > highestThrustFound)
        {
            highestThrustFound = thrust;
        }
    }

    int32_t reduction = 0;
    if (highestThrustFound > maxAllowedThrust)
    {
        reduction = highestThrustFound - maxAllowedThrust;
        isCapped = true;
    }

    for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
    {
        int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
        motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, powerDistributionGetIdleThrust());

        // TODO: delete this
        motorPwm->list[motorIndex] = motorThrustBatCompUncapped->list[motorIndex];
    }

    return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
    int32_t thrust = idleThrust;
#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
#endif
    return thrust;
}

float powerDistributionGetMaxThrust() {
    // maximum thrust per motor (70% pwm at nominal voltage in Newtons)
    return 5.0f;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_ADD(PARAM_FLOAT, pwmAdjust1, &pwmAdjust1)
PARAM_ADD(PARAM_FLOAT, pwmAdjust4, &pwmAdjust4)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
// PARAM_GROUP_START(quadSysId)

// PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
// PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
// PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
// PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
// PARAM_GROUP_STOP(quadSysId)

LOG_GROUP_START(powerDist)
LOG_ADD(LOG_FLOAT, m1_pwm, &m1_pwm)
LOG_ADD(LOG_FLOAT, m4_pwm, &m4_pwm)
LOG_GROUP_STOP(powerDist)