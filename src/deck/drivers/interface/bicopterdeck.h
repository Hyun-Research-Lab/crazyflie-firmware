/**
 *
 * servo.h - servo header file
 *
 * The motors PWM ratio is a value on 16 bits (from 0 to 0xFFFF)
 * the functions of the driver will make the conversions to the actual PWM
 * precision (ie. if the precision is 8bits 0xFFFF and 0xFF00 are equivalents).
 *
 * The precision is set in number of bits by the define MOTORS_PWM_BITS
 * The timer prescaler is set by MOTORS_PWM_PRESCALE
 */
#ifndef __BICOPTERDECK_H__
#define __BICOPTERDECK_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "deck_core.h"

/******** Defines ********/
#define SERVO_PWM_PERIOD         8000  // ARR register content
#define SERVO_PWM_FREQUENCY_HZ   50 // target servo pwm frequency
#define SERVO_PWM_PRESCALE       (uint16_t) (210) // 84mhz / (50hz * ARR)

extern double s_servo1_angle;
extern double s_servo2_angle;
extern float measuredServoLeftDeg;
extern float measuredServoRightDeg;

/**
 * Servo Initialization
 */
void servoInit();

bool servoTest(void);

/**
 *
 * @brief Set servo angle.
 * @param: angle: desired servo angle in degrees
 */
void servo1SetAngle(double angle);

/**
 *
 * @brief Set servo angle.
 * @param: angle: desired servo angle in degrees
 */
void servo2SetAngle(double angle);


void bicopterDeckTask(void* arg);

#endif /* __BICOPTERDECK_H__ */
