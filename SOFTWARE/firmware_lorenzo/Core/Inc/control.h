/*
 * control.h
 *
 *  Created on: Dec 3, 2025
 *      Author: lorenzo
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#include "cat.h"
#include "Odometry.h"
#include <math.h>

// ────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINES FOR ROTATION CONTROL ────────────────────────────────
// ────────────────────────────────────────────────────────────

// PID gains for orientation control (tune these!)
#define MAX_MOTOR_RPM            300.0f          // ← YOUR HARD LIMIT

#define PID_KP_ROT               4.0f
#define PID_KI_ROT               0.05f
#define PID_KD_ROT               10.0f

#define MAX_ROT_SPEED_DEG_S      500.0f          // allows full 300 RPM spin-on-spot
#define INTEGRAL_LIMIT           (MAX_MOTOR_RPM / PID_KI_ROT)

#define WHEEL_DIAMETER_MM        60.0f
#define WHEEL_CIRCUMFERENCE_MM   (M_PI * WHEEL_DIAMETER_MM)   // ≈ 188.4 mm
#define ROBOT_RADIUS_MM          85.0f
#define DT_S                     ((float)DELTA_TIME_MS / 1000.0f)   // 0.02 s

// ────────────────────────────────────────────────────────────
// ──────────────────────────── STRUCTS ────────────────────────────────
// ────────────────────────────────────────────────────────────

typedef struct{
float w1;
float w2;
float w3;
}MotorSpeeds_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── PROTOTYPES ────────────────────────────────
// ────────────────────────────────────────────────────────────

float control_rotation_speed(void);
#endif /* INC_CONTROL_H_ */
