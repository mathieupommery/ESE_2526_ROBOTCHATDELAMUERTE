/*
 * OmniwheelCinematics.h
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

// ─────────────────────────────────────────────────────────────
// ──────────────────────────── INCLUDES ────────────────────────────────
// ────────────────────────────────────────────────────────────

#include "main.h"

// ────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINITIONS ────────────────────────────────
// ────────────────────────────────────────────────────────────

#define WHEEL_DIAMETER 60
#define WHEEL_RADIUS WHEEL_DIAMETER/2
#define BEARING_DIAMETER 14
#define ROBOT_WHEEL_TF 85

#define MOTOR_MAX_RPM           300
#define WHEEL_CIRCUMFERENCE    (M_PI * WHEEL_DIAMETER)                    // ≈ 188.4 mm
#define MAX_WHEEL_SPEED_RPM     ((MOTOR_MAX_RPM * WHEEL_CIRCUMFERENCE) / 60.0f)  // ≈ 942 mm/s

#define SPEED_COEFFICIENT WHEEL_RADIUS/3

#define SPEED_CONTRIBUTIION_WEIGHT sqrt(3)/2

#define ROTATIONAL_SPEED_COEFFICIENT WHEEL_RADIUS/(3*ROBOT_WHEEL_TF)

#define WHEEL_1_DIRECTION 0
#define WHEEL_2_DIRECTION 120
#define WHEEL_3_DIRECTION 240

#ifndef DELTA_TIME_MS
#define DELTA_TIME_MS 20.0f
#endif

// ────────────────────────────────────────────────────────────
// ──────────────────────────── VARIABLES ────────────────────────────────
// ────────────────────────────────────────────────────────────

typedef struct{
	float speed_bearing_0_mms;
	float speed_bearing_120_mms;
	float speed_bearing_240_mms;
	float rotational_speed_degs;

	float prev_speed_bearing_0_mms;
	float prev_speed_bearing_120_mms;
	float prev_speed_bearing_240_mms;
	float prev_rotational_speed_degs;
}Robot_Cinematics_t;

typedef struct{
	float distance_bearing_0_mm;
	float distance_bearing_120_mm;
	float distance_bearing_240_mm;
	float delta_orientation_deg;
}Robot_Movement_t;

typedef struct{
	float x;
	float y;
	float w;
}Robot_XY_Position_t;

// Handle principal
typedef struct {
    Robot_Cinematics_t cinematics;
    Robot_Movement_t   movement;
    Robot_XY_Position_t   position;
} OmniOdometry_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── PROTOTYPES ────────────────────────────────
// ────────────────────────────────────────────────────────────

void OmniOdometry_ComputeRobotSpeed(OmniOdometry_t *odom,
                                    Robot_Target_Movement_t *target,
                                    MOTOR_COM motor_speeds);

void OmniOdometry_ComputeMovement(OmniOdometry_t *odo);

void OmniOdometry_ComputePosition(OmniOdometry_t *odo);

void OmniOdometry_ComputeInverseCinematics(OmniOdometry_t *odo,
                                 const Robot_Target_Movement_t *target,
                                 MOTOR_COM *Motor_com_out);

#endif /* INC_ODOMETRY_H_ */
