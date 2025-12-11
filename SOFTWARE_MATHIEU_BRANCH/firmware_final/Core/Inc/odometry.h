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

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdlib.h"

// ────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINITIONS ────────────────────────────────
// ────────────────────────────────────────────────────────────

#define WHEEL_DIAMETER 60
#define WHEEL_RADIUS WHEEL_DIAMETER/2
#define BEARING_DIAMETER 14
#define ROBOT_WHEEL_TF 85

#define SPEED_COEFFICIENT WHEEL_RADIUS/3
#define SPEED_CONTRIBUTIION_WEIGHT sqrt(3)/2

#define ROTATIONAL_SPEED_COEFFICIENT WHEEL_RADIUS/(3*ROBOT_WHEEL_TF)

#define WHEEL_1_DIRECTION 0
#define WHEEL_2_DIRECTION 120
#define WHEEL_3_DIRECTION 240

#define DELTA_TIME_MS 20

// ────────────────────────────────────────────────────────────
// ──────────────────────────── BEARINGS ────────────────────────────────
// ────────────────────────────────────────────────────────────

typedef enum{
	speed_bearing_0_mms,
	speed_bearing_120_mms,
	speed_bearing_240_mms
}BEARINGS;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── VARIABLES ────────────────────────────────
// ────────────────────────────────────────────────────────────

typedef struct{
	uint16_t speed_bearing_0_mms;
	uint16_t speed_bearing_120_mms;
	uint16_t speed_bearing_240_mms;
	uint16_t rotational_speed_degs;
	uint16_t prev_speed_bearing_0_mms;
	uint16_t prev_speed_bearing_120_mms;
	uint16_t prev_speed_bearing_240_mms;
	uint16_t prev_rotational_speed_degs;
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

typedef struct{
	Robot_Cinematics_t robot_speed;
	Robot_Movement_t robot_movement;
	Robot_XY_Position_t robot_position;
}Odometry_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── PROTOTYPES ────────────────────────────────
// ────────────────────────────────────────────────────────────

void odometry_Init(void);
void compute_speed(uint8_t, uint16_t, uint16_t, uint16_t);
void compute_movement(void);
void compute_xy_position(void);

#endif /* INC_ODOMETRY_H_ */
