/*
 * OmniwheelCinematics.c
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#include <odometry.h>

Robot_Cinematics_t cinematics;
Robot_Movement_t movement;
Robot_XY_Position_t position;
Odometry_t odometry;

void odometry_Init(void){

// Initialize Speeds //
	cinematics.speed_bearing_0_mms = 0;
	cinematics.speed_bearing_120_mms = 0;
	cinematics.speed_bearing_240_mms = 0;
	cinematics.rotational_speed_degs = 0;
	cinematics.prev_speed_bearing_0_mms = 0;
	cinematics.prev_speed_bearing_120_mms = 0;
	cinematics.prev_speed_bearing_240_mms = 0;
	cinematics.prev_rotational_speed_degs = 0;

// Initialize Movements //
	movement.distance_bearing_0_mm = 0;
	movement.distance_bearing_120_mm = 0;
	movement.distance_bearing_240_mm = 0;
	movement.delta_orientation_deg = 0;

// Initialize XY Position //
	position.x = 0;
	position.y = 0;
	position.w = 0;
}

void compute_speed(uint8_t bearing, uint16_t speed_bearing, uint16_t speed_RR, uint16_t speed_RL){
	uint16_t bearing_speed = SPEED_COEFFICIENT*(speed_RR*SPEED_COEFFICIENT-speed_RL*SPEED_COEFFICIENT);
	uint16_t rotational_speed = ROTATIONAL_SPEED_COEFFICIENT*(speed_bearing+speed_RR+speed_RL);
	switch (bearing){
	case 0:
		cinematics.prev_speed_bearing_0_mms = cinematics.speed_bearing_0_mms;
		cinematics.speed_bearing_0_mms = bearing_speed;
	case 1:
		cinematics.speed_bearing_120_mms = cinematics.prev_speed_bearing_120_mms;
		cinematics.speed_bearing_120_mms = bearing_speed;
	case 2:
		cinematics.speed_bearing_240_mms = cinematics.prev_speed_bearing_240_mms;
		cinematics.speed_bearing_240_mms = bearing_speed;
	}
	cinematics.rotational_speed_degs = rotational_speed;
}

void compute_movement(void){
	movement.delta_orientation_deg = 1000*(cinematics.rotational_speed_degs - cinematics.prev_rotational_speed_degs)/DELTA_TIME_MS;
	movement.distance_bearing_0_mm = 1000*(cinematics.speed_bearing_0_mms - cinematics.prev_speed_bearing_0_mms)/DELTA_TIME_MS;
	movement.distance_bearing_120_mm = 1000*(cinematics.speed_bearing_120_mms - cinematics.prev_speed_bearing_120_mms)/DELTA_TIME_MS;
	movement.distance_bearing_240_mm = 1000*(cinematics.speed_bearing_240_mms - cinematics.prev_speed_bearing_240_mms)/DELTA_TIME_MS;
}

void compute_xy_position(void){
	position.w = position.w + movement.delta_orientation_deg;
	position.x = position.x
			+ sinf(position.w) * movement.distance_bearing_0_mm
			+ sinf(position.w + 120) * movement.distance_bearing_120_mm
			+ sinf(position.w + 240) * movement.distance_bearing_240_mm;
	position.y = position.y
			+ sinf(position.w) * movement.distance_bearing_0_mm
			+ sinf(position.w + 120) * movement.distance_bearing_120_mm
			+ sinf(position.w + 240) * movement.distance_bearing_240_mm;
}
