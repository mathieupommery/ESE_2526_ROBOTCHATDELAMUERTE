/*
 * OmniwheelCinematics.c
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#include "odometry.h"

/* -------------------------------------------------------------------------- */
/*                     1. INITIALISATION                                      */
/* -------------------------------------------------------------------------- */

void OmniOdometry_Init(OmniOdometry_t *odo)
{
	if (!odo) return;

	memset(odo, 0, sizeof(OmniOdometry_t));
}

/* -------------------------------------------------------------------------- */
/*      2. DIRECT CINEMATICS – wheel speeds → robot speed (mm/s & °/s) */
/* -------------------------------------------------------------------------- */

void OmniOdometry_ComputeRobotSpeed(OmniOdometry_t *odom,
		RobotTargetMovement_t *target,	//extract bearing
		MOTOR_COM motor_speeds)				//extract wheel speeds
{
	if (odom == NULL) return;

	// Beware! speeds must be converted from rpm to deg/s
	switch (target->bearing) {
	case WHEEL_1_DIRECTION:
		odom->cinematics.prev_speed_bearing_0_mms = odom->cinematics.speed_bearing_0_mms;
		odom->cinematics.speed_bearing_0_mms = SPEED_COEFFICIENT *
				(motor_speeds.w1 * (2 * M_PI / 60.0f) - motor_speeds.w2 *(2 * M_PI / 60.0f));
		break;
	case WHEEL_2_DIRECTION:
		odom->cinematics.prev_speed_bearing_120_mms = odom->cinematics.speed_bearing_120_mms;
		odom->cinematics.speed_bearing_120_mms =  SPEED_COEFFICIENT *
				(motor_speeds.w2 * (2 * M_PI / 60.0f) - motor_speeds.w0 * (2 * M_PI / 60.0f));
		break;
	case WHEEL_3_DIRECTION:
		odom->cinematics.prev_speed_bearing_240_mms = odom->cinematics.speed_bearing_240_mms;
		odom->cinematics.speed_bearing_240_mms =  SPEED_COEFFICIENT *
				(motor_speeds.w0 * (2 * M_PI / 60.0f) - motor_speeds.w1 * (2 * M_PI / 60.0f));
		break;
	default:
		return; // invalid bearing
	}

	// Rotation is the same for all three calls → just overwrite
	odom->cinematics.rotational_speed_degs = ROTATIONAL_SPEED_COEFFICIENT *
			(motor_speeds.w0 * (2 * M_PI / 60.0f) + motor_speeds.w1 * (2 * M_PI / 60.0f) + motor_speeds.w2 * (2 * M_PI / 60.0f));;
}

/* -------------------------------------------------------------------------- */
/*                        3. Movement over Δt                                 */
/* -------------------------------------------------------------------------- */

void OmniOdometry_ComputeMovement(OmniOdometry_t *odo)
{
	if (odo == NULL) return;

	float dt_s = DELTA_TIME_MS / 1000.0f;

	odo->movement.delta_orientation_deg = odo->cinematics.rotational_speed_degs * dt_s;

	odo->movement.distance_bearing_0_mm   = odo->cinematics.speed_bearing_0_mms   * dt_s;
	odo->movement.distance_bearing_120_mm = odo->cinematics.speed_bearing_120_mms * dt_s;
	odo->movement.distance_bearing_240_mm = odo->cinematics.speed_bearing_240_mms * dt_s;
}

/* -------------------------------------------------------------------------- */
/*                     4. ODOMETRY – update X, Y, θ                           */
/* -------------------------------------------------------------------------- */

void OmniOdometry_ComputePosition(OmniOdometry_t *odo)
{
	if (!odo) return;

	odo->position.w += odo->movement.delta_orientation_deg;

	float w_rad = odo->position.w * (M_PI / 180.0f);  // DEG_TO_RAD not defined → done manually

	float d0   = odo->movement.distance_bearing_0_mm;
	float d120 = odo->movement.distance_bearing_120_mm;
	float d240 = odo->movement.distance_bearing_240_mm;

	odo->position.x +=
			d0   * cosf(w_rad + 0.0f * M_PI/180.0f)
	+ d120 * cosf(w_rad + 120.0f * M_PI/180.0f)
	+ d240 * cosf(w_rad + 240.0f * M_PI/180.0f);

	odo->position.y +=
			d0   * sinf(w_rad + 0.0f * M_PI/180.0f)
	+ d120 * sinf(w_rad + 120.0f * M_PI/180.0f)
	+ d240 * sinf(w_rad + 240.0f * M_PI/180.0f);
}

/* -------------------------------------------------------------------------- */
/*      5. INVERSE CINEMATICS – target movement → wheel speeds (clamped)    */
/* -------------------------------------------------------------------------- */

void OmniOdometry_ComputeInverseCinematics(OmniOdometry_t *odom, RobotTargetMovement_t *target, MOTOR_COM *Motor_com_out)
{
	if (!odom || !target || !Motor_com_out) {
		Motor_com_out->w0 = Motor_com_out->w1 = Motor_com_out->w2 = 0;
		return;
	}

	if (target->distance_mm == 0.0f && target->rotation_degs == 0.0f) {
		Motor_com_out->w0 = Motor_com_out->w1 = Motor_com_out->w2 = 0;
		return;
	}

	// Temp variables for the three wheel speeds
	float wB = 0.0f;   // "front" wheel (bearing)
	float wRR = 0.0f;  // rear-right
	float wRL = 0.0f;  // rear-left

	// Compute rotation contribution (same for all wheels)
	float rot_contrib = (target->rotation_degs / ROTATIONAL_SPEED_COEFFICIENT)/3*6;
	// because ω = ROTATIONAL_SPEED_COEFFICIENT × (w0 + w1 + w2)
	// → each wheel gets +rot_contrib from rotation

	// Compute differential drive contribution along the selected axis
	// v_axis = SPEED_COEFFICIENT × (wRR - wRL)
	// → wRR - wRL = v_axis / SPEED_COEFFICIENT
	float diff = target->distance_mm / (SPEED_COEFFICIENT * DELTA_TIME_MS * 6);
	wRR = +diff / 2;
	wRL = -diff / 2;
	// wB gets 0 from translation (it's the axis wheel → no differential needed)
	// else: pure rotation → wRR = wRL = 0 from translation

	// Add rotation to all wheels
	wB  += rot_contrib;
	wRR += rot_contrib;
	wRL += rot_contrib;

	float max_abs = fmaxf(fabsf(wB), fmaxf(fabsf(wRR), fabsf(wRL)));
	//if wheel speed exceeds 300rpm we scale all speeds down
	if (max_abs > MOTOR_MAX_RPM) {
		float scale = MOTOR_MAX_RPM / max_abs;
		wB *= scale;
		wRR *= scale;
		wRL *= scale;
	}

	switch (target->bearing){
	case WHEEL_1_DIRECTION:
		Motor_com_out->w0 = wB;
		Motor_com_out->w1 = wRR;
		Motor_com_out->w2 = wRL;
		break;
	case WHEEL_2_DIRECTION:
		Motor_com_out->w0 = wRL;
		Motor_com_out->w1 = wB;
		Motor_com_out->w2 = wRR;
		break;
	case WHEEL_3_DIRECTION:
		Motor_com_out->w0 = wRR;
		Motor_com_out->w1 = wRL;
		Motor_com_out->w2 = wB;
		break;
	}
}
