/*
 * control.c
 *
 * Created on: Nov 30, 2025
 * Author: lorenzo
 */

#include "control.h"
#include "Lidar.h"
#include "odometry.h"
#include "vl53l0x.h"

void Omnicontrol_movement(Lidar_t *lidar, RobotControl_t *control)
{
	switch(control->gamestate){
	case SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly:
		Target_t *targetedobject = &lidar->cluster_struct.targets[control->enemy];
		if (targetedobject->size == 0) {
			control->target.distance_mm = 0;
			control->target.rotation_degs = 0;
			return;
		}

		// Cone in front (300°–60° wrap-around)
		uint16_t diff0 = abs(targetedobject->angle - 0);
		if (MIN(diff0, 360 - diff0) < 60) {
			control->target.bearing = 0;
			int16_t delta = (int16_t)targetedobject->angle;
			if (delta > 180) delta -= 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetedobject->distance;
			}
			return;
		}

		// Cone at 120°
		uint16_t diff120 = abs(targetedobject->angle - 120);
		if (MIN(diff120, 360 - diff120) < 60) {
			control->target.bearing = 120;
			int16_t delta = (int16_t)targetedobject->angle - 120;
			if (delta > 180) delta -= 360;
			else if (delta < -180) delta += 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetedobject->distance;
			}
			return;
		}

		// Cone at 240°
		uint16_t diff240 = abs(targetedobject->angle - 240);
		if (MIN(diff240, 360 - diff240) < 60) {
			control->target.bearing = 240;
			int16_t delta = (int16_t)targetedobject->angle - 240;
			if (delta > 180) delta -= 360;
			else if (delta < -180) delta += 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetedobject->distance;
			}
			return;
		}

		// No valid target
		control->target.rotation_degs = 0;
	default:
		control->target.rotation_degs = 0;
		control->target.distance_mm = 0;
		control->target.bearing = 0;
	}
}

void Staticcontrol_movement(Lidar_t *lidar, RobotControl_t *control, OmniOdometry_t *odom, TOF_t *tofs){
	switch(control->gamestate){
	case SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly:
		Target_t *targetedobject = &lidar->cluster_struct.targets[control->enemy];
		if (targetedobject->size == 0) {
			control->target.distance_mm = 0;
			control->target.rotation_degs = 0;
			return;
		}

		if(tofs->tof[0].tof_int && tofs->tofs_cooldown){
			control->target.distance_mm = 0;
			control->enemy = (control->enemy + 1)%MAX_TARGETS;
			targetedobject = &lidar->cluster_struct.targets[control->enemy];
			tofs->tof[0].tof_int = 0;
		}

		// Compute delta angle to aim for enemy
		control->target.bearing = 180;
		int16_t delta = (int16_t)targetedobject->angle - 180;
		if (delta > 180) delta -= 360;
		control->target.rotation_degs = delta;
		if (delta<=5){
			control->target.distance_mm = targetedobject->distance;
		}
		control->target.rotation_degs = 0;
	case MouseGoesZoomZoomLikeSpeedyGonzalesArribaArriba:

		//		control->target.bearing = 0;
		//		control->target.rotation_degs = (targetedobject->angle - 180)%360;
		// Put this in callback of tap to initialize escape

//		Target_t *targetedobject = &lidar->cluster_struct.targets[control->enemy];
		control->target.bearing = 180;
		if(tofs->tof[0].tof_int && tofs->tofs_cooldown){
			control->target.distance_mm = 0;
			control->target.rotation_degs = fmodf(odom->position.w + targetedobject->angle, 360.0f);
			tofs->tof[0].tof_int = 0;
			break;
		}
		control->target.distance_mm = abs(targetedobject->distance - KEEP_ROBOT_AT_BAY);
	default:
		control->target.rotation_degs = 0;
		control->target.distance_mm = 0;
		control->target.bearing = 0;
	}
}
