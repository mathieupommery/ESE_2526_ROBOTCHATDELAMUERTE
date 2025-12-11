/*
 * control.c
 *
 * Created on: Nov 30, 2025
 * Author: lorenzo
 */

#include "control.h"

void control_movement(Lidar_t *lidar, RobotControl_t *control)
{
	switch(control->gamestate){
	case SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly:
		Target_t *targetzero = lidar->cluster_struct.targets[0];
		if (targetzero->size == 0) {
			control->target.distance_mm = 0;
			control->target.rotation_degs = 0;
			return;
		}

		// Cone in front (300°–60° wrap-around)
		uint16_t diff0 = abs(targetzero->angle - 0);
		if (MIN(diff0, 360 - diff0) < 60) {
			control->target.bearing = 0;
			int16_t delta = (int16_t)center;
			if (delta > 180) delta -= 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetzero->distance;
			}
			return;
		}

		// Cone at 120°
		uint16_t diff120 = abs(targetzero->angle - 120);
		if (MIN(diff120, 360 - diff120) < 60) {
			control->target.bearing = 120;
			int16_t delta = (int16_t)center - 120;
			if (delta > 180) delta -= 360;
			else if (delta < -180) delta += 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetzero->distance;
			}
			return;
		}

		// Cone at 240°
		uint16_t diff120 = abs(targetzero->angle - 240);
		if (MIN(diff120, 360 - diff120) < 60) {
			control->target.bearing = 240;
			int16_t delta = (int16_t)center - 120;
			if (delta > 180) delta -= 360;
			else if (delta < -180) delta += 360;
			control->target.rotation_degs = delta;
			if (delta<=5){
				control->target.distance_mm = targetzero->distance;
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


