/*
 * Control.c
 *
 *  Created on: Nov 30, 2025
 *      Author: lorenzo
 */

#include "cat.h"

extern LidarClusterList_t lidar;
Cat_t cat;

void attack(void){
	uint16_t diff0 = abs(lidar.clusters[0].center_angle - 0);
	if (MIN(diff0, 360 - diff0) < 60) {
		cat.target_bearing = 0;

		if (lidar.clusters[0].center_angle > 300){
			cat.delta_orientation = lidar.clusters[0].center_angle - 360;
		}
		else{
			cat.delta_orientation = lidar.clusters[0].center_angle;
		}
	}

	if (abs(lidar.clusters[0].center_angle - 120) < 60) {
		cat.target_bearing = 120;
		cat.delta_orientation = lidar.clusters[0].center_angle -120;
	}

	if (abs(lidar.clusters[0].center_angle - 240) < 60) {
		cat.target_bearing = 240;
		cat.delta_orientation = lidar.clusters[0].center_angle - 240;
	}
}

