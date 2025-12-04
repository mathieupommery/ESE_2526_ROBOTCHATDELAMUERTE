/*
 * control.h
 *
 *  Created on: Nov 30, 2025
 *      Author: lorenzo
 */

#ifndef INC_GAMEMODE_H_
#define INC_GAMEMODE_H_

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <Lidar.h>
#include <stdlib.h>

typedef struct{
	uint16_t target_bearing;
	int delta_orientation;
	float target_speed;
}Cat_t;

void attack(void);

#endif /* INC_GAMEMODE_H_ */
