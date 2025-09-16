/*
 * ylidar.h
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#ifndef INC_YLIDAR_H_
#define INC_YLIDAR_H_



#define YLIDAR_CIRC_BUF_SIZE 1024
#define YLIDAR_CIRC_BUF_MASK 1023


#define SIG_LIDAR_HALF       (0x0001)
#define SIG_LIDAR_FULL       (0x0002)


typedef enum {
    FSM_STATE_0,
    FSM_STATE_1,
    FSM_STATE_2,
    FSM_STATE_3
} YLIDAR_STATE;



#endif /* INC_YLIDAR_H_ */
