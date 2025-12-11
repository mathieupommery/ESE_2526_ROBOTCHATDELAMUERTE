/*
 * ylidar.h
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"

#define YLIDAR_CIRC_BUF_SIZE 1024

#define LidarMaxDepth 500

// ──────────────────────────── Clusters ────────────────────────────────

#define ClusterThreshold 100

#define SIG_LIDAR_HALF       (0x0001)
#define SIG_LIDAR_FULL       (0x0002)

#define LIDAR_POINTS             359
#define MAX_CLUSTERS             3

#define CLUSTER_THRESHOLD_MM     450
#define MIN_CLUSTER_POINTS       7
#define MAX_CLUSTER_POINTS		 100
#define MIN_CLUSTER_SIZE_MM      30
#define MAX_CLUSTER_SIZE_MM      150
#define MIN_VALID_DISTANCE_MM    50
#define MAX_VALID_DISTANCE_MM    200

#define CLUSTER_BUF_SIZE	LIDAR_POINTS + 90
#define MAX_TEMP_CLUSTERS	20

// Angle handling (1 degree per unit → 360° = 360)
#define FULL_CIRCLE_DEGREES           360

// Object tracking parameters (tune these!)
#define MAX_TRACKED_OBJECTS           MAX_CLUSTERS
#define TRACK_MATCH_THRESHOLD         45.0f     // Max weighted distance to consider a match
#define ANGLE_TRACK_WEIGHT            1.2f      // Higher = angle more important
#define DISTANCE_TRACK_WEIGHT         0.08f     // Lower = less sensitive to small jumps
#define SIZE_TRACK_WEIGHT             0.8f

typedef enum {
	FSM_STATE_0,
	FSM_STATE_1,
	FSM_STATE_2,
	FSM_STATE_3,
	FSM_STATE_4
} YLIDAR_STATE;

typedef struct{
uint16_t ylidar_read_index;
uint16_t ylidar_write_index;
uint8_t ylidar_circular_buffer[YLIDAR_CIRC_BUF_SIZE];
YLIDAR_STATE ydlidarstate;
uint8_t LSN;
uint8_t CT;
uint16_t ylidar_finalbuffer[359];
float FSA_float;
float LSA_float;
uint16_t CHECKSUM;
uint16_t XOR;
}LiDARParsing_t;

typedef struct {
	uint16_t start_angle;
	uint16_t end_angle;
	uint16_t point_count;
	uint16_t avg_dist;
	float size_mm;
} TempCluster_t;

typedef struct{
	TempCluster_t temp_clusters[MAX_TEMP_CLUSTERS];
	uint16_t cluster_buf[CLUSTER_BUF_SIZE];
	uint8_t temp_cluster_cnt;
}TempClusters_t;


typedef struct {

    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *gpio_port;
    uint16_t            gpio_pin;
	LiDARParsing_t parse_struct;
	TempClusters_t cluster_struct;


} Lidar_t;

HAL_StatusTypeDef Lidar_Init(Lidar_t *lidar,UART_HandleTypeDef *huart,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef ylidar_fsm(Lidar_t *lidar);

#endif /* INC_LIDAR_H_ */
