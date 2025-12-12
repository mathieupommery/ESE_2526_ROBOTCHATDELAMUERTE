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

#define LidarMaxDepth 200

// ──────────────────────────── Clusters ────────────────────────────────

#define ClusterThreshold 100

#define SIG_LIDAR_HALF       (0x0001)
#define SIG_LIDAR_FULL       (0x0002)

#define LIDAR_POINTS             359
#define MAX_TARGETS				 5

#define CLUSTER_THRESHOLD_MM     50
#define MIN_CLUSTER_POINTS       0
#define MAX_CLUSTER_POINTS		 100
#define MIN_CLUSTER_SIZE_MM      20
#define MAX_CLUSTER_SIZE_MM      150
#define MIN_VALID_DISTANCE_MM    50		// preferably greater than or equal to cluster threshold
#define MAX_VALID_DISTANCE_MM    200

#define WRAP_SIZE			90
#define CLUSTER_BUF_SIZE	LIDAR_POINTS + WRAP_SIZE
#define MAX_TEMP_CLUSTERS	3
#define MAX_HOLE_COUNT		10
#define MAX_HOLE_SIZE		5

typedef enum {
	FSM_STATE_0,
	FSM_STATE_1,
	FSM_STATE_2,
	FSM_STATE_3,
	FSM_STATE_4
} YLIDAR_STATE;

typedef enum{
	SEARCH,
	IN_CLUSTER,
	END_CLUSTER
}CLUSTERIZATION_STATE;

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
	uint16_t cluster[MAX_CLUSTER_POINTS];
	uint8_t hole_cnt;
	uint8_t hole_points;
	float avg_dist;
	float size_mm;
} TempCluster_t;

typedef struct{
	uint16_t angle;
	float distance;
	float size;
}Target_t;

typedef struct{
	Target_t targets[MAX_TARGETS];
	uint8_t target_cnt;
	TempCluster_t temp_clusters[MAX_TEMP_CLUSTERS];
	uint16_t cluster_buf[CLUSTER_BUF_SIZE];
	uint8_t temp_cluster_cnt;
	CLUSTERIZATION_STATE clusterizationstate;
}ClusterParsing_t;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *gpio_port;
    uint16_t            gpio_pin;
	LiDARParsing_t parse_struct;
	ClusterParsing_t cluster_struct;
} Lidar_t;

HAL_StatusTypeDef Lidar_Init(Lidar_t *lidar,UART_HandleTypeDef *huart,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef ylidar_fsm(Lidar_t *lidar);
HAL_StatusTypeDef Clusterize(Lidar_t *lidar);


#endif /* INC_LIDAR_H_ */
