/*
 * ylidar.h
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <math.h>


#define PI_F     3.14159265358979323846f
#define HALF_PI  1.57079632679489661923f
#define RAD2DEG  57.295779513082320876f


#define CAN_DIAM_MM        66.0f
#define CAN_TOL_MM         25.0f
#define CAN_MIN_MM         (CAN_DIAM_MM - CAN_TOL_MM)
#define CAN_MAX_MM         (CAN_DIAM_MM + CAN_TOL_MM)

#define CAN_MIN_DIST_MM    150u
#define CAN_MAX_DIST_MM    3000u

#define CAN_GATE_MAX_DA_DEG    25u
#define CAN_GATE_MAX_DD_MM     800u

// smoothing (1 target)
#define CAN_SMOOTH_SHIFT_ANG  2   // 1/4
#define CAN_SMOOTH_SHIFT_DIST 2
#define CAN_SMOOTH_SHIFT_SIZE 2
#define CAN_MISS_MAX          3   // nb scans sans detection avant "perte"



#define YLIDAR_CIRC_BUF_SIZE 1024

#define LidarMaxDepth 2000


#define LIDAR_POINTS 360u
#define MAX_TARGETS  4u

/* ===== Réglages cluster ===== */
#define CL_MIN_DIST_MM         50u
#define CL_MAX_DIST_MM         2000u
#define CL_DERIV_THRESH_MM     70u   // rupture entre deux angles adjacents
#define CL_MIN_CLUSTER_POINTS  5u
#define CL_MIN_CLUSTER_SIZE_MM 60.0f
#define CL_MAX_CLUSTER_SIZE_MM 140.0f

#define CL_MAX_HOLE_RUN        3u    // trous consécutifs tolérés dans un cluster
#define CL_HOLE_REJOIN_THRESH  180u  // seuil diff distance pour "rejoindre" après trou (mm)

#define CL_WRAP_MERGE_ANGLE    10u     // zone 0° / 359° pour fusion wrap
#define CL_ENABLE_MEDIAN3      1

typedef struct {
    uint16_t start;
    uint16_t end;
    uint16_t avg_dist;
    uint16_t min_dist;
    uint16_t points;
} TempCluster_t;


typedef struct {
    uint16_t angle;
    uint16_t distance;
    uint16_t width;
    uint16_t points;
} Target_t;

typedef struct {
    Target_t targets[MAX_TARGETS];
    uint8_t  target_cnt;
} ClusterFrame_t;


typedef struct {
    ClusterFrame_t frame_a;
    ClusterFrame_t frame_b;
    ClusterFrame_t *build;
    ClusterFrame_t *ready;
    uint8_t ready_flag;
    Target_t smooth_can;
    uint8_t  can_valid;
    uint8_t  can_missed;
} ClusterizerPP_t;


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
uint8_t turn_finished_flag;
}LiDARParsing_t;


typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *gpio_port;
    uint16_t            gpio_pin;
	LiDARParsing_t parse_struct;
	ClusterizerPP_t cluster_struct;

}Lidar_t;

HAL_StatusTypeDef Lidar_Init(Lidar_t *lidar,UART_HandleTypeDef *huart,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef ylidar_fsm(Lidar_t *lidar);

#endif /* INC_LIDAR_H_ */
