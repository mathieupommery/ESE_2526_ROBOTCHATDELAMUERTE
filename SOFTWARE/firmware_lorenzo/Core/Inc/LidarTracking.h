/*
 * ylidar.h
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#ifndef INC_LIDARTRACKING_H_
#define INC_LIDARTRACKING_H_

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdlib.h"

void ylidar_fsm(void);
void trackObject(void);

#define YLIDAR_CIRC_BUF_SIZE 1024
#define YLIDAR_CIRC_BUF_MASK 1023

#define LidarMaxDepth 500

#define ClusterThreshold 100

#define SIG_LIDAR_HALF       (0x0001)
#define SIG_LIDAR_FULL       (0x0002)

typedef enum {
    FSM_STATE_0,
    FSM_STATE_1,
    FSM_STATE_2,
    FSM_STATE_3,
	FSM_STATE_4
} YLIDAR_STATE;

#define LIDAR_POINTS             360
#define MAX_CLUSTERS              10

// ────────────────────── CONFIGURATION ──────────────────────
#define CLUSTER_THRESHOLD_MM     450
#define MAX_HOLE_COUNT           3
#define HOLE_TOLERANCE_MM        10
#define MIN_CLUSTER_POINTS       7
#define MAX_CLUSTER_POINTS		 100
#define MIN_CLUSTER_SIZE_MM       50
#define MAX_CLUSTER_SIZE_MM       150
#define MIN_VALID_DISTANCE_MM    80
#define MAX_VALID_DISTANCE_MM    1000
// ─────────────────────────────────────────────────────────────

typedef struct {
    uint16_t first_angle;
    uint16_t last_angle;
    uint16_t center_angle;
    uint16_t distance_mm;
    float    size_mm;
    uint16_t point_count;
    uint32_t timestamp_ms;
} LidarCluster_t;

typedef struct {
    LidarCluster_t clusters[MAX_CLUSTERS];
    uint8_t       count;
    uint32_t      scan_id;
    bool          new_data;
} LidarClusterList_t;

// Prototypes
//void LidarObjectTracker_Init(void);
void LidarClusterTracker_ProcessScan(const uint16_t scan[LIDAR_POINTS], LidarClusterList_t *result);

#endif /* INC_LIDARTRACKING_H_ */
