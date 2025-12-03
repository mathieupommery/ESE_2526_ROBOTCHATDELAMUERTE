/*
 * ylidar.h
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#ifndef INC_LIDARTRACKING_H_
#define INC_LIDARTRACKING_H_

// ─────────────────────────────────────────────────────────────
// ──────────────────────────── INCLUDES ────────────────────────────────
// ────────────────────────────────────────────────────────────
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"

// ─────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINES ────────────────────────────────
// ────────────────────────────────────────────────────────────

// ──────────────────────────── LiDAR Parsing ────────────────────────────────

#define YLIDAR_CIRC_BUF_SIZE 1024
#define YLIDAR_CIRC_BUF_MASK 1023

#define LidarMaxDepth 500

// ──────────────────────────── Clusters ────────────────────────────────

#define ClusterThreshold 100

#define SIG_LIDAR_HALF       (0x0001)
#define SIG_LIDAR_FULL       (0x0002)

#define LIDAR_POINTS             359
#define MAX_CLUSTERS             3

#define CLUSTER_THRESHOLD_MM     100
#define MAX_HOLE_COUNT			 10
#define MAX_HOLE_POINTS			 10
#define MIN_CLUSTER_POINTS       7
#define MAX_CLUSTER_POINTS		 100
#define MIN_CLUSTER_SIZE_MM      10
#define MAX_CLUSTER_SIZE_MM      200
#define MIN_VALID_DISTANCE_MM    50
#define MAX_VALID_DISTANCE_MM    300

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

// ────────────────────────────────────────────────────────────
// ──────────────────────────── ETATS ────────────────────────────────
// ────────────────────────────────────────────────────────────

typedef enum {
	FSM_STATE_0,
	FSM_STATE_1,
	FSM_STATE_2,
	FSM_STATE_3,
	FSM_STATE_4
} YLIDAR_STATE;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── STRUCTURES DEBUG ────────────────────────────────
// ────────────────────────────────────────────────────────────

// ──────────────────────────── Parsing LiDAR ────────────────────────────────

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
uint16_t FSA;
uint16_t LSA;
uint16_t CHECKSUM;
uint16_t XOR;
uint16_t ANGLEINIT;
uint32_t CPU_cycles;
}LiDARParsing_t;

// ──────────────────────────── Cluster Temporaires ────────────────────────────────

typedef struct {
	uint16_t start_angle;
	uint16_t end_angle;
	uint16_t point_count;
	uint16_t avg_dist;
	float size_mm;
	uint16_t first_point;
	uint16_t last_point;
} TempCluster_t;

typedef struct{
	TempCluster_t temp_clusters[MAX_TEMP_CLUSTERS];
	uint16_t cluster_buf[CLUSTER_BUF_SIZE];
	uint8_t temp_cluster_cnt;
}TempClusters_t;

// ──────────────────────────── Clusters à organiser ────────────────────────────────

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
	uint16_t center_angle;
	uint16_t distance_mm;
	float    size_mm;
	uint32_t timestamp_ms;
	uint8_t object_id;
} Object_t;

typedef struct {
	LidarCluster_t clusters[MAX_CLUSTERS];
	uint8_t       count;
	uint32_t      scan_id;
	bool          new_data;
	uint32_t CPU_cycles;
} LidarClusterList_t;

// ─────────────────────────────────────────────────────────────
// ──────────────────────────── PROTORYPES ────────────────────────────────
// ────────────────────────────────────────────────────────────

void LidarTracking_Init(void);
void ylidar_fsm(void);
void trackObject(void);
void LidarClusterTracker_ProcessScan(void);
void cluster_object_identify(void);

#endif /* INC_LIDARTRACKING_H_ */
