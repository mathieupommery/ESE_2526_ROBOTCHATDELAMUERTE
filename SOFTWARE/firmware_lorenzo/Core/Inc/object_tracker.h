#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define LIDAR_POINTS             360
#define MAX_OBJECTS              10

// ────────────────────── CONFIGURATION ──────────────────────
#define CLUSTER_THRESHOLD_MM     450
#define MAX_HOLE_COUNT           3
#define HOLE_TOLERANCE_MM        10
#define MIN_CLUSTER_POINTS       7
#define MAX_CLUSTER_POINTS		 100
#define MIN_OBJECT_SIZE_MM       50
#define MAX_OBJECT_SIZE_MM       150
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
} LidarObject_t;

typedef struct {
    LidarObject_t objects[MAX_OBJECTS];
    uint8_t       count;
    uint32_t      scan_id;
    bool          new_data;
} LidarObjectList_t;

// Prototypes
//void LidarObjectTracker_Init(void);
void LidarObjectTracker_ProcessScan(const uint16_t scan[LIDAR_POINTS], LidarObjectList_t *result);

#endif
