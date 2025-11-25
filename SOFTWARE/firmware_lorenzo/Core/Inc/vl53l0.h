#ifndef VL53L0_H
#define VL53L0_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define VL53L0X_MAX_TOFS 4

#define NUM_TOFS 3

typedef enum {
    I2C_BUS_1,
    I2C_BUS_3,
    I2C_BUS_4
} I2C_Bus_t;

/* VL53L0X struct */
typedef struct
{
    uint8_t id;                     /* TOF index (1..4) */
    I2C_Bus_t bus;                  /* I2C bus used */
    uint16_t raw_distance_mm;       /* Last raw distance read */
    float filtered_distance_mm;     /* Last filtered distance */
    float alpha;                    /* Low-pass filter coefficient (0..1) */
    int16_t offset_mm;              /* Calibration offset (mm) */
    uint8_t initialized;            /* 1 = sensor ready */
    float prev_filtered_mm;
    uint8_t edge_detected;          /* 1 = sudden drop/edge detected */
    float edge_threshold_mm;        /* e.g. 150–300mm jump */
    uint8_t edge_cooldown;          /* prevent spam */
} VL53L0X_t;

/* Public API */
I2C_Bus_t TOF_GetBus(uint8_t tof_index);
uint8_t VL53L0X_Init(uint8_t index, float alpha);
VL53L0X_t* VL53L0X_GetHandle(uint8_t index);
uint8_t VL53L0X_IsEdgeDetected(uint8_t index);
void VL53L0X_Update(uint8_t index);

#endif
