#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include <stdint.h>
#include "i2c.h"

#ifndef TOF0_I2C
#define TOF0_I2C (&hi2c4)
#endif

#ifndef TOF1_I2C
#define TOF1_I2C (&hi2c1)
#endif

#ifndef TOF2_I2C
#define TOF2_I2C (&hi2c3)
#endif
#define TOF_COOLDOWN_TIME 1000
#define TOF_IT_DELTA_THRESHOLD 200
#define TOF_IT_DIST_THRESHOLD 300

typedef struct
{
	uint8_t id;
	I2C_HandleTypeDef *hi2c;   // pointeur direct vers le handle HAL

	uint16_t raw_distance_mm;
	float filtered_distance_mm;

	uint16_t prev_raw_distance_mm;
	float prev_filtered_distance_mm;

	float alpha;               // coefficient du low-pass filter
	int16_t offset_mm;         // calibration offset

	uint8_t tof_int;
	uint8_t tof_cooldown;
	uint32_t tof_cooldown_start;

	uint8_t initialized;
} VL53L0X_t;


typedef struct TOF{
	VL53L0X_t tof[3];
	uint8_t tofs_cooldown;
}TOF_t;



HAL_StatusTypeDef VL53L0X_Init(VL53L0X_t * tof, I2C_HandleTypeDef *hi2c,uint8_t id, float alpha);
void VL53L0X_Update(VL53L0X_t *tof, TOF_t *alltofs);
HAL_StatusTypeDef VL53L0X_Calibrate(uint8_t tof_index, uint16_t known_distance_mm, uint16_t samples);


#endif
