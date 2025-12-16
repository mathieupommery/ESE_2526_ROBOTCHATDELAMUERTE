#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include <stdint.h>
#include "i2c.h"


#define TOF1_I2C (&hi2c4)
#define TOF2_I2C (&hi2c1)
#define TOF3_I2C (&hi2c3)

typedef struct
{
	uint8_t id;
	I2C_HandleTypeDef *hi2c;   // pointeur direct vers le handle HAL

	uint16_t raw_distance_mm;
	float filtered_distance_mm;

	float alpha;               // coefficient du low-pass filter
	int16_t offset_mm;         // calibration offset

	uint8_t initialized;
} VL53L0X_t;

HAL_StatusTypeDef VL53L0X_Init(VL53L0X_t * tof, I2C_HandleTypeDef *hi2c,uint8_t id, float alpha);
void VL53L0X_Update(VL53L0X_t *tof);
HAL_StatusTypeDef VL53L0X_Calibrate(uint8_t tof_index, uint16_t known_distance_mm, uint16_t samples);


#endif
