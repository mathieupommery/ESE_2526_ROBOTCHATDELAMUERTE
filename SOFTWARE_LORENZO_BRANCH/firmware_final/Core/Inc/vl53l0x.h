#ifndef VL53L0X_H
#define VL53L0X_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

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

HAL_StatusTypeDef VL53L0X_Init(uint8_t tof_index, float alpha);
void VL53L0X_Update(VL53L0X_t *tof);

uint16_t VL53L0X_GetRawDistance(uint8_t tof_index);
float VL53L0X_GetFilteredDistance(uint8_t tof_index);

void VL53L0X_SetOffset(uint8_t tof_index, int16_t offset_mm);
int16_t VL53L0X_GetOffset(uint8_t tof_index);

HAL_StatusTypeDef VL53L0X_Calibrate(uint8_t tof_index, uint16_t known_distance_mm, uint16_t samples);

VL53L0X_t* VL53L0X_GetHandle(uint8_t tof_index);

#endif
