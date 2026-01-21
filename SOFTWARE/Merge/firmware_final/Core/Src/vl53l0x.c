#include <vl53l0x.h>
#include "cmsis_os.h"

/* === VL53L0X Registers === */
#define VL53L0X_ADDR                (0x29 << 1)
#define SYSRANGE_START              0x00
#define SYSTEM_INTERRUPT_CLEAR      0x0B
#define RESULT_INTERRUPT_STATUS     0x13
#define RESULT_RANGE_MM_HIGH        0x1E
#define RESULT_RANGE_MM_LOW         0x1F
#define IDENTIFICATION_MODEL_ID     0xC0
#define VL53L0X_EXPECTED_ID         0xEEAA
#define VL53L0X_TIMEOUT             20

static void TOF_Delay(uint32_t ms)
{
#ifdef CMSIS_RTOS_V1
	osDelay(ms);
#else
	HAL_Delay(ms);
#endif
}

/* ============================================================
 *                        INITIALISATION
 * ============================================================ */
HAL_StatusTypeDef VL53L0X_Init(VL53L0X_t * tof, I2C_HandleTypeDef *hi2c,uint8_t id, float alpha)
{
	tof->id=id;
	tof->hi2c = hi2c;
	tof->alpha = (alpha < 0) ? 0 : (alpha > 1 ? 1 : alpha);
	tof->filtered_distance_mm = 0;
	tof->offset_mm = 0;

	uint8_t id_buf[2];
	HAL_StatusTypeDef st;

	/* Read model ID */
	st = HAL_I2C_Mem_Read(tof->hi2c, VL53L0X_ADDR,IDENTIFICATION_MODEL_ID,I2C_MEMADD_SIZE_8BIT,id_buf, 2, 100);
	if (st != HAL_OK)
		return st;

	uint16_t idtest = (id_buf[0] << 8) | id_buf[1];
	if (idtest != VL53L0X_EXPECTED_ID){
		return HAL_ERROR;
	}


	/* Stop any running measurement */
	uint8_t stop = 0;
	HAL_I2C_Mem_Write(tof->hi2c, VL53L0X_ADDR, SYSRANGE_START,I2C_MEMADD_SIZE_8BIT, &stop, 1, 100);
	TOF_Delay(5);

	/* Start continuous mode */
	uint8_t start = 0x02;
	HAL_I2C_Mem_Write(tof->hi2c, VL53L0X_ADDR, SYSRANGE_START,I2C_MEMADD_SIZE_8BIT, &start, 1, 100);
	TOF_Delay(10);

	tof->initialized = 1;
	return HAL_OK;
}

/* ============================================================
 *                        UPDATE MEASUREMENT
 * ============================================================ */
void VL53L0X_Update(VL53L0X_t *tof, TOF_t *alltofs)
{
	if (!tof || !tof->initialized)
		return;

	uint8_t status;
	uint8_t buf[2];

	/* Wait for measurement ready */
	uint32_t start = DWT->CYCCNT;
	do {
		HAL_I2C_Mem_Read(tof->hi2c, VL53L0X_ADDR,
				RESULT_INTERRUPT_STATUS,
				I2C_MEMADD_SIZE_8BIT,
				&status, 1, VL53L0X_TIMEOUT);

		if (status & 0x07) break;
	}
	while ((DWT->CYCCNT - start) < VL53L0X_TIMEOUT*SystemCoreClock/1000);

	/* Read range */
	if (HAL_I2C_Mem_Read(tof->hi2c, VL53L0X_ADDR,
			RESULT_RANGE_MM_HIGH,
			I2C_MEMADD_SIZE_8BIT,
			buf, 2, VL53L0X_TIMEOUT) != HAL_OK)
		return;

	uint16_t raw = (buf[0] << 8) | buf[1];
	raw = (raw > tof->offset_mm) ? (raw - tof->offset_mm) : 0;

	/* Clear interrupt */
	uint8_t clr = 1;
	HAL_I2C_Mem_Write(tof->hi2c, VL53L0X_ADDR,
			SYSTEM_INTERRUPT_CLEAR,
			I2C_MEMADD_SIZE_8BIT,
			&clr, 1, 100);

	tof->raw_distance_mm = raw;
	tof->filtered_distance_mm = tof->alpha * raw + (1.0f - tof->alpha) * tof->filtered_distance_mm;

	if(!tof->tof_cooldown){
		if(tof->filtered_distance_mm-tof->prev_filtered_distance_mm > TOF_IT_DELTA_THRESHOLD && tof->filtered_distance_mm > TOF_IT_DIST_THRESHOLD){
			tof->tof_cooldown_start = DWT->CYCCNT;
			tof->tof_int = 1;
			tof->tof_cooldown = 1;
			alltofs->tofs_cooldown = 1;
			return;
		}

	}
	if(DWT->CYCCNT - tof->tof_cooldown_start >= TOF_COOLDOWN_TIME*SystemCoreClock/1000){
		tof->tof_cooldown = 0;
		alltofs->tofs_cooldown = 0;
		return;
	}
}

//HAL_StatusTypeDef VL53L0X_Calibrate(uint8_t tof_index, uint16_t known_mm, uint16_t samples)
//{
//	if (tof_index < 1 || tof_index > VL53L0X_MAX_COUNT)
//		return HAL_ERROR;
//
//	VL53L0X_t *tof = &tof_array[tof_index - 1];
//	if (!tof->initialized)
//		return HAL_ERROR;
//
//	if (samples < 3) samples = 3;
//	if (samples > 20) samples = 20;
//
//	uint32_t sum = 0;
//	for (uint16_t i = 0; i < samples; i++)
//	{
//		VL53L0X_Update(tof);
//		sum += tof->raw_distance_mm;
//		TOF_Delay(20);
//	}
//
//	tof->offset_mm = (sum / samples) - known_mm;
//	return HAL_OK;
//}



