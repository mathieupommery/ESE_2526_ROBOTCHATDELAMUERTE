#include "vl53l0.h"
#include <string.h>
#include <stdio.h>

/* VL53L0X default I2C address */
#define VL53L0X_DEFAULT_ADDR 0x52

/* VL53L0X registers */
#define SYSRANGE_START          0x00
#define RESULT_INTERRUPT_STATUS 0x13
#define RESULT_RANGE_VALUE      0x1E
#define SYSTEM_INTERRUPT_CLEAR  0x0B

/* Max TOFs managed */
static VL53L0X_t tofs[VL53L0X_MAX_TOFS];

/* External I2C handles (defined in main.c) */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

/* Map TOF index to I2C bus */
I2C_Bus_t TOF_GetBus(uint8_t tof_index)
{
    switch (tof_index)
    {
        case 1: return I2C_BUS_4;
        case 2: return I2C_BUS_1;
        case 3: return I2C_BUS_3;
        default: return I2C_BUS_1;
    }
}

/* Get HAL I2C handle from bus */
static I2C_HandleTypeDef* VL53L0X_GetI2CHandle(I2C_Bus_t bus)
{
    switch(bus)
    {
        case I2C_BUS_1: return &hi2c1;
        case I2C_BUS_3: return &hi2c3;
        case I2C_BUS_4: return &hi2c4;
        default: return &hi2c1;
    }
}

/* Low-level I2C register operations */
static HAL_StatusTypeDef VL53L0X_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, VL53L0X_DEFAULT_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef VL53L0X_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, VL53L0X_DEFAULT_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

static HAL_StatusTypeDef VL53L0X_ReadReg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, VL53L0X_DEFAULT_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) != HAL_OK)
        return HAL_ERROR;

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

/* Initialize a TOF by index (1..VL53L0X_MAX_TOFS) */
uint8_t VL53L0X_Init(uint8_t index, float alpha)
{
    if (index < 1 || index > VL53L0X_MAX_TOFS) return 0;

    VL53L0X_t *tof = &tofs[index - 1];
    I2C_HandleTypeDef *hi2c = VL53L0X_GetI2CHandle(TOF_GetBus(index));

    tof->id = index;
    tof->bus = TOF_GetBus(index);
    tof->alpha = alpha;
    tof->offset_mm = 0;
    tof->edge_threshold_mm = 200;
    tof->raw_distance_mm = 0;
    tof->filtered_distance_mm = 0;
    tof->prev_filtered_mm = 0;
    tof->edge_detected = 0;
    tof->edge_cooldown = 0;

    // Minimal init sequence
    if (VL53L0X_WriteReg(hi2c, 0x88, 0x00) != HAL_OK) return 0;
    if (VL53L0X_WriteReg(hi2c, 0x80, 0x01) != HAL_OK) return 0;
    if (VL53L0X_WriteReg(hi2c, 0xFF, 0x01) != HAL_OK) return 0;
    if (VL53L0X_WriteReg(hi2c, 0x00, 0x00) != HAL_OK) return 0;

    tof->initialized = 1;
    return 1;
}

/* Get pointer to TOF struct */
VL53L0X_t* VL53L0X_GetHandle(uint8_t index)
{
    if (index < 1 || index > VL53L0X_MAX_TOFS) return NULL;
    return &tofs[index - 1];
}

/* Check if edge detected */
uint8_t VL53L0X_IsEdgeDetected(uint8_t index)
{
    VL53L0X_t *tof = VL53L0X_GetHandle(index);
    if (!tof) return 0;
    return tof->edge_detected;
}

/* Update TOF: read distance, filter, detect edges */
void VL53L0X_Update(uint8_t index)
{
    VL53L0X_t *tof = VL53L0X_GetHandle(index);
    if (!tof || !tof->initialized) return;

    I2C_HandleTypeDef *hi2c = VL53L0X_GetI2CHandle(tof->bus);

    // Start measurement
    VL53L0X_WriteReg(hi2c, SYSRANGE_START, 0x01);

    // Wait for data ready
    uint8_t status = 0;
    uint32_t timeout = 1000;
    do {
        VL53L0X_ReadReg(hi2c, RESULT_INTERRUPT_STATUS, &status);
        timeout--;
    } while ((status & 0x07) == 0 && timeout > 0);

    if (timeout == 0) return;

    // Read raw distance
    uint16_t distance = 0;
    if (VL53L0X_ReadReg16(hi2c, RESULT_RANGE_VALUE, &distance) != HAL_OK) return;

    // Clear interrupt
    VL53L0X_WriteReg(hi2c, SYSTEM_INTERRUPT_CLEAR, 0x01);

    tof->raw_distance_mm = distance + tof->offset_mm;

    // Apply low-pass filter
    tof->filtered_distance_mm = tof->alpha * tof->raw_distance_mm +
                                (1.0f - tof->alpha) * tof->prev_filtered_mm;

    // Edge detection
    if (tof->edge_cooldown == 0 &&
        (tof->prev_filtered_mm - tof->filtered_distance_mm) > tof->edge_threshold_mm)
    {
        tof->edge_detected = 1;
        tof->edge_cooldown = 5;
    }
    else
    {
        tof->edge_detected = 0;
        if (tof->edge_cooldown > 0) tof->edge_cooldown--;
    }

    tof->prev_filtered_mm = tof->filtered_distance_mm;
}
