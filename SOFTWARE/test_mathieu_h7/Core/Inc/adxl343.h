

#ifndef INC_ADXL343_H_
#define INC_ADXL343_H_

#include "stm32h7xx_hal.h" // replace with your series header (stm32h7xx_hal.h, stm32f4xx_hal.h, ...)
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>




// --- ADXL343 register map (subset)
#define ADXL343_REG_DEVID 0x00u
#define ADXL343_REG_THRESH_TAP 0x1Du
#define ADXL343_REG_OFSX 0x1Eu
#define ADXL343_REG_OFSY 0x1Fu
#define ADXL343_REG_OFSZ 0x20u
#define ADXL343_REG_DUR 0x21u
#define ADXL343_REG_LATENT 0x22u
#define ADXL343_REG_WINDOW 0x23u
#define ADXL343_REG_THRESH_ACT 0x24u
#define ADXL343_REG_THRESH_INACT 0x25u
#define ADXL343_REG_TIME_INACT 0x26u
#define ADXL343_REG_ACT_INACT_CTL 0x27u
#define ADXL343_REG_THRESH_FF 0x28u
#define ADXL343_REG_TIME_FF 0x29u
#define ADXL343_REG_TAP_AXES 0x2Au
#define ADXL343_REG_ACT_TAP_STATUS 0x2Bu
#define ADXL343_REG_BW_RATE 0x2Cu
#define ADXL343_REG_POWER_CTL 0x2Du
#define ADXL343_REG_INT_ENABLE 0x2Eu
#define ADXL343_REG_INT_MAP 0x2Fu
#define ADXL343_REG_INT_SOURCE 0x30u
#define ADXL343_REG_DATA_FORMAT 0x31u
#define ADXL343_REG_DATAX0 0x32u // through 0x37
#define ADXL343_REG_DATAX1 0x33u
#define ADXL343_REG_DATAY0 0x34u
#define ADXL343_REG_DATAY1 0x35u
#define ADXL343_REG_DATAZ0 0x36u
#define ADXL343_REG_DATAZ1 0x37u
#define ADXL343_REG_FIFO_CTL 0x38u
#define ADXL343_REG_FIFO_STATUS 0x39u


// Device ID expected value (same as ADXL345/343)
#define ADXL343_DEVID_VALUE 0xE5u


// DATA_FORMAT bits
#define ADXL343_DATA_FMT_RANGE_2G 0x00u
#define ADXL343_DATA_FMT_RANGE_4G 0x01u
#define ADXL343_DATA_FMT_RANGE_8G 0x02u
#define ADXL343_DATA_FMT_RANGE_16G 0x03u
#define ADXL343_DATA_FMT_FULL_RES 0x08u
// POWER_CTL bits
#define ADXL343_POWER_MEASURE 0x08u
// SPI protocol bits
#define ADXL343_SPI_RW_READ 0x80u
#define ADXL343_SPI_MB 0x40u
// Full-res scale factor (approx.) in g/LSB
#define ADXL343_FULLRES_LSB_PER_G (1.0f/256.0f) // ≈0.00390625 g/LSB

typedef struct {
SPI_HandleTypeDef *hspi;
GPIO_TypeDef *cs_port;
uint16_t cs_pin;

uint8_t data_format;
uint8_t bw_rate;
int16_t raw[3];
float g[3];
float lsb_per_g;
uint8_t _tx[8];
uint8_t _rx[8];
} adxl343_t;


// --- Public API
// Low-level combined transfer (send & optionally receive), with CS handling.
// If is_read==true, reads `len` bytes from `reg` into dev->_rx[1..len] and copies to `rx` if non-NULL.
// If is_read==false, writes `len` bytes from `tx` to `reg`.
HAL_StatusTypeDef ADXL343_Transfer(adxl343_t *dev, uint8_t reg, bool multi, bool is_read,
const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms);


// Initialize device: verifies DEVID, sets BW_RATE, DATA_FORMAT (full-res + range), and MEASURE mode.
// bw_rate: see datasheet (e.g., 0x0C=400 Hz). range: 0..3 for ±2/4/8/16g.
// Returns HAL_OK on success and leaves dev->g[] updated with first measurement.
HAL_StatusTypeDef ADXL343_Init(adxl343_t *dev, SPI_HandleTypeDef *hspi,
GPIO_TypeDef *cs_port, uint16_t cs_pin,
uint8_t bw_rate, uint8_t range, uint32_t timeout_ms);


// Read the 6-byte XYZ registers and update dev->raw[] and dev->g[].
HAL_StatusTypeDef ADXL343_ReadXYZ(adxl343_t *dev, uint32_t timeout_ms);


// Write one register.
static inline HAL_StatusTypeDef ADXL343_WriteReg(adxl343_t *dev, uint8_t reg, uint8_t value, uint32_t to_ms) {
return ADXL343_Transfer(dev, reg, false, false, &value, NULL, 1, to_ms);
}


// Read one register
static inline HAL_StatusTypeDef ADXL343_ReadReg(adxl343_t *dev, uint8_t reg, uint8_t *value, uint32_t to_ms) {
return ADXL343_Transfer(dev, reg, false, true, NULL, value, 1, to_ms);
}


#endif
