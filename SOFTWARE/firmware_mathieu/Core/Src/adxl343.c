#include "adxl343.h"


#ifndef ADXL343_CS_LOW
#define ADXL343_CS_LOW(dev) HAL_GPIO_WritePin((dev)->cs_port, (dev)->cs_pin, GPIO_PIN_RESET)
#endif
#ifndef ADXL343_CS_HIGH
#define ADXL343_CS_HIGH(dev) HAL_GPIO_WritePin((dev)->cs_port, (dev)->cs_pin, GPIO_PIN_SET)
#endif


// Internal helper: build command byte per SPI rules.
static inline uint8_t adxl343_cmd(uint8_t reg, bool multi, bool is_read) {
uint8_t cmd = reg & 0x3Fu;
if (multi) cmd |= ADXL343_SPI_MB;
if (is_read) cmd |= ADXL343_SPI_RW_READ;
return cmd;
}


HAL_StatusTypeDef ADXL343_Transfer(adxl343_t *dev, uint8_t reg, bool multi, bool is_read,
const uint8_t *tx, uint8_t *rx, size_t len, uint32_t timeout_ms)
{
if (!dev || !dev->hspi) return HAL_ERROR;


HAL_StatusTypeDef st = HAL_OK;
uint8_t cmd = adxl343_cmd(reg, multi || (len>1), is_read);


ADXL343_CS_LOW(dev);


// Send command/address
st = HAL_SPI_Transmit(dev->hspi, &cmd, 1, timeout_ms);
if (st != HAL_OK) { ADXL343_CS_HIGH(dev); return st; }


if (is_read) {
// Read len bytes. Use dummy 0x00 on MOSI.
if (rx == NULL) rx = dev->_rx; // internal buffer fallback
st = HAL_SPI_Receive(dev->hspi, rx, (uint16_t)len, timeout_ms);
} else {
// Write len bytes from tx
if (tx == NULL) { ADXL343_CS_HIGH(dev); return HAL_ERROR; }
st = HAL_SPI_Transmit(dev->hspi, (uint8_t*)tx, (uint16_t)len, timeout_ms);
}


ADXL343_CS_HIGH(dev);
return st;
}



HAL_StatusTypeDef ADXL343_Init(adxl343_t *dev, SPI_HandleTypeDef *hspi,
GPIO_TypeDef *cs_port, uint16_t cs_pin,
uint8_t bw_rate, uint8_t range, uint32_t timeout_ms)
{
if (!dev) return HAL_ERROR;
dev->hspi = hspi;
dev->cs_port = cs_port;
dev->cs_pin = cs_pin;


// Ensure CS high
ADXL343_CS_HIGH(dev);


// Check DEVID
uint8_t devid = 0;
HAL_StatusTypeDef st = ADXL343_ReadReg(dev, ADXL343_REG_DEVID, &devid, timeout_ms);
if (st != HAL_OK) return st;
if (devid != ADXL343_DEVID_VALUE) return HAL_ERROR;


// Configure BW_RATE (output data rate)
dev->bw_rate = bw_rate;
st = ADXL343_WriteReg(dev, ADXL343_REG_BW_RATE, bw_rate, timeout_ms);
if (st != HAL_OK) return st;


// DATA_FORMAT: full-res + range, right-justified, 4-wire SPI
dev->data_format = (uint8_t)(ADXL343_DATA_FMT_FULL_RES | (range & 0x03u));
st = ADXL343_WriteReg(dev, ADXL343_REG_DATA_FORMAT, dev->data_format, timeout_ms);
if (st != HAL_OK) return st;


// Scale factor
if (dev->data_format & ADXL343_DATA_FMT_FULL_RES) {
dev->lsb_per_g = 1.0f / 256.0f; // ~0.0039 g/LSB
} else {
// Non full-res: 10-bit fixed scale: 256 LSB/g at ±2g, 128 at ±4g, etc.
switch (range & 0x03u) {
case ADXL343_DATA_FMT_RANGE_2G: dev->lsb_per_g = 256.0f; break;
case ADXL343_DATA_FMT_RANGE_4G: dev->lsb_per_g = 128.0f; break;
case ADXL343_DATA_FMT_RANGE_8G: dev->lsb_per_g = 64.0f; break;
default: dev->lsb_per_g = 32.0f; break; // 16g
}
// convert later as g = raw / lsb_per_g
}


st = ADXL343_WriteReg(dev, ADXL343_REG_THRESH_TAP, ADXL343_TRESHOLD_VALUE, timeout_ms);
if (st != HAL_OK) return st;

st = ADXL343_WriteReg(dev, ADXL343_REG_DUR, ADXL343_REG_DUR_VALUE, timeout_ms);
if (st != HAL_OK) return st;

st = ADXL343_WriteReg(dev, ADXL343_REG_TAP_AXES, ADXL343_REG_TAP_AXES_VALUE, timeout_ms);
if (st != HAL_OK) return st;

st = ADXL343_WriteReg(dev, ADXL343_REG_INT_MAP, ADXL343_REG_INTMAP_VALUE, timeout_ms);
if (st != HAL_OK) return st;

st = ADXL343_WriteReg(dev, ADXL343_REG_INT_ENABLE, ADXL343_REG_INTEN_VALUE, timeout_ms);
if (st != HAL_OK) return st;



// Enter measurement mode
st = ADXL343_WriteReg(dev, ADXL343_REG_POWER_CTL, ADXL343_POWER_MEASURE, timeout_ms);
if (st != HAL_OK) return st;


// Optional: flush FIFO / set bypass
(void)ADXL343_WriteReg(dev, ADXL343_REG_FIFO_CTL, 0x00u, timeout_ms);


// Prime one reading
return ADXL343_ReadXYZ(dev, timeout_ms);
}





HAL_StatusTypeDef ADXL343_ReadXYZ(adxl343_t *dev, uint32_t timeout_ms)
{
uint8_t buf[6];
HAL_StatusTypeDef st = ADXL343_Transfer(dev, ADXL343_REG_DATAX0, true, true,
NULL, buf, sizeof(buf), timeout_ms);
if (st != HAL_OK) return st;


// Little-endian sign-extended 16-bit (right-justified)
dev->raw[0] = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
dev->raw[1] = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
dev->raw[2] = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);


if (dev->data_format & ADXL343_DATA_FMT_FULL_RES) {
// In full-res, 3.9 mg/LSB independent of range
dev->g[0] = dev->raw[0] * (1.0f/256.0f);
dev->g[1] = dev->raw[1] * (1.0f/256.0f);
dev->g[2] = dev->raw[2] * (1.0f/256.0f);
} else {
dev->g[0] = (float)dev->raw[0] / dev->lsb_per_g;
dev->g[1] = (float)dev->raw[1] / dev->lsb_per_g;
dev->g[2] = (float)dev->raw[2] / dev->lsb_per_g;
}
return HAL_OK;
}


HAL_StatusTypeDef ADXL343_INT_HANDLER(adxl343_t *dev, uint32_t timeout_ms){

	dev->tap_event=1;

	uint8_t registerstate=0x00;
	HAL_StatusTypeDef st = ADXL343_ReadReg(dev, ADXL343_REG_ACT_TAP_STATUS, &registerstate, timeout_ms);
	if (st != HAL_OK) return st;

	if(registerstate & 0x01){
		dev->z_tap=1;
	}
	if(registerstate & 0x02){
		dev->y_tap=1;
	}
	if(registerstate & 0x04){
		dev->x_tap=1;
	}

	uint8_t intsourcestate=0x00;
	st = ADXL343_ReadReg(dev, ADXL343_REG_INT_SOURCE, &intsourcestate, timeout_ms);
	if (st != HAL_OK) return st;

	return st;
}
