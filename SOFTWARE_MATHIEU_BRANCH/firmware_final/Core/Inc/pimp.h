/*
 * pimp.h
 *
 *  Created on: Jan 24, 2026
 *      Author: mathi
 */

#ifndef INC_PIMP_H_
#define INC_PIMP_H_

#include "main.h"
#include "crsf.h"




HAL_StatusTypeDef crsf_proccess_switch(CRSF_t *crsf,UART_HandleTypeDef *huart,uint16_t threshold);

#endif /* INC_PIMP_H_ */
