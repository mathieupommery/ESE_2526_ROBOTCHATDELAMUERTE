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



HAL_StatusTypeDef audio_proccess_song(CRSF_t *crsf,UART_HandleTypeDef *huart,uint16_t threshold,const char *cmd,uint8_t channel);

#endif /* INC_PIMP_H_ */
