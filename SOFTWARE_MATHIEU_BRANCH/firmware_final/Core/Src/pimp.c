/*
 * pimp.c
 *
 *  Created on: Jan 24, 2026
 *      Author: mathi
 */
#include "pimp.h"

uint8_t armed = 1;

HAL_StatusTypeDef audio_proccess_song(CRSF_t *crsf,UART_HandleTypeDef *huart,uint16_t threshold,const char *cmd,uint8_t channel){

	HAL_StatusTypeDef status = HAL_ERROR;
	uint16_t value = 0;
	if(channel > 0 && channel < 32){
	value = crsf->crsf_command[channel];
	}
	else{
		return status;
	}


    if (armed && value > threshold)
    {
        status = HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 100);
        armed = 0;
    }
    else if (!armed && value < threshold)
    {
        armed = 1;
    }

    return status;


}
