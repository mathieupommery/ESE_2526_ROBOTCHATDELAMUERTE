/*
 * pimp.c
 *
 *  Created on: Jan 24, 2026
 *      Author: mathi
 */
#include "pimp.h"

uint8_t audio_cmd_buf[128];

HAL_StatusTypeDef crsf_proccess_switch(CRSF_t *crsf,UART_HandleTypeDef *huart,uint16_t threshold){

	HAL_StatusTypeDef status = HAL_ERROR;
	uint16_t value = 0;
	uint8_t index = 0;
	uint16_t size = 0;

	//channel 5
	index=4;
	value = crsf->crsf_command[index];
    if (crsf->channel_armed[index] && value > threshold)
    {
    	//do something here
    	size = snprintf((uint8_t *)audio_cmd_buf,128,(uint8_t *)"p /bomboclat.wav\n\r");
        status = HAL_UART_Transmit(huart, (uint8_t *)audio_cmd_buf, size, 100);
        crsf->channel_armed[index] = 0;
    }
    else if (!crsf->channel_armed[index] && value < threshold)
    {
    	crsf->channel_armed[index] = 1;
    }
	//channel 6
	index=5;
	value = crsf->crsf_command[index];
    if (crsf->channel_armed[index] && value > threshold)
    {
    	//do something here
    	size = snprintf((uint8_t *)audio_cmd_buf,128,(uint8_t *)"p /android_notification.wav\n\r");
    	status = HAL_UART_Transmit(huart, (uint8_t *)audio_cmd_buf, size, 100);
        crsf->channel_armed[index] = 0;
    }
    else if (!crsf->channel_armed[index] && value < threshold)
    {
    	crsf->channel_armed[index] = 1;
    }
	//channel 7
	index=6;
	value = crsf->crsf_command[index];
    if (crsf->channel_armed[index] && value > threshold)
    {
    	//do something here
    	size = snprintf((uint8_t *)audio_cmd_buf,128,(uint8_t *)"p /romantic.wav\n\r");
    	status = HAL_UART_Transmit(huart, (uint8_t *)audio_cmd_buf, size, 100);
        crsf->channel_armed[index] = 0;
    }
    else if (!crsf->channel_armed[index] && value < threshold)
    {
    	crsf->channel_armed[index] = 1;
    }




    return status;


}
