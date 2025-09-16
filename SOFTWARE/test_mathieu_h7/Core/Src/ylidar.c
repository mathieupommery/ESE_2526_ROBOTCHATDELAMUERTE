/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */


#include "ylidar.h"
#include "main.h"




uint16_t ylidar_read_index=0;
uint16_t ylidar_write_index=0;

uint8_t ylidar_circular_buffer[YLIDAR_CIRC_BUF_SIZE];

YLIDAR_STATE ydlidarstate=FSM_STATE_0;

uint8_t current_point_number=0;

float FSA=0.0;
float LSA=0.0;



void ylidar_fsm(void)
{
    switch (ydlidarstate)
    {
        case FSM_STATE_0:

        	if((ylidar_circular_buffer[ylidar_read_index]==0xAA)&& (ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]==0x55)){
        		ydlidarstate=FSM_STATE_1;
        		ylidar_read_index=(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;
        	}
        	else{
        		ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
        	}

            break;

        case FSM_STATE_1:

        	if(!(ylidar_circular_buffer[ylidar_read_index]&0x01)){
        		ydlidarstate=FSM_STATE_2;
        		ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
        	}
        	else{
        		ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
        		ydlidarstate=FSM_STATE_0;
        	}


            break;

        case FSM_STATE_2:

        	current_point_number=ylidar_circular_buffer[ylidar_read_index];
        	uint16_t available=0;

        	if (ylidar_write_index >= ylidar_read_index) {
        	    available = ylidar_write_index - ylidar_read_index;
        	} else {
        	    available = (YLIDAR_CIRC_BUF_SIZE - ylidar_read_index) + ylidar_write_index;
        	}

        	uint16_t needed=current_point_number*2+7;

        	if(needed<=available){
        		ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
        		ydlidarstate=FSM_STATE_0;


        	}
        	else{
        		ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
        		ydlidarstate=FSM_STATE_3;
        	}


            break;

        case FSM_STATE_3:

        	FSA=(float)(((ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[ylidar_read_index])>>1)/64.0;
        	LSA=(float)(((ylidar_circular_buffer[(ylidar_read_index+3)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE])>>1)/64.0;

        	ylidar_read_index=(ylidar_read_index+(current_point_number*2))%YLIDAR_CIRC_BUF_SIZE;
        	ydlidarstate=FSM_STATE_0;


            break;

        default:
        	ydlidarstate=FSM_STATE_0;
            break;
    }

}
