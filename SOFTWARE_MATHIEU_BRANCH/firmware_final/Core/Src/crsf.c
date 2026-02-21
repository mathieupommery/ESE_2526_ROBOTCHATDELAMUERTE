/*
 * crsf.c
 *
 *  Created on: Jan 21, 2026
 *      Author: mathi
 */

#include "crsf.h"


static uint8_t crsf_crc8(uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

static void crsf_decode_channels(uint8_t *payload, uint16_t *channels)
{
    uint32_t bitbuf = 0;
    uint8_t bits = 0;
    uint8_t ch = 0;

    for (uint8_t i = 0; i < 22; i++) {
        bitbuf |= ((uint32_t)payload[i]) << bits;
        bits += 8;

        while (bits >= 11 && ch < 16) {
            channels[ch++] = bitbuf & 0x7FF;
            bitbuf >>= 11;
            bits -= 11;
        }
    }
}

HAL_StatusTypeDef crsf_Init(CRSF_t *crsf,UART_HandleTypeDef *huart){

	crsf->huart=huart;
	crsf->crsf_read_index=0;
	crsf->crsf_write_index=0;
	crsf->crsfstate=CRSF_STATE_0;
	HAL_StatusTypeDef result=HAL_OK;

	for(int i = 0; i<32; i++){
		crsf->channel_armed[i]=1;
	}

	if(HAL_UART_Receive_DMA(crsf->huart,crsf->crsf_circular_buffer,CRSF_CIRC_BUF_LENGHT)!=HAL_OK){
		result=HAL_ERROR;
	}

    __HAL_DMA_ENABLE_IT(crsf->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(crsf->huart->hdmarx, DMA_IT_TC);

	return result;
}




HAL_StatusTypeDef crsf_fsm(CRSF_t *crsf)
{
    uint16_t available = 0;
    static uint8_t frame_len = 0;
    static uint8_t frame_type = 0;
    static uint8_t index = 0;


    if (crsf->crsf_write_index >= crsf->crsf_read_index) {
        available = crsf->crsf_write_index - crsf->crsf_read_index;
    } else {
        available = (CRSF_CIRC_BUF_LENGHT - crsf->crsf_read_index)+ crsf->crsf_write_index;
    }

    if (available <= 4) {
        return HAL_BUSY;
    }

    while (available > 4) {

        if (crsf->crsf_write_index >= crsf->crsf_read_index) {
            available = crsf->crsf_write_index - crsf->crsf_read_index;
        } else {
            available = (CRSF_CIRC_BUF_LENGHT - crsf->crsf_read_index)+ crsf->crsf_write_index;
        }

        switch (crsf->crsfstate) {

        case CRSF_STATE_0:

            if (crsf->crsf_circular_buffer[crsf->crsf_read_index] == CRSF_ADDR_FLIGHT_CONTROLLER) {
                crsf->crsf_trame_handler[0] = CRSF_ADDR_FLIGHT_CONTROLLER;
                index = 1;
                crsf->crsf_read_index =(crsf->crsf_read_index + 1) % CRSF_CIRC_BUF_LENGHT;
                crsf->crsfstate = CRSF_STATE_1;
            } else {
                crsf->crsf_read_index =(crsf->crsf_read_index + 1) % CRSF_CIRC_BUF_LENGHT;
            }
            break;
        case CRSF_STATE_1:
            frame_len = crsf->crsf_circular_buffer[crsf->crsf_read_index];

            if (frame_len < 4 || frame_len > 60) {
                crsf->crsfstate = CRSF_STATE_0;
                break;
            }
            crsf->crsf_trame_handler[1] = frame_len;

            if (crsf->crsf_write_index >= crsf->crsf_read_index) {
                available = crsf->crsf_write_index - crsf->crsf_read_index;
            } else {
                available = (CRSF_CIRC_BUF_LENGHT - crsf->crsf_read_index)+ crsf->crsf_write_index;
            }

            if(frame_len > available){
            	return HAL_BUSY;
            }
            crsf->crsf_read_index =(crsf->crsf_read_index + 1) % CRSF_CIRC_BUF_LENGHT;
            crsf->crsfstate = CRSF_STATE_2;
            break;
        case CRSF_STATE_2:
        	frame_type = crsf->crsf_circular_buffer[crsf->crsf_read_index];
        	if(frame_type == CRSF_TYPE_RC_CHANNELS){
        		index=0;
        		while(index  < frame_len){
        			crsf->crsf_trame_handler[index + 2] =crsf->crsf_circular_buffer[crsf->crsf_read_index];
        			index = index + 1;
                    crsf->crsf_read_index =(crsf->crsf_read_index + 1) % CRSF_CIRC_BUF_LENGHT;
        		}
        		crsf->crsfstate = CRSF_STATE_3;
        	}
        	else{
        		crsf->crsfstate = CRSF_STATE_0;
        	}

            break;

        case CRSF_STATE_3: {
        	uint8_t crc_rx_index = crsf->crsf_trame_handler[1] + 1;
            uint8_t crc_rx =crsf->crsf_trame_handler[crc_rx_index];

            uint8_t crc_calc = crsf_crc8(&crsf->crsf_trame_handler[2], crsf->crsf_trame_handler[1] - 1);

            if (crc_calc == crc_rx) {

                    crsf_decode_channels(&crsf->crsf_trame_handler[3], crsf->crsf_command);
                    crsf->crsfstate = CRSF_STATE_0;
                    return HAL_OK;
            }

            crsf->crsfstate = CRSF_STATE_0;
            break;
        }

        default:
            crsf->crsfstate = CRSF_STATE_0;
            break;
        }
    }

    return HAL_BUSY;
}



HAL_StatusTypeDef crsf_command(CRSF_t *crsf,MOTOR_COM * comstruct)
{
	uint16_t throttle=crsf->crsf_command[2];
	uint16_t ail=crsf->crsf_command[0];

	if((throttle > (CRSF_MIDDLE-THROTTLE_DEAD_BAND)) && (throttle < (CRSF_MIDDLE+THROTTLE_DEAD_BAND))){
		throttle=CRSF_MIDDLE;
	}
	if((ail > (CRSF_MIDDLE-AIL_DEAD_BAND)) && (ail < (CRSF_MIDDLE+AIL_DEAD_BAND))){
		ail=CRSF_MIDDLE;
	}

	comstruct->v=(float)(throttle-CRSF_MIDDLE)*(MAX_MS/CRSF_SPAN);
	comstruct->w=(float)(ail-CRSF_MIDDLE)*(MAX_DEGS/CRSF_SPAN)*(2*M_PI/60.0f);

	limit_vw_by_wheel_speed(comstruct);

	return HAL_OK;
}





