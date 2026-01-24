/*
 * crsf.h
 *
 *  Created on: Jan 21, 2026
 *      Author: mathi
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#include "main.h"
#include "mcc_com_master.h"
#include "odometry.h"

#define  CRSF_CIRC_BUF_LENGHT 512

typedef enum {
    CRSF_STATE_0 = 0,
    CRSF_STATE_1,
    CRSF_STATE_2,
    CRSF_STATE_3
} CRSF_STATE;

#define CRSF_ADDR_FLIGHT_CONTROLLER  0xC8
#define CRSF_TYPE_RC_CHANNELS        0x16
#define CRSF_MAX_FRAME_SIZE          64
#define CRSF_CHANNEL_COUNT           16

#define CRSF_MIDDLE           1000.0f
#define CRSF_SPAN           1000.0f


#define MAX_MS 1.5f
#define MAX_DEGS 200.0f

#define THROTTLE_DEAD_BAND 75u
#define AIL_DEAD_BAND 70u

typedef struct{
	UART_HandleTypeDef *huart;

uint16_t crsf_read_index;
uint16_t crsf_write_index;
uint8_t crsf_circular_buffer[CRSF_CIRC_BUF_LENGHT];
CRSF_STATE crsfstate;
uint8_t crsf_trame_handler[128];
uint16_t crsf_command[32];
uint8_t channel_armed[32];
}CRSF_t;


HAL_StatusTypeDef crsf_fsm(CRSF_t *crsf);
HAL_StatusTypeDef crsf_command(CRSF_t *crsf,MOTOR_COM * comstruct);
HAL_StatusTypeDef crsf_Init(CRSF_t *crsf,UART_HandleTypeDef *huart);



#endif /* INC_CRSF_H_ */
