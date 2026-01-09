/*
 * mcc_com.h
 *
 *  Created on: Nov 29, 2025
 *      Author: mathi
 */

#ifndef INC_MCC_COM_SLAVE_H_
#define INC_MCC_COM_SLAVE_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <math.h>

#define NORMAL_MODE
//#define DEBUG_MODE




#define MOTOR_FRAME_MOTOR_COUNT  3
#define MOTOR_FRAME_SIZE         30
#define MOTOR_COM_RX_BUF_SIZE    128
#define FRAME_HEADER    0x55AA

#define RPM_2_RAD_S   ((2.0f * M_PI / 60.0f))
#define RAD_S_2_RPM   (60.0f / (2.0f * M_PI))

typedef union {
    struct {
        uint16_t header;
        uint8_t  flags;
        uint8_t  counter;

        float targetv;
        float targetw;
        uint32_t time;

        float actualv;
		float actualw;
		uint32_t actualtime;

        uint16_t crc;
    } f;
    uint8_t raw[MOTOR_FRAME_SIZE];
} MotorFrameUnion_t;

typedef struct {


	TIM_HandleTypeDef *basehtim;
    UART_HandleTypeDef *huart;
    uint8_t rx_buf[MOTOR_COM_RX_BUF_SIZE];
    MotorFrameUnion_t rx_struct;
    MotorFrameUnion_t tx_struct;
    volatile uint16_t 		write_index;
    volatile uint16_t        read_index;
    volatile uint8_t timer_flag;

    uint32_t cnt_frame_late;
    uint32_t cnt_frame_lost;

#ifdef DEBUG_MODE
    char     dbg_tx_buf[64];   // buffer texte pour snprintf
    char     dbg_rx_buf[32];   // buffer réception ligne de commande
    uint8_t  dbg_rx_index;     // index dans dbg_rx_buf
#endif


} MOTOR_COM;


HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim,TIM_HandleTypeDef *basehtim);
HAL_StatusTypeDef MotorCom_Process(MOTOR_COM * comstruct);

#endif /* INC_MCC_COM_SLAVE_H_ */
