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

#define NORMAL_MODE
//#define DEBUG_MODE




#define MOTOR_FRAME_MOTOR_COUNT  3
#define MOTOR_FRAME_SIZE         30
#define MOTOR_COM_RX_BUF_SIZE    128
#define FRAME_HEADER    0x55AA

typedef union {
    struct {
        uint16_t header;
        uint8_t  flags;
        uint8_t  counter;
        float    targetSpeed[MOTOR_FRAME_MOTOR_COUNT];
        float    actualSpeed[MOTOR_FRAME_MOTOR_COUNT];
        uint16_t crc;
    } f;
    uint8_t raw[MOTOR_FRAME_SIZE];
} MotorFrameUnion_t;

typedef struct {

    UART_HandleTypeDef *huart;
    MotorFrameUnion_t rx_struct;
    MotorFrameUnion_t tx_buf;
    uint8_t timer_flag;

    uint8_t tx_flag;

    uint16_t 		write_index;
    uint16_t        read_index;

    uint8_t rx_buf[MOTOR_COM_RX_BUF_SIZE];

#ifdef DEBUG_MODE
    char     dbg_tx_buf[64];   // buffer texte pour snprintf
    char     dbg_rx_buf[32];   // buffer réception ligne de commande
    uint8_t  dbg_rx_index;     // index dans dbg_rx_buf
#endif




} MOTOR_COM;


HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim);
HAL_StatusTypeDef MotorCom_Process(MOTOR_COM * comstruct);

#endif /* INC_MCC_COM_SLAVE_H_ */
