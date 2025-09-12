/*
 * lidar_process_uart_data.h
 *
 *  Created on: Sep 11, 2025
 *      Author: lorenzo
 */

#ifndef INC_LIDAR_PROCESS_UART_DATA_H_
#define INC_LIDAR_PROCESS_UART_DATA_H_

#include "main.h"

#define LIDAR_RESOLUTION 360
extern uint16_t lidar_data[LIDAR_RESOLUTION];

#define UART_RX_BUFFER_SIZE 512
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static uint16_t last_pos;

void lidar_process_uart_data(uint8_t *buffer, uint16_t length);

extern UART_HandleTypeDef huart2;

#endif /* INC_LIDAR_PROCESS_UART_DATA_H_ */
