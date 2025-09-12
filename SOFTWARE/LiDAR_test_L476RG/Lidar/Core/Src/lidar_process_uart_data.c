
#include "lidar_process_uart_data.h"

uint16_t lidar_data[LIDAR_RESOLUTION];/* tableau pour 0.5° résolution */
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

void lidar_process_uart_data(uint8_t *buffer, uint16_t length) {
	//const char *message = "USART1_Lidar_process\r\n";
	//HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

    for (int i = 0; i < length - 1; i++) {
        if (buffer[i] == 0x55 && buffer[i+1] == 0xAA) {
            uint8_t lsn = buffer[i+3];
            uint16_t fsa = buffer[i+4] | (buffer[i+5] << 8);
            uint16_t lsa = buffer[i+6] | (buffer[i+7] << 8);

            for (int j = 0; j < lsn; j++) {
                uint16_t dist = buffer[i+10 + j*2] | (buffer[i+11 + j*2] << 8);
                float angle = fsa + ((float)(lsa - fsa) * j) / (lsn - 1);
                int index = (int)(angle * LIDAR_RESOLUTION / 360.0f) % LIDAR_RESOLUTION;
                lidar_data[index] = dist;
            }

            i += 10 + 2 * lsn - 1; /* sauter à la prochaine trame */
        }
    }

    char buffer_tx[512]; // buffer pour une tranche
    char temp[8];

    for (size_t i = 0; i < LIDAR_RESOLUTION; i += 90) { // tranches de 45°
        buffer_tx[0] = '\0'; // reset

        for (size_t j = 0; j < 90; j++) {
            snprintf(temp, sizeof(temp), "%d,", lidar_data[i + j]);
            strcat(buffer_tx, temp);
        }

        // Remplacer la dernière virgule par \r\n
        size_t len = strlen(buffer_tx);
        buffer_tx[len - 1] = '\r';
        buffer_tx[len] = '\n';
        buffer_tx[len + 1] = '\0';

        HAL_UART_Transmit(&huart2, (uint8_t*)buffer_tx, strlen(buffer_tx), HAL_MAX_DELAY);

    }
}
