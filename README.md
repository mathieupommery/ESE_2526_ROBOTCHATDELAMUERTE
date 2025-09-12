# Robot Chat omniwheel


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        // Copier les 1ers 64 octets
        for (int i = 0; i < DMA_CHUNK_SIZE / 2; i++) {
            circular_buffer[write_index] = dma_rx_buffer[i];
            write_index = (write_index + 1) % CIRC_BUF_SIZE;
        }
    }
}
