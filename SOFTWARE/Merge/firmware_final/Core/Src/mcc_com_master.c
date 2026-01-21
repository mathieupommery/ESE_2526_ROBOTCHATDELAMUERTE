#include "mcc_com_master.h"
#include <string.h>
#include "usart.h"
#include "usbd_cdc_if.h"

extern MOTOR_COM com_struct;

typedef enum {
    LOOKHEADER = 0,
    CHECKFRAME,
    PROCESSANDSEND
}RXSTATE;

uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;

        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}



HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart){

	comstruct->huart=huart;
	comstruct->write_index=0;
	comstruct->read_index=0;
	comstruct->cnt_frame_late=0;
	comstruct->cnt_frame_lost=0;
	HAL_StatusTypeDef result=HAL_OK;

	comstruct->v=0.0f;
	comstruct->w=0.0f;

#ifdef NORMAL_MODE
	if(HAL_UART_Receive_DMA(comstruct->huart,comstruct->rx_buf,MOTOR_COM_RX_BUF_SIZE)!=HAL_OK){
		result=HAL_ERROR;
	}
#endif
#ifdef DEBUG_MODE
	if(HAL_UART_Receive_DMA(comstruct->huart,comstruct->dbg_rx_buf,DEBUG_RX_BUF_SIZE)!=HAL_OK){
		result=HAL_ERROR;
	}
#endif

    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);
return result;
}




HAL_StatusTypeDef MotorCom_Process(MOTOR_COM * comstruct){

	HAL_StatusTypeDef result=HAL_BUSY;

#ifdef NORMAL_MODE
	RXSTATE rxstate=LOOKHEADER;
	uint8_t temp_buf[MOTOR_FRAME_SIZE];
	uint16_t index=comstruct->read_index;
	uint16_t available=0;


	comstruct->tx_struct.f.header=FRAME_HEADER;
	comstruct->tx_struct.f.flags=0;

	if(comstruct->tx_struct.f.counter>=255){
		comstruct->tx_struct.f.counter=0;
	}
	else{
	comstruct->tx_struct.f.counter=comstruct->tx_struct.f.counter+1;
	}

	comstruct->tx_struct.f.actualv=0.0f;
	comstruct->tx_struct.f.actualw=0.0f;
	comstruct->tx_struct.f.actualtime=0;


	comstruct->tx_struct.f.targetv=comstruct->v;
	comstruct->tx_struct.f.targetw=comstruct->w;
	comstruct->tx_struct.f.time=0;

	comstruct->tx_struct.f.crc=crc16_ccitt((uint8_t *)comstruct->tx_struct.raw, MOTOR_FRAME_SIZE-2);


	    if (HAL_UART_Transmit(comstruct->huart,(uint8_t *)comstruct->tx_struct.raw, MOTOR_FRAME_SIZE,100) == HAL_OK) {
	        result=HAL_OK;
	    }
	    else {
	        result=HAL_ERROR;
	    }



	if (comstruct->write_index >= comstruct->read_index) {
		available = comstruct->write_index - comstruct->read_index;
	}
	else {
		available = (MOTOR_COM_RX_BUF_SIZE - comstruct->read_index) + comstruct->write_index;
	}



	while(available>=2){

		index = comstruct->read_index;

	    if (comstruct->write_index >= comstruct->read_index) {
	        available = comstruct->write_index - comstruct->read_index;
	    } else {
	        available = (MOTOR_COM_RX_BUF_SIZE - comstruct->read_index)+ comstruct->write_index;
	    }

	    if (available < 2)
	    	{
	    	return HAL_BUSY;
	    	}


		switch(rxstate){

		case LOOKHEADER:
		{
			uint16_t tempheader=(comstruct->rx_buf[(index+1)%MOTOR_COM_RX_BUF_SIZE]<<8)|(comstruct->rx_buf[index]);
			if(tempheader==FRAME_HEADER){
				rxstate=CHECKFRAME;
			}
			else{
				comstruct->read_index=(comstruct->read_index+1)%MOTOR_COM_RX_BUF_SIZE;
			}
			break;
		}
		case CHECKFRAME:
			if(available>=MOTOR_FRAME_SIZE){

				for(int i=0;i<(MOTOR_FRAME_SIZE);i++){

					temp_buf[i]=comstruct->rx_buf[(index+i)%MOTOR_COM_RX_BUF_SIZE];
				}

				uint16_t checksum=crc16_ccitt((uint8_t *)temp_buf, MOTOR_FRAME_SIZE-2);
				uint16_t crc=(uint16_t) (temp_buf[MOTOR_FRAME_SIZE-1]<<8)|temp_buf[MOTOR_FRAME_SIZE-2];
				if(checksum == crc ){
						rxstate=PROCESSANDSEND;
					}
				else{
									comstruct->cnt_frame_lost++;
									comstruct->read_index=(comstruct->read_index+1)%MOTOR_COM_RX_BUF_SIZE;
									rxstate=LOOKHEADER;
					}
			}
			else{
				comstruct->cnt_frame_late++;
				return HAL_BUSY;
			}
			break;
		case PROCESSANDSEND:
			for(int i=0;i<MOTOR_FRAME_SIZE;i++){
				comstruct->rx_struct.raw[i]=comstruct->rx_buf[(index+i)%MOTOR_COM_RX_BUF_SIZE];
			}
			comstruct->read_index=(comstruct->read_index+MOTOR_FRAME_SIZE)%MOTOR_COM_RX_BUF_SIZE;
			rxstate=LOOKHEADER;
			return HAL_OK;

			break;
		}

	}
	return result;
#endif

}



