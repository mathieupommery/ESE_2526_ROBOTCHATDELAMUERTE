#include "mcc_com_master.h"
#include <string.h>
#include "usart.h"
#include "usbd_cdc_if.h"

MOTOR_COM com_struct;

typedef enum {
    LOOKHEADER = 0,
    CHECKFRAME,
    PROCESSANDSEND
}RXSTATE;



HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart){

	comstruct->huart=huart;
	comstruct->write_index=0;
	comstruct->read_index=0;
	comstruct->tx_buf.f.header=FRAME_HEADER;
	HAL_StatusTypeDef result=HAL_OK;

#ifdef NORMAL_MODE
	if(HAL_UART_Receive_DMA(comstruct->huart,comstruct->rx_buf,MOTOR_COM_RX_BUF_SIZE)!=HAL_OK){
		result=HAL_ERROR;
	}
#endif
#ifdef DEBUG_MODE
	if(HAL_UART_Receive_DMA(comstruct->huart,comstruct->rx_buf,2)!=HAL_OK){
		result=HAL_ERROR;
	}
#endif
return result;
}




HAL_StatusTypeDef MotorCom_Process(MOTOR_COM * comstruct){

	HAL_StatusTypeDef result=HAL_BUSY;

#ifdef NORMAL_MODE
	RXSTATE rxstate=LOOKHEADER;

	uint16_t index=comstruct->read_index;
	uint16_t available=0;

	if(comstruct->counter>=255){
		comstruct->counter=0;
	}
	else{
	comstruct->counter=comstruct->counter+1;
	}


	comstruct->tx_buf.f.targetSpeed[0]=comstruct->w0;
	comstruct->tx_buf.f.targetSpeed[1]=comstruct->w1;
	comstruct->tx_buf.f.targetSpeed[2]=comstruct->w2;

	comstruct->tx_buf.f.actualSpeed[0]=0.0f;
	comstruct->tx_buf.f.actualSpeed[1]=0.0f;
	comstruct->tx_buf.f.actualSpeed[2]=0.0f;

	comstruct->tx_buf.f.counter=comstruct->counter;
	comstruct->tx_buf.f.flags=0;
	comstruct->tx_buf.f.header=FRAME_HEADER;
	comstruct->tx_buf.f.crc=0;

	comstruct->read_index=(comstruct->read_index+MOTOR_FRAME_SIZE)%MOTOR_COM_RX_BUF_SIZE;

	if (comstruct->tx_flag == 0) {
	    if (HAL_UART_Transmit_DMA(comstruct->huart,(uint8_t *)comstruct->tx_buf.raw, MOTOR_FRAME_SIZE) == HAL_OK) {
	        comstruct->tx_flag = 1;
	        result=HAL_OK;
	    }
	    else {
	        result=HAL_ERROR;
	    }
	}
	else {
		result=HAL_BUSY;
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
	        available = (MOTOR_COM_RX_BUF_SIZE - comstruct->read_index)
	                    + comstruct->write_index;
	    }

	    if (available < 2)
	    	{
	    	break;
	    	}





		switch(rxstate){

		case LOOKHEADER:
		{
			uint16_t tempheader=(comstruct->rx_buf[index]<<8)|(comstruct->rx_buf[(index+1)%MOTOR_COM_RX_BUF_SIZE]);
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
				//checksum here
				rxstate=PROCESSANDSEND;
			}
			else{
				return HAL_BUSY;
			}


			break;
		case PROCESSANDSEND:
			for(int i=0;i<MOTOR_FRAME_SIZE;i++){
				comstruct->rx_struct.raw[i]=comstruct->rx_buf[(index+i)%MOTOR_COM_RX_BUF_SIZE];
			}
			return HAL_OK;

			break;
		}

	}
	return result;
#endif
#ifdef DEBUG_MODE




		return result;
#endif

}




void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef NORMAL_MODE
        com_struct.write_index=(uint16_t)(MOTOR_COM_RX_BUF_SIZE / 2u);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef NORM_MODE
        com_struct.write_index=0;
#endif
#ifdef DEBUG_MODE
    	CDC_Transmit_FS((uint8_t *)com_struct.dbg_rx_buf, 2);
    	HAL_UART_Receive_DMA(&huart1, (uint8_t *)com_struct.dbg_rx_buf, 2);
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
        com_struct.tx_flag=0;
}
