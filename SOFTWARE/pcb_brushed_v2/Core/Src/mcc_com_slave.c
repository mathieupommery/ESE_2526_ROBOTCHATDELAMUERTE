#include "mcc_com_slave.h"
#include "mcc_control.h"
#include <string.h>


extern Motor_t g_motors[3];
extern MOTOR_COM com_struct;

typedef enum {
    LOOKHEADER = 0,
    CHECKFRAME,
    PROCESSANDSEND
}RXSTATE;

uint32_t t4=0;
uint32_t t5=0;
uint32_t t6=0;
uint32_t t7=0;
uint32_t tc=0;
uint32_t td=0;
int flag=0;



HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim){

	comstruct->huart=huart;
	comstruct->write_index=0;
	comstruct->read_index=0;
	comstruct->tx_buf.f.header=FRAME_HEADER;
	comstruct->tx_flag=0;
	HAL_StatusTypeDef result=HAL_OK;

	if(HAL_TIM_Base_Start_IT(htim)!=HAL_OK){
		result=HAL_ERROR;
	}

	if(HAL_UART_Receive_DMA(comstruct->huart,comstruct->rx_buf,MOTOR_COM_RX_BUF_SIZE)!=HAL_OK){
		result=HAL_ERROR;
	}
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);
return result;
}




HAL_StatusTypeDef MotorCom_Process(MOTOR_COM * comstruct){


	if(comstruct->timer_flag!=1){
		__NOP();
		return HAL_BUSY;
	}
	comstruct->timer_flag=0;
	HAL_StatusTypeDef result=HAL_BUSY;

#ifdef NORMAL_MODE


	RXSTATE rxstate=LOOKHEADER;

	uint16_t index=comstruct->read_index;
	uint16_t available=0;
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
			uint16_t tempheader=(comstruct->rx_buf[index]<<8)|(comstruct->rx_buf[(index+1)%MOTOR_COM_RX_BUF_SIZE]);
			if(tempheader==0xAA55){
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
			rxstate=LOOKHEADER;

			Motor_SetTargetSpeed(&g_motors[0],comstruct->rx_struct.f.targetSpeed[0]);
			Motor_SetTargetSpeed(&g_motors[1],comstruct->rx_struct.f.targetSpeed[1]);
			Motor_SetTargetSpeed(&g_motors[2],comstruct->rx_struct.f.targetSpeed[2]);

			comstruct->tx_buf.f.targetSpeed[0]=0.0f;
			comstruct->tx_buf.f.targetSpeed[1]=0.0f;
			comstruct->tx_buf.f.targetSpeed[2]=0.0f;

			comstruct->tx_buf.f.actualSpeed[0]=g_motors[0].speedRpm;
			comstruct->tx_buf.f.actualSpeed[1]=g_motors[1].speedRpm;
			comstruct->tx_buf.f.actualSpeed[2]=g_motors[2].speedRpm;

			comstruct->tx_buf.f.counter=comstruct->rx_struct.f.counter;
			comstruct->tx_buf.f.flags=0;
			comstruct->tx_buf.f.header=FRAME_HEADER;
			comstruct->tx_buf.f.crc=0;

			comstruct->read_index=(comstruct->read_index+MOTOR_FRAME_SIZE)%MOTOR_COM_RX_BUF_SIZE;

				if (HAL_UART_Transmit(comstruct->huart,(uint8_t*)comstruct->tx_buf.raw,MOTOR_FRAME_SIZE,100) == HAL_OK) {
			        return HAL_OK;
			    }
			    else {
			        return HAL_ERROR;
			    }

			break;
		}

	}
	return result;
#endif
#ifdef DEBUG_MODE
	uint16_t len = 0;
	 len = snprintf(comstruct->dbg_tx_buf,64,"%0.2f,%0.2f\r\n",g_motors[0].targetSpeedRpm,g_motors[0].speedRpm);
	 flag=1-flag;
	 if(flag){
		 t4= DWT->CYCCNT;
	 }
	 else{
		 t5=DWT->CYCCNT-t4;
	 }
	 if(t5>tc){
		 tc=t5;
	 }

	 t6= DWT->CYCCNT;
	 if (HAL_UART_Transmit(comstruct->huart,(uint8_t*)comstruct->dbg_tx_buf,len,100) == HAL_OK) {
		        result=HAL_OK;
		    }
	 else {
		        result=HAL_ERROR;
		    }
	 t7= DWT->CYCCNT - t6;
	 if(t7>td){
		 td=t7;
	 }
	 return result;
#endif

}

