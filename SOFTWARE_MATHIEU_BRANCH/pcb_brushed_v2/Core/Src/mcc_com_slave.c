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



HAL_StatusTypeDef MotorCom_Init(MOTOR_COM * comstruct,UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim,TIM_HandleTypeDef *basehtim){
	comstruct->basehtim=basehtim;
	comstruct->huart=huart;
	comstruct->write_index=0;
	comstruct->read_index=0;
	comstruct->cnt_frame_late=0;
	comstruct->cnt_frame_lost=0;
	HAL_StatusTypeDef result=HAL_OK;

	if(HAL_TIM_Base_Start_IT(htim)!=HAL_OK){
		result=HAL_ERROR;
	}

	if(HAL_TIM_Base_Start(basehtim)!=HAL_OK){
		result=HAL_ERROR;
	}
	basehtim->Instance->CNT=0;

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
	uint8_t temp_buf[MOTOR_FRAME_SIZE];
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
			rxstate=LOOKHEADER;

			float w0=0.0f;//moteur de gauche
			float w1=0.0f;//moteur de droite
			float v=comstruct->rx_struct.f.targetv;
			float w=comstruct->rx_struct.f.targetw;
			float actualv=0.0f;
			float actualw=0.0f;
			uint32_t actualtime=0;

			w0=((v + 0.5f * L_DIST * w) / R_WHEEL)*RAD_S_2_RPM;
			w1=((v - 0.5f * L_DIST * w) / R_WHEEL)*RAD_S_2_RPM;

			actualv=0.5f * R_WHEEL * (g_motors[1].speedRpm - g_motors[0].speedRpm) * RPM_2_RAD_S;
			actualw=-(R_WHEEL / L_DIST) * (g_motors[0].speedRpm + g_motors[1].speedRpm) * RPM_2_RAD_S;
			actualtime=(uint32_t ) comstruct->basehtim->Instance->CNT;
			comstruct->basehtim->Instance->CNT=0;

			Motor_SetTargetSpeed(&g_motors[0],-w0);
			Motor_SetTargetSpeed(&g_motors[1],w1);





			comstruct->tx_struct.f.header=FRAME_HEADER;
			comstruct->tx_struct.f.flags=0;
			comstruct->tx_struct.f.counter=comstruct->rx_struct.f.counter;


			comstruct->tx_struct.f.actualv=actualv;
			comstruct->tx_struct.f.actualw=actualw;
			comstruct->tx_struct.f.actualtime=actualtime;




			comstruct->tx_struct.f.targetv=0.0f;
			comstruct->tx_struct.f.targetw=0.0f;
			comstruct->tx_struct.f.time=0;


			comstruct->tx_struct.f.crc=crc16_ccitt((uint8_t *)comstruct->tx_struct.raw, MOTOR_FRAME_SIZE-2);


			comstruct->read_index = (comstruct->read_index + MOTOR_FRAME_SIZE) % MOTOR_COM_RX_BUF_SIZE;

				if (HAL_UART_Transmit(comstruct->huart,(uint8_t*)comstruct->tx_struct.raw,MOTOR_FRAME_SIZE,100) == HAL_OK) {
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

