/*
 * AS5047P.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include <math.h>
#include "MA330.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_dmamux.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include <stdbool.h>
#include <string.h>


extern uint32_t tps1;
extern uint32_t tps_tot;


static void cs_low(MA330_t *encd) {
    HAL_GPIO_WritePin(encd->MA330_cs_port, encd->MA330_cs_pin, GPIO_PIN_RESET);
}

static void cs_high(MA330_t *encd) {
	HAL_GPIO_WritePin(encd->MA330_cs_port, encd->MA330_cs_pin, GPIO_PIN_SET);
}



/* ====== INIT DMA seule (post HAL) : ne touche pas SPI2/GPIO ====== */
void SPI2_FOC_DMA_LL_Init(MA330_t *encd)
{
    /* DMAMUX: CH2=SPI2_RX, CH3=SPI2_TX */

    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI2_TX);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_SPI2_RX);

    /* RX: P->M, 8-bit, no inc, length=2 (préchargée) */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&SPI2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)encd->spi_rx_buffer);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
    LL_DMA_ClearFlag_GI2(DMA1);

    /* TX: M->P, 8-bit, no inc, length=2 (préchargée) */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&SPI2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)encd->spi_tx_buffer);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
    LL_DMA_ClearFlag_GI3(DMA1);

    /* IT fin de RX (la seule fiable pour “fin vraie” d’un full-duplex) */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    encd->spi_tx_buffer[0]=0x00;
    encd->spi_tx_buffer[1]=0x00;

    LL_SPI_Enable(SPI2);

    encd->g_spi_done = 0;
    cs_high(encd);


}

/* ====== KICK non bloquant : lance un TxRx 2 octets (8-bit x2) ====== */
bool SPI2_FOC_DMA_KickTxRx2B(MA330_t *encd,uint8_t Byte1,uint8_t Byte2)
{
    if (!LL_SPI_IsEnabled(SPI2)) {
        LL_SPI_Enable(SPI2);
    }
    if (LL_SPI_IsActiveFlag_BSY(SPI2)) {
        return false;
    }
    encd->spi_tx_buffer[0] = Byte1;
    encd->spi_tx_buffer[1] = Byte2;

    encd->g_spi_done = 0;

    cs_low(encd);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
    LL_DMA_ClearFlag_GI2(DMA1);
    LL_DMA_ClearFlag_GI3(DMA1);

    encd->g_spi_done = 0;

    // Activer d’abord RX (prêt à capter), puis TX
    LL_SPI_EnableDMAReq_RX(SPI2);
    LL_SPI_EnableDMAReq_TX(SPI2);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2); // RX
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3); // TX

    return true;
}

/* ====== Handler d’IRQ DMA (RX fin) ====== */





/* ====== Utilitaire bloquant court (polling) si tu en veux un) ====== */
bool SPI2_FOC_DMA_TxRx2B_Blocking(MA330_t * encd, uint8_t Byte1, uint8_t Byte2, uint32_t timeout_loops)
{

    if (!SPI2_FOC_DMA_KickTxRx2B(encd, Byte1, Byte2)) return false;

    HAL_Delay(5);
    uint8_t problem=0;

    //gerer les erreurs
    while (timeout_loops>10) {
        if (encd->g_spi_done == 1){
        	break;
        }
        HAL_Delay(1);
    }
    if (problem == 1) {
        /* Stop propre si timeout */
        LL_SPI_DisableDMAReq_TX(SPI2);
        LL_SPI_DisableDMAReq_RX(SPI2);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        return false;
    }
    /* Nettoyage minimal et lecture */
    LL_SPI_DisableDMAReq_TX(SPI2);
    LL_SPI_DisableDMAReq_RX(SPI2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_ClearFlag_TC3(DMA1);

    return true;
}



//attention 20ms minimum apres une ecriture de registre
int MA330_Init(MA330_t *encd, GPIO_TypeDef *cs_port, uint16_t cs_pin,uint8_t FW){
    if (encd == NULL || cs_port == NULL || cs_pin == 0) {
        return 0;
    }

    encd->MA330_cs_port = cs_port;
    encd->MA330_cs_pin = cs_pin;
    
    SPI2_FOC_DMA_LL_Init(encd);

    HAL_Delay(1);

    if(FW>0){


	if (SPI2_FOC_DMA_TxRx2B_Blocking(encd, 0x4E, 0x00, 1000) != true) {
		cs_high(encd);
        return 0;
    }

	HAL_Delay(1);


	if (SPI2_FOC_DMA_TxRx2B_Blocking(encd, 0x00, 0x00, 1000) != true) {
		cs_high(encd);
        return 0;
    }
	uint8_t actualfw=encd->spi_rx_buffer[0];

	HAL_Delay(1);

	if (SPI2_FOC_DMA_TxRx2B_Blocking(encd, 0x8E,(uint8_t ) FW, 1000) != true) {
		cs_high(encd);
        return 0;
    }

	HAL_Delay(25);

	if (SPI2_FOC_DMA_TxRx2B_Blocking(encd, 0x00, 0x00, 1000) != true) {
		cs_high(encd);
        return 0;
    }

	if(encd->spi_rx_buffer[0]!=FW){
		cs_high(encd);

		return 0;
	}
	cs_high(encd);

    }
    return 1;
}

//14 bit de donnee utile
int MA330_start(MA330_t *encd) {

	if (SPI2_FOC_DMA_KickTxRx2B(encd,0x00,0x00) != true) {
        return 0;
    }

	return 1;
}


float MA330_get_degree(MA330_t *encd) {
	cs_high(encd);

    const uint16_t raw_data =(uint16_t)((encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1]);

    const float angle_raw = (float)((raw_data >> 2) & 0x3FFF) * ANGLE_SCALE_FACTOR;



//    float angle_diff = angle_raw - encd->prev_raw_angle;
//    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * INV_360);//normalisation entre -180 et 180
//
//    if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
//        if (++encd->spike_counter < SPIKE_REJECT_COUNT) {
//            return encd->angle_filtered;
//        }
//        encd->spike_counter = 0;
//    } else {
//        encd->spike_counter = 0;
//    }

    encd->prev_raw_angle = angle_raw;

//    // Filter IIR dengan wrap-around
//    float filtered_diff = angle_raw - encd->angle_filtered;
//    filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) * INV_360);
//    encd->angle_filtered += ANGLE_FILTER_ALPHA * filtered_diff;

//    if (encd->angle_filtered >= 360.0f)
//        encd->angle_filtered -= 360.0f;
//    else if (encd->angle_filtered < 0.0f)
//        encd->angle_filtered += 360.0f;
//
//    return encd->angle_filtered;

}


float MA330_get_rpm(MA330_t *encd, uint32_t dt_us) {
    // Handle angle wrap-around (optimized)
    float angle_diff = encd->angle_filtered - encd->prev_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f)* INV_360 );
    encd->prev_angle = encd->angle_filtered;

    // Calculate RPM (optimized floating point)
    float rpm_instant = (angle_diff * MICROS_TO_MINUTES) / (dt_us * DEGREES_PER_REV);

    // Two-stage spike rejection
    float rpm_delta = rpm_instant - encd->prev_rpm;
    float abs_delta = fabsf(rpm_delta);

    if (abs_delta > MAX_RPM_JUMP) {
        // Gradual rejection with 50% of the delta, capped at MAX_RPM_JUMP
        float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
        rpm_instant = encd->prev_rpm + limited_delta;
    }

    // IIR Filter with dynamic weighting
    float filtered = encd->filtered_rpm * (1.0f - RPM_FILTER_ALPHA) + rpm_instant * RPM_FILTER_ALPHA;

    // Very low RPM clamping (0.1 RPM resolution)
    if (fabsf(filtered) < 0.1f) {
        filtered = 0.0f;
    }

    // Update state
    encd->prev_rpm = rpm_instant;
    encd->filtered_rpm = filtered;

    return encd->filtered_rpm;
}

float MA330_get_actual_degree(MA330_t *encd) {
    const float m_current_angle = encd->angle_filtered;
	float angle_dif = (m_current_angle - encd->output_prev_angle);

	if (angle_dif< -180) {
		encd->output_angle_ovf++;
	}
	else if (angle_dif> 180) {
		encd->output_angle_ovf--;
	}
	float out_deg = (m_current_angle + encd->output_angle_ovf * 360.0);
    encd->output_angle_filtered = (1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * encd->output_angle_filtered + ACTUAL_ANGLE_FILTER_ALPHA * out_deg;
	encd->output_prev_angle = m_current_angle;

	// return out_deg;
    return encd->output_angle_filtered;
}



void Put_inside_DMA1_Channel2_3_IRQ(MA330_t *encd)
{

    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);

        // Stop requêtes / canaux
        LL_SPI_DisableDMAReq_RX(SPI2);
        LL_SPI_DisableDMAReq_TX(SPI2);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        cs_high(encd);

        MA330_get_degree(encd);

        tps_tot=(uint32_t ) TIM2->CNT-tps1;

        encd->g_spi_done = 1;
        encd->g_spi_error=1;

    }

    if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
        LL_DMA_ClearFlag_TE2(DMA1);
        // erreur
    }

    if (LL_DMA_IsActiveFlag_TE3(DMA1)) {
        LL_DMA_ClearFlag_TE3(DMA1);
        // erreur
    }

}

//int MA330_Init(MA330_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,uint8_t FW){
//    if (encd == NULL || hspi == NULL || cs_port == NULL || cs_pin == 0) {
//        return 0;
//    }
//
//    encd->MA330_spi = hspi;
//    encd->MA330_cs_port = cs_port;
//    encd->MA330_cs_pin = cs_pin;
//
//    cs_high(encd);
//
//    HAL_Delay(1);
//
//    if(FW>0){
//    cs_low(encd);
//    uint8_t receive_buffer[2];
//    uint8_t send_buffer[]={0x4E,0x00};
//
//	if (HAL_SPI_TransmitReceive(encd->MA330_spi, send_buffer, receive_buffer, 2, 1000) != HAL_OK) {
//		cs_high(encd);
//        return 0;
//    }
//	cs_high(encd);
//	HAL_Delay(1);
//	cs_low(encd);
//
//	send_buffer[0]=0x00;
//	send_buffer[1]=0x00;
//	if (HAL_SPI_TransmitReceive(encd->MA330_spi, send_buffer, receive_buffer, 2, 1000) != HAL_OK) {
//		cs_high(encd);
//        return 0;
//    }
//	//uint8_t actualfw=receive_buffer[0];
//	cs_high(encd);
//	HAL_Delay(1);
//	cs_low(encd);
//
//	send_buffer[0]=0x8E;
//	send_buffer[1]=FW;
//
//	if (HAL_SPI_TransmitReceive(encd->MA330_spi, send_buffer, receive_buffer, 2, 1000) != HAL_OK) {
//		cs_high(encd);
//        return 0;
//    }
//
//	cs_high(encd);
//	HAL_Delay(25);
//	cs_low(encd);
//
//	send_buffer[0]=0x00;
//	send_buffer[1]=0x00;
//
//	if (HAL_SPI_TransmitReceive(encd->MA330_spi, send_buffer, receive_buffer, 2, 1000) != HAL_OK) {
//		cs_high(encd);
//        return 0;
//    }
//
//	if(receive_buffer[0]!=FW){
//		cs_high(encd);
//
//		return 0;
//	}
//	cs_high(encd);
//
//    }
//    return 1;
//}
//
////14 bit de donnee utile
//int MA330_start(MA330_t *encd) {
//    uint8_t dummy[2];
//    dummy[0]=0x00;
//    dummy[1]=0x00;
//
//	cs_low(encd);
//	if (HAL_SPI_TransmitReceive_DMA(encd->MA330_spi, (uint8_t*)dummy, encd->spi_rx_buffer, 2) != HAL_OK) {
//        return 0;
//    }
//
//	return 1;
//}

