/*
 * led.c
 *
 *  Created on: Mar 14, 2025
 *      Author: mathi
 */


#include "led.h"

uint8_t  tomjerry_grb[WS2812_GRB_BUF_LEN(TOM_JERRY_LED_COUNT)];
uint32_t tomjerry_dma[WS2812_DMA_BUF_LEN(TOM_JERRY_LED_COUNT)];

/* Bandeau 2 : externe (30 LED) */
uint8_t  ext_grb[WS2812_GRB_BUF_LEN(EXT_LED_COUNT)];
uint32_t ext_dma[WS2812_DMA_BUF_LEN(EXT_LED_COUNT)];


/* Roue de couleur type NeoPixel: pos 0–255 -> RGB (non gamma-corrigé) */
static void WS2812_ColorWheel(uint8_t pos, uint8_t brightness,
                              uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t r_raw, g_raw, b_raw;

    if (pos < 85U) {
        r_raw = (uint8_t)(pos * 3U);
        g_raw = (uint8_t)(255U - pos * 3U);
        b_raw = 0U;
    }
    else if (pos < 170U) {
        pos -= 85U;
        r_raw = (uint8_t)(255U - pos * 3U);
        g_raw = 0U;
        b_raw = (uint8_t)(pos * 3U);
    }
    else {
        pos -= 170U;
        r_raw = 0U;
        g_raw = (uint8_t)(pos * 3U);
        b_raw = (uint8_t)(255U - pos * 3U);
    }

    /* Application de la luminosité globale (0–255) */
    uint16_t bri = brightness;
    *r = (uint8_t)((r_raw * bri) / 255U);
    *g = (uint8_t)((g_raw * bri) / 255U);
    *b = (uint8_t)((b_raw * bri) / 255U);
}



HAL_StatusTypeDef LED_Strip_Init(led_strip_t *strip,TIM_HandleTypeDef *htim,uint32_t tim_channel,uint8_t *grb_buf,uint16_t led_count,uint32_t *dma_buf,uint16_t dma_buf_len)
{
    if (strip == NULL || htim == NULL || grb_buf == NULL || dma_buf == NULL) {
        return HAL_ERROR;
    }

    if (led_count == 0U) {
        return HAL_ERROR;
    }

    uint16_t min_dma_len = WS2812_DMA_BUF_LEN(led_count);
    if (dma_buf_len < min_dma_len) {
        return HAL_ERROR;
    }

    strip->htim         = htim;
    strip->tim_channel  = tim_channel;
    strip->led_count    = led_count;
    strip->grb_buf      = grb_buf;
    strip->dma_buf      = dma_buf;
    strip->dma_buf_len  = dma_buf_len;
    strip->dma_len_used = 0U;
    strip->dma_busy     = 0U;

    for (uint16_t i = 0; i < (uint16_t)(3U * led_count); i++) {
        strip->grb_buf[i] = 0U;
    }
    for (uint16_t i = 0; i < dma_buf_len; i++) {
        strip->dma_buf[i] = 0U;
    }

    return HAL_TIM_PWM_Init(strip->htim);
}


void LED_Strip_SetPixelRGB(led_strip_t *strip,uint16_t index,uint8_t r, uint8_t g, uint8_t b)
{
    if (strip == NULL) {
        return;
    }

    if (index >= strip->led_count) {
        return;
    }
    strip->grb_buf[3U * index + 0U] = g;
    strip->grb_buf[3U * index + 1U] = r;
    strip->grb_buf[3U * index + 2U] = b;
}


void LED_Strip_SetRGBArray(led_strip_t *strip,const uint8_t *rgb,uint16_t led_count)
{
    if (strip == NULL || rgb == NULL) {
        return;
    }

    if (led_count > strip->led_count) {
        led_count = strip->led_count;
    }

    /* Remplissage des LED utilisées */
    for (uint16_t i = 0; i < led_count; i++) {
        uint8_t r = rgb[3U * i + 0U];
        uint8_t g = rgb[3U * i + 1U];
        uint8_t b = rgb[3U * i + 2U];

        /* Ordre GRB */
        strip->grb_buf[3U * i + 0U] = g;
        strip->grb_buf[3U * i + 1U] = r;
        strip->grb_buf[3U * i + 2U] = b;
    }

    /* Les LED restantes sont éteintes */
    for (uint16_t i = led_count; i < strip->led_count; i++) {
        strip->grb_buf[3U * i + 0U] = 0U;
        strip->grb_buf[3U * i + 1U] = 0U;
        strip->grb_buf[3U * i + 2U] = 0U;
    }
}


/* Construit strip->dma_buf à partir de strip->grb_buf et lance le DMA */
HAL_StatusTypeDef LED_Strip_Refresh(led_strip_t *strip)
{
    if (strip == NULL) {
        return HAL_ERROR;
    }

    if (strip->dma_busy) {
        return HAL_BUSY;
    }

    uint16_t dmabufindex = 0;
    const uint16_t nb_led = strip->led_count;

    /* Longueur minimale théorique */
    uint16_t min_len = WS2812_MinDmaBufferLen(nb_led);
    if (min_len > strip->dma_buf_len) {
        /* Buffer DMA trop petit */
        return HAL_ERROR;
    }

    /* Construction du DMA à partir du buffer GRB */
    for (uint16_t i = 0; i < nb_led; i++) {
        for (uint8_t j = 0; j < 3U; j++) {
            uint8_t val = strip->grb_buf[3U * i + j];

            /* Envoi des 8 bits, LSB first, comme ton code initial */
            for (uint8_t k = 0; k < 8U; k++) {
                if ((val >> k) & 0x01U) {
                    strip->dma_buf[dmabufindex] = WS2812_HI_VAL;
                } else {
                    strip->dma_buf[dmabufindex] = WS2812_LOW_VAL;
                }
                dmabufindex++;
            }
        }
    }

    /* Période de reset */
    for (uint16_t i = 0; i < WS2812_RST_PERIOD; i++) {
        strip->dma_buf[dmabufindex++] = 0U;
    }

    /* Petit padding de sécurité */
    for (uint8_t i = 0; i < 5U; i++) {
        strip->dma_buf[dmabufindex++] = 0U;
    }

    strip->dma_len_used = dmabufindex;
    HAL_StatusTypeDef status = HAL_OK;
    if(strip->dma_busy==0){
    	status = HAL_TIM_PWM_Start_DMA(strip->htim,strip->tim_channel,(uint32_t *)strip->dma_buf,strip->dma_len_used);
        if (status == HAL_OK) {
            strip->dma_busy = 1;
        }
    }
    else {
    	return HAL_BUSY;
    }


    return status;
}


void LED_Strip_DMA_Callback(led_strip_t *strip)
{
    if (strip == NULL) {
        return;
    }

    HAL_TIM_PWM_Stop_DMA(strip->htim, strip->tim_channel);
    strip->dma_busy = 0;
}


void LED_Strip_FillRainbow(led_strip_t *strip,uint16_t offset,uint8_t brightness)
{
    if (strip == NULL || strip->led_count == 0U) {
        return;
    }

    const uint16_t n = strip->led_count;

    /* Si brightness = 0 => tout éteint */
    if (brightness == 0U) {
        for (uint16_t i = 0; i < n; i++) {
            LED_Strip_SetPixelRGB(strip, i, 0U, 0U, 0U);
        }
        return;
    }

    /* On mappe la bande sur la roue 0–255.
       offset permet de faire tourner le rainbow. */
    for (uint16_t i = 0; i < n; i++) {
        /* Étale 0–255 sur la longueur du strip */
        uint32_t step = (uint32_t)256U * i / n;
        uint16_t hue  = (uint16_t)(offset + step);
        uint8_t pos   = (uint8_t)(hue & 0xFFU);

        uint8_t r, g, b;
        WS2812_ColorWheel(pos, brightness, &r, &g, &b);
        LED_Strip_SetPixelRGB(strip, i, r, g, b);
    }
}


