/*
 * led.h
 *
 *  Created on: Mar 14, 2025
 *      Author: mathi
 */

#ifndef INC_LED_H_
#define INC_LED_H_





#include "main.h"
#include "tim.h"


#ifndef WS2812_HI_VAL
#define WS2812_HI_VAL   (64U)    // rapport cyclique pour un '1'
#endif

#ifndef WS2812_LOW_VAL
#define WS2812_LOW_VAL  (32U)    // rapport cyclique pour un '0'
#endif

#ifndef WS2812_RST_PERIOD
#define WS2812_RST_PERIOD  (50U) // nombre de périodes pour le reset (t > 50 µs)
#endif


#define WS2812_GRB_BUF_LEN(LED_COUNT)   ((uint16_t)((LED_COUNT) * 3U))
#define WS2812_DMA_BUF_LEN(LED_COUNT)   ((uint16_t)(24U * (LED_COUNT) + WS2812_RST_PERIOD + 5U))



#define TOM_JERRY_LED_COUNT (2U)
#define EXT_LED_COUNT (54U)

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t           tim_channel;

    uint16_t           led_count;

    uint8_t           *grb_buf;
    uint32_t          *dma_buf;
    uint16_t           dma_buf_len;
    uint16_t           dma_len_used;

    volatile uint8_t   dma_busy;
} led_strip_t;

static inline uint16_t WS2812_MinDmaBufferLen(uint16_t led_count)
{
    return (uint16_t)(24U * led_count + WS2812_RST_PERIOD + 5U);
}

HAL_StatusTypeDef LED_Strip_Init(led_strip_t *strip,TIM_HandleTypeDef *htim,uint32_t tim_channel,uint8_t *grb_buf,uint16_t led_count,uint32_t *dma_buf,uint16_t dma_buf_len);

/* Met un pixel (index) à une couleur R,G,B */
void LED_Strip_SetPixelRGB(led_strip_t *strip,uint16_t index,uint8_t r, uint8_t g, uint8_t b);
/* Met à jour plusieurs LED à partir d'un tableau RGB [R0,G0,B0,R1,G1,B1,...] */
void LED_Strip_SetRGBArray(led_strip_t *strip,const uint8_t *rgb,uint16_t led_count);
HAL_StatusTypeDef LED_Strip_Refresh(led_strip_t *strip);
void LED_Strip_DMA_Callback(led_strip_t *strip);

void LED_Strip_FillRainbow(led_strip_t *strip,uint16_t offset,uint8_t brightness);


#endif /* INC_LED_H_ */




