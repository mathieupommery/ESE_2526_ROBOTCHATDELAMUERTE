#ifndef MA330_H_
#define MA330_H_

#include "main.h"

typedef struct {
    GPIO_TypeDef *MA330_cs_port;
    uint16_t MA330_cs_pin;
    uint8_t spi_rx_buffer[2];
    uint8_t spi_tx_buffer[2];
    uint8_t g_spi_done;
    uint8_t g_spi_error;
    
    float angle_filtered;
    float prev_angle_filtered;
    float prev_raw_angle;
    uint8_t spike_counter;
    
    float prev_angle;
    float filtered_rpm;
    float prev_rpm;
    float angle_accumulator;
    uint32_t time_accumulator;
    
	float output_prev_angle;
	float output_angle_ovf;
    float output_angle_filtered;
}MA330_t;
   

#define DEFAULT_FW 119  //time const 1024us
#define NORMAL_FW 102  //time const 512us
#define LOW_FW 85  //time const 256us

#define ANGLE_SCALE_FACTOR   0.02197265625f
#define INV_360   0.0027777778f
#define MAX_ANGLE_JUMP_DEG 20//a modifier si temps de mesure plus long
#define ANGLE_FILTER_ALPHA 0.2 //le prendre >0.2 pour reactivite
#define SPIKE_REJECT_COUNT 10

#define MICROS_TO_MINUTES 60000000
#define DEGREES_PER_REV 360
#define MAX_RPM_JUMP 50
#define RPM_FILTER_ALPHA 0.3

#define ACTUAL_ANGLE_FILTER_ALPHA 0.3




int MA330_Init(MA330_t *encd, GPIO_TypeDef *cs_port, uint16_t cs_pin,uint8_t FW);

int MA330_start(MA330_t *encd);

float MA330_get_degree(MA330_t *encd);
float MA330_get_rpm(MA330_t *encd, uint32_t dt_us);
float MA330_get_actual_degree(MA330_t *encd);



#endif
