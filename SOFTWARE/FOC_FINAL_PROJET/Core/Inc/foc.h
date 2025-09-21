/*
 * foc.h
 *
 *  Created on: Sep 21, 2025
 *      Author: mathi
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_


/* Horloge du timer (après x2 APB si applicable) */
#ifndef FOC_TIM_CLK_HZ
#define FOC_TIM_CLK_HZ         128000000UL   /* ex. 170 MHz */
#endif

/* Division "clock division" CKD (n’affecte PAS la fréquence PWM, sert aux filtres & deadtime) */
#ifndef TIM_CLOCK_DIVIDER
#define TIM_CLOCK_DIVIDER      1UL           /* 1, 2 ou 4 */
#endif

/* Prescaler compteur (PSC) -> tick compteur = FOC_TIM_CLK_HZ/(PSC+1) */
#ifndef FOC_PSC
#define FOC_PSC                0UL
#endif

/* Mode compteur:
   0=edge-aligned ; 1/2/3=center-aligned (double la période effective) */
#ifndef FOC_CMS
#define FOC_CMS                1UL           /* center-aligned mode 1 par défaut */
#endif

/* Fréquence PWM demandée (fondamentale) */
#ifndef FOC_PWM_FREQ_HZ
#define FOC_PWM_FREQ_HZ        20000UL       /* 20 kHz */
#endif

/* ======= DÉRIVÉS ======= */

/* Horloge des “ticks” du compteur */
#define FOC_CNT_TICK_HZ        ( (FOC_TIM_CLK_HZ) / ((FOC_PSC) + 1UL) )

/* Facteur période selon le mode (centré = 2) */
#define FOC_CMS_FACTOR         ( (FOC_CMS)==0UL ? 1UL : 2UL )

/* Nombre de ticks par période (ARR+1), arrondi au plus proche */
#define PWM_PERIOD_TICKS     (FOC_CNT_TICK_HZ + ((FOC_PWM_FREQ_HZ*FOC_CMS_FACTOR)/2ULL))/(FOC_PWM_FREQ_HZ * FOC_CMS_FACTOR)

#ifndef FOC_UPDATE_EVENT_DIV
#define FOC_UPDATE_EVENT_DIV       1UL       /* 20 kHz */
#endif





/* Sanity checks */
#if (PWM_PERIOD_TICKS < 2)
# error "FOC_PWM_FREQ_HZ trop élevée pour FOC_TIM_CLK_HZ/PSC/CMS"
#endif

#define HTMIN 1UL

//decalage pour le debut d'interruption de lecture adc et effet hall


#define DEAD_TIME_COUNTS        20000UL




//DTG[7:0]: Dead-time generator setup
//This bit-field defines the duration of the dead-time inserted between the complementary
//outputs. DT correspond to this duration.
//DTG[7:5] = 0xx => DT = DTG[7:0] x tDTG with tDTG = tDTS.
//DTG[7:5] = 10x => DT = (64 + DTG[5:0]) x tDTG with tDTG =2xtDTS.
//DTG[7:5] = 110 => DT = (32 + DTG[4:0]) x tDTG with tDTG =8xtDTS.
//DTG[7:5] = 111 => DT = (32 + DTG[4:0]) x tDTG with tDTG = 16 x tDTS.
//Example if tDTS = 125 ns (8 MHz), dead-time possible values are:
//0 to 15875 ns by 125 ns steps,
//16 μs to 31750 ns by 250 ns steps,
//32 μs to 63 μs by 1 μs steps,
//64 μs to 126 μs by 2 μs steps
//Note: This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been programmed

#include <stdint.h>
#include "FOC_math.h"
#include "pid_utils.h"

#define ERROR_LUT_SIZE (1024)

#define MAG_CAL_RES (1024*2)
#define MAG_CAL_STEP ((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

extern _Bool foc_ready;

typedef enum {
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE,
	AUDIO_MODE,
	TEST_MODE,
}motor_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;


typedef struct {
	uint8_t pole_pairs;
	float kv;
	float La, Lb, Lc;
	float Ra, Rb, Rc;
	float max_current;

	float m_angle_rad; // mechanical angle
	float e_angle_rad; // electrical angle
	float e_angle_rad_comp; // electrical angle
	float m_angle_offset;

	float vd, vq;
	float id, iq;
	float id_filtered, iq_filtered;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float i_bus;

	float actual_rpm;
	float actual_angle;

	float id_ref, iq_ref;
	float rpm_ref;

    uint8_t loop_count;

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;
	uint32_t pwm_res;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	motor_mode_t control_mode;

	float gear_ratio;
	dir_mode_t sensor_dir;
}foc_t;

void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c,
		uint32_t pwm_res);
void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);
void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);
void foc_gear_reducer_init(foc_t *hfoc, float ratio);
void foc_set_limit_current(foc_t *hfoc, float i_limit);
void foc_current_control_update(foc_t *hfoc);
void foc_speed_control_update(foc_t *hfoc, float rpm_reference);
void foc_position_control_update(foc_t *hfoc, float deg_reference);
void foc_calc_electric_angle(foc_t *hfoc, float m_rad);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);


#endif /* INC_FOC_H_ */
