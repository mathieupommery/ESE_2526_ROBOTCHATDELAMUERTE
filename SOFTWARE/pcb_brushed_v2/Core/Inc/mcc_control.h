/*
 * mcc_control.h
 *
 *  Created on: Nov 22, 2025
 *      Author: mathi
 */

#ifndef INC_MCC_CONTROL_H_
#define INC_MCC_CONTROL_H_

#include "stm32g4xx_hal.h"   // adapte le nom exact
#include <stdint.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;


#define MOT1_PWM_HTIM        (&htim3)
#define MOT1_PWM_CH_A        TIM_CHANNEL_4   // PWMA
#define MOT1_PWM_CH_B        TIM_CHANNEL_3   // PWMB

#define MOT1_ENC_HTIM        (&htim2)
#define MOT1_ENC_CH_A        TIM_CHANNEL_2   // HALLA
#define MOT1_ENC_CH_B        TIM_CHANNEL_1   // HALLB

#define MOT2_PWM_HTIM        (&htim4)
#define MOT2_PWM_CH_A        TIM_CHANNEL_2   // PWMA
#define MOT2_PWM_CH_B        TIM_CHANNEL_1   // PWMB

#define MOT2_ENC_HTIM        (&htim1)
#define MOT2_ENC_CH_A        TIM_CHANNEL_2   // HALLA
#define MOT2_ENC_CH_B        TIM_CHANNEL_1   // HALLB


#define MOT3_PWM_HTIM        (&htim3)
#define MOT3_PWM_CH_A        TIM_CHANNEL_1   // PWMA (à confirmer)
#define MOT3_PWM_CH_B        TIM_CHANNEL_2   // PWMB (à confirmer)

#define MOT3_ENC_HTIM        (&htim8)
#define MOT3_ENC_CH_A        TIM_CHANNEL_2   // HALLA
#define MOT3_ENC_CH_B        TIM_CHANNEL_1   // HALLB

#define NTC_ADC              (&hadc2)
#define NTC_ADC_CHANNEL      ADC_CHANNEL_5   // correspond à ADC2_IN5


#define TICK_PER_ROTOR_TURN  12.0f
#define ENCODER_MULTIPLIER  4.0f
#define GEAR_RATIO 34.014f

#define SPEED_WINDOW_MS 20

#define RAMP_MAX  20.0f

#define ANGLE_ERROR  0.5f


typedef enum
{
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION
} MotorMode_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float outMax;
} PID_Params_t;

typedef struct
{
    float integrator;
    float prevError;
} PID_State_t;

typedef struct
{
    // --- Hardware ---
    TIM_HandleTypeDef *htimPwm;
    uint32_t pwmChA;
    uint32_t pwmChB;

    TIM_HandleTypeDef *htimEnc;
    uint32_t encChA;
    uint32_t encChB;

    float   ticksPerRev;

    // --- Mesures ---
    int32_t encRaw;
    int32_t encPrev;
    float   speedRps;
    float   speedRpm;
    float   angleDeg;
    int32_t pos_buffer[SPEED_WINDOW_MS];
    uint8_t idx;
    uint8_t filled;
    // --- Consignes ---
    MotorMode_t mode;
    float targetSpeedRpm;
    float targetAngleDeg;
    float rampdiffmax;
    float cmd;

    uint8_t angle_flag;

    // --- PID states par moteur (mais params communs) ---
    PID_State_t speedPid;
    PID_State_t posPid;

} Motor_t;


void App_InitMotors(void);
void Motor_Init(Motor_t *m,TIM_HandleTypeDef *htimPwm, uint32_t chA, uint32_t chB,TIM_HandleTypeDef *htimEnc, uint32_t encA, uint32_t encB);
void Motor_UpdateMeasurement(Motor_t *m, float dt);
void Motor_ApplyCommand(Motor_t *m);
void Motor_SetCommonSpeedPid(float Kp, float Ki, float Kd, float outMax);
void Motor_SetCommonPosPid  (float Kp, float Ki, float Kd, float outMax);
void Motor_SetTargetSpeed(Motor_t *m, float rpm);
void Motor_GotoAngle     (Motor_t *m, float angleDeg);
void Motor_Disable       (Motor_t *m);
void MotorControlTask(float dt);


#endif /* INC_MCC_CONTROL_H_ */
