/*
 * mcc_control.h
 *
 *  Created on: Nov 22, 2025
 *      Author: mathi
 */

#ifndef INC_MCC_CONTROL_H_
#define INC_MCC_CONTROL_H_

#include "stm32g4xx_hal.h"   // adapte le nom exact
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define MOTOR_CTRL_PERIOD_MS   5

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc2;


#define MOT1_PWM_HTIM        (&htim3)
#define MOT1_PWM_CH_A        TIM_CHANNEL_4   // PWMA
#define MOT1_PWM_CH_B        TIM_CHANNEL_3   // PWMB

#define MOT1_ENC_HTIM        (&htim2)
#define MOT1_ENC_CH_A        TIM_CHANNEL_1   // HALLA
#define MOT1_ENC_CH_B        TIM_CHANNEL_2   // HALLB

#define MOT2_PWM_HTIM        (&htim4)
#define MOT2_PWM_CH_A        TIM_CHANNEL_2   // PWMA
#define MOT2_PWM_CH_B        TIM_CHANNEL_1   // PWMB

#define MOT2_ENC_HTIM        (&htim1)
#define MOT2_ENC_CH_A        TIM_CHANNEL_1   // HALLA
#define MOT2_ENC_CH_B        TIM_CHANNEL_2   // HALLB


#define MOT3_PWM_HTIM        (&htim3)
#define MOT3_PWM_CH_A        TIM_CHANNEL_1   // PWMA (à confirmer)
#define MOT3_PWM_CH_B        TIM_CHANNEL_2   // PWMB (à confirmer)

#define MOT3_ENC_HTIM        (&htim8)
#define MOT3_ENC_CH_A        TIM_CHANNEL_1   // HALLA
#define MOT3_ENC_CH_B        TIM_CHANNEL_2   // HALLB

#define NTC_ADC              (&hadc2)
#define NTC_ADC_CHANNEL      ADC_CHANNEL_5   // correspond à ADC2_IN5




typedef enum
{
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION
} MotorMode_t;

// Paramètres PID communs à tous les moteurs
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float outMax;      // saturation |u| <= outMax (<=1.0)
} PID_Params_t;

// État interne d’un PID (propre à chaque moteur)
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

    float   ticksPerRev;   // ticks par tour moteur
    float   gearRatio;     // rapport de réduction (tour roue / tour moteur)

    // --- Mesures ---
    int32_t encRaw;        // compteur brut
    int32_t encPrev;       // pour calcul delta
    float   speedRps;      // vitesse en tour/s moteur
    float   speedRpm;      // vitesse en tour/min moteur
    float   angleDeg;      // angle roue en degrés

    // --- Consignes ---
    MotorMode_t mode;
    float targetSpeedRpm;      // consigne de vitesse
    float targetAngleDeg;      // consigne d’angle (absolu roue)

    // --- PID states par moteur (mais params communs) ---
    PID_State_t speedPid;
    PID_State_t posPid;

} Motor_t;

// ---------- API ----------
void App_InitMotors(void);
// Appelé pour chaque moteur
void Motor_Init(Motor_t *m,
                TIM_HandleTypeDef *htimPwm, uint32_t chA, uint32_t chB,
                TIM_HandleTypeDef *htimEnc, uint32_t encA, uint32_t encB,
                float ticksPerRev, float gearRatio);

// PID communs à tous les moteurs (même params, états séparés)
void Motor_SetCommonSpeedPid(float Kp, float Ki, float Kd, float outMax);
void Motor_SetCommonPosPid  (float Kp, float Ki, float Kd, float outMax);

// Commandes
void Motor_SetTargetSpeed(Motor_t *m, float rpm);
void Motor_GotoAngle     (Motor_t *m, float angleDeg);
void Motor_Disable       (Motor_t *m);

// Tâche de contrôle (à créer dans main)
void MotorControlTask(float dt);


#endif /* INC_MCC_CONTROL_H_ */
