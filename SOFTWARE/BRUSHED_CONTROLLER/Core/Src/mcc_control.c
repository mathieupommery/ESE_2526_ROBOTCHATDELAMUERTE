/*
 * mcc_control.c
 *
 *  Created on: Nov 22, 2025
 *      Author: mathi
 */


//tim4ch2 pwa mot2
//tim4ch1 pwmb mot2
//
//tim1ch2 hallb mot2
//tim1ch1 halla mot2
//
//tim3 ch1 pwmb mot3
//tim3 ch2 pwmb mot3
//
//tim8ch1 halla mot3
//tim8ch2 hallb mot3
//
//tim3ch3 pwmb mot1
//tim3ch4 pwma mot1
//
//tim2ch1 halla mot1
//tim2ch2 hallb mot1
//
//adc2in5 ntc

#include "mcc_control.h"
#include <math.h>

// PID communs (mêmes paramètres pour les 3 moteurs)
static PID_Params_t g_speedPidParams = {0};
static PID_Params_t g_posPidParams   = {0};

#define MOTOR_COUNT_MAX  3
extern Motor_t g_motors[3];
// ---------- Helpers internes ----------

static float clampf(float x, float minVal, float maxVal)
{
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

// Applique commande -1..+1 sur DRV8871 (double PWM)
static void Motor_ApplyCommand(Motor_t *m, float u)
{
    if (m->htimPwm == NULL) return;

    u = clampf(u, -1.0f, 1.0f);

    uint32_t arr = m->htimPwm->Instance->ARR;
    float duty = fabsf(u);
    uint32_t ccrDuty = (uint32_t)(duty * (float)arr);

    if (u > 0.0f)
    {
        // Sens A : chA PWM, chB 0
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, ccrDuty);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, 0);
    }
    else if (u < 0.0f)
    {
        // Sens B : chA 0, chB PWM
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, 0);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, ccrDuty);
    }
    else
    {
        // Stop
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, 0);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, 0);
    }
}

// PID simple (paramètres communs, état propre au moteur)
static float PID_Update(const PID_Params_t *p, PID_State_t *s,
                        float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Proportionnel
    float P = p->Kp * error;

    // Intégral
    s->integrator += error * dt;
    float I = p->Ki * s->integrator;

    // Dérivé
    float derivative = (error - s->prevError) / dt;
    float D = p->Kd * derivative;

    float out = P + I + D;
    out = clampf(out, -p->outMax, p->outMax);

    s->prevError = error;

    return out;
}

// Mise à jour mesures à partir encodeur
static void Motor_UpdateMeasurement(Motor_t *m, float dt)
{
    if (m->htimEnc == NULL) return;

    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(m->htimEnc);
    int32_t delta = (int32_t)(cnt - m->encPrev);
    m->encPrev = cnt;
    m->encRaw  = cnt;

    // vitesse en tour/s moteur
    float revs = (float)delta / m->ticksPerRev;
    m->speedRps = revs / dt;
    m->speedRpm = m->speedRps * 60.0f;

    // angle roue en deg : on convertit le compteur (modulo 1 tour moteur -> 360)
    float motorRevs = (float)cnt / m->ticksPerRev;
    float wheelRevs = motorRevs / m->gearRatio;
    m->angleDeg = wheelRevs * 360.0f;
}

// ---------- API impl ----------

void Motor_Init(Motor_t *m,
                TIM_HandleTypeDef *htimPwm, uint32_t chA, uint32_t chB,
                TIM_HandleTypeDef *htimEnc, uint32_t encA, uint32_t encB,
                float ticksPerRev, float gearRatio)
{
    if (m == NULL) return;

    m->htimPwm  = htimPwm;
    m->pwmChA   = chA;
    m->pwmChB   = chB;

    m->htimEnc  = htimEnc;
    m->encChA   = encA;
    m->encChB   = encB;

    m->ticksPerRev = ticksPerRev;
    m->gearRatio   = gearRatio;

    m->encRaw  = 0;
    m->encPrev = (int32_t)__HAL_TIM_GET_COUNTER(htimEnc);
    m->speedRps = 0.0f;
    m->speedRpm = 0.0f;
    m->angleDeg = 0.0f;

    m->mode            = MOTOR_MODE_DISABLED;
    m->targetSpeedRpm  = 0.0f;
    m->targetAngleDeg  = 0.0f;

    m->speedPid.integrator = 0.0f;
    m->speedPid.prevError  = 0.0f;
    m->posPid.integrator   = 0.0f;
    m->posPid.prevError    = 0.0f;

}

void Motor_SetCommonSpeedPid(float Kp, float Ki, float Kd, float outMax)
{
    g_speedPidParams.Kp     = Kp;
    g_speedPidParams.Ki     = Ki;
    g_speedPidParams.Kd     = Kd;
    g_speedPidParams.outMax = outMax;
}

void Motor_SetCommonPosPid(float Kp, float Ki, float Kd, float outMax)
{
    g_posPidParams.Kp     = Kp;
    g_posPidParams.Ki     = Ki;
    g_posPidParams.Kd     = Kd;
    g_posPidParams.outMax = outMax;
}

void Motor_SetTargetSpeed(Motor_t *m, float rpm)
{
    if (m == NULL) return;
    m->mode           = MOTOR_MODE_SPEED;
    m->targetSpeedRpm = rpm;
}

void Motor_GotoAngle(Motor_t *m, float angleDeg)
{
    if (m == NULL) return;
    m->mode          = MOTOR_MODE_POSITION;
    m->targetAngleDeg = angleDeg;
}

void Motor_Disable(Motor_t *m)
{
    if (m == NULL) return;
    m->mode = MOTOR_MODE_DISABLED;
    Motor_ApplyCommand(m, 0.0f);
    m->speedPid.integrator = 0.0f;
    m->posPid.integrator   = 0.0f;
}

// ---------- Tâche FreeRTOS ----------

void App_InitMotors(void)
{
    HAL_TIM_PWM_Start(MOT1_PWM_HTIM, MOT1_PWM_CH_A);
    HAL_TIM_PWM_Start(MOT1_PWM_HTIM, MOT1_PWM_CH_B);
    HAL_TIM_Encoder_Start(MOT1_ENC_HTIM, TIM_CHANNEL_ALL);

    // Moteur 2
    HAL_TIM_PWM_Start(MOT2_PWM_HTIM, MOT2_PWM_CH_A);
    HAL_TIM_PWM_Start(MOT2_PWM_HTIM, MOT2_PWM_CH_B);
    HAL_TIM_Encoder_Start(MOT2_ENC_HTIM, TIM_CHANNEL_ALL);

    // Moteur 3
    HAL_TIM_PWM_Start(MOT3_PWM_HTIM, MOT3_PWM_CH_A);
    HAL_TIM_PWM_Start(MOT3_PWM_HTIM, MOT3_PWM_CH_B);
    HAL_TIM_Encoder_Start(MOT3_ENC_HTIM, TIM_CHANNEL_ALL);

    Motor_Init(&g_motors[0],
               MOT1_PWM_HTIM, MOT1_PWM_CH_A, MOT1_PWM_CH_B,
               MOT1_ENC_HTIM, MOT1_ENC_CH_A, MOT1_ENC_CH_B,
               2048.0f,   // ticks par tour moteur
               1.0f       // ratio mécanique
    );

    Motor_Init(&g_motors[1],
               MOT2_PWM_HTIM, MOT2_PWM_CH_A, MOT2_PWM_CH_B,
               MOT2_ENC_HTIM, MOT2_ENC_CH_A, MOT2_ENC_CH_B,
               2048.0f,
               1.0f
    );

    Motor_Init(&g_motors[2],
               MOT3_PWM_HTIM, MOT3_PWM_CH_A, MOT3_PWM_CH_B,
               MOT3_ENC_HTIM, MOT3_ENC_CH_A, MOT3_ENC_CH_B,
               2048.0f,
               1.0f
    );

    // PID communs
    Motor_SetCommonSpeedPid(1.0f, 0.1f, 0.0f, 1.0f);
    Motor_SetCommonPosPid  (2.0f, 0.0f, 0.0f, 1.0f);
}


void MotorControlTask(float dt)
{
        // Boucle de contrôle
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++)
        {

            // Update mesures
            Motor_UpdateMeasurement(&g_motors[i], dt);

            float cmd = 0.0f;

            switch (g_motors[i].mode)
            {
            case MOTOR_MODE_DISABLED:
                cmd = 0.0f;
                break;

            case MOTOR_MODE_SPEED:
                cmd = PID_Update(&g_speedPidParams,&g_motors[i].speedPid,
                		g_motors[i].targetSpeedRpm, g_motors[i].speedRpm, dt);
                break;

            case MOTOR_MODE_POSITION:
            {
                // PID direct sur l’angle (simple, tu peux passer en cascade plus tard)
                float angleError = g_motors[i].targetAngleDeg - g_motors[i].angleDeg;

                // Option : normaliser l’erreur angle pour éviter de tourner dans le mauvais sens
                while (angleError > 180.0f)  angleError -= 360.0f;
                while (angleError < -180.0f) angleError += 360.0f;

                float angleSetpoint =g_motors[i].targetAngleDeg; // setpoint absolu
                float measurement   =g_motors[i].angleDeg;

                cmd = PID_Update(&g_posPidParams,&g_motors[i].posPid,
                                 angleSetpoint, measurement, dt);
                break;
            }

            default:
                cmd = 0.0f;
                break;
            }

            Motor_ApplyCommand(&g_motors[i], cmd);
        }

}




