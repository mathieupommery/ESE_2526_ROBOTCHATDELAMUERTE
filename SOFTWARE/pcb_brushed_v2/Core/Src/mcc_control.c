/*
 * mcc_control.c
 *
 *  Created on: Nov 22, 2025
 *      Author: mathi
 */


//adc2in5 ntc

#include "mcc_control.h"
#include <math.h>

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


void Motor_ApplyCommand(Motor_t *m)
{
    if (m->htimPwm == NULL) return;

    float u = clampf(m->cmd, -1.0f, 1.0f);
    if(fabs(u)<=0.1){
    	u=0.0;
    }

    uint32_t arr = m->htimPwm->Instance->ARR;
    float duty =1.0-fabsf(u);
    uint32_t ccrDuty = (uint32_t)(duty * (float)arr);

    if (u > 0.0f)
    {
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, ccrDuty);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, arr);

    }
    else if (u < 0.0f)
    {
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, arr);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, ccrDuty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChA, 0);
        __HAL_TIM_SET_COMPARE(m->htimPwm, m->pwmChB, 0);
    }
}

static float PID_Update(const PID_Params_t *p, PID_State_t *s,float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    float P = p->Kp * error;

    s->integrator += error * dt;

    if (p->Ki > 0.0f) {
        float iMax = p->outMax / p->Ki;
        if (s->integrator >  iMax) s->integrator =  iMax;
        if (s->integrator < -iMax) s->integrator = -iMax;
    }
    float I = p->Ki * s->integrator;

    float derivative = (error - s->prevError) / dt;
    float D = p->Kd * derivative;

    float out = P + I + D;

    out = clampf(out, -p->outMax, p->outMax);

    float u_dead = 0.25f;

    if((out< u_dead) && (out > -u_dead)){

    	out=0.0f;
    }
    s->prevError = error;

    return out;
}

// Mise à jour mesures à partir encodeur
//void Motor_UpdateMeasurement(Motor_t *m, float dt)
//{
//    if (m->htimEnc == NULL || dt <= 0.0f) return;
//
//    uint32_t cnt = __HAL_TIM_GET_COUNTER(m->htimEnc);
//    int32_t delta = (int16_t)(cnt - (uint16_t)m->encPrev);
//
//    m->encPrev   = cnt;
//    m->encRaw    = cnt;
//    float revs = (float)delta / m->ticksPerRev;
//
//    m->speedRps = revs / dt;
//    m->speedRpm = m->speedRps * 60.0f;
//
//    float angle_bis = m->angleDeg + revs * 360.0f;
//
//    angle_bis = fmodf(angle_bis, 360.0f);
//    if (angle_bis < 0.0f) angle_bis += 360.0f;
//
//    m->angleDeg = angle_bis;
//}

void Motor_UpdateMeasurement(Motor_t *m, float dt)
{
    int32_t pos_now = (int32_t)__HAL_TIM_GET_COUNTER(m->htimEnc);
    m->pos_buffer[m->idx] = pos_now;
    uint8_t idx_old = (m->idx + 1) % SPEED_WINDOW_MS;
    m->idx = (m->idx + 1) % SPEED_WINDOW_MS;
    if (m->filled < SPEED_WINDOW_MS) {
        m->filled++;
        m->speedRps = 0.0f;
        m->speedRpm = 0.0f;
        return;
    }
    int32_t pos_old = m->pos_buffer[idx_old];

    int16_t delta16 = (int16_t)((int16_t)pos_now - (int16_t)pos_old);
    int32_t delta_counts = (int32_t)delta16;
    float Te = dt * (float)SPEED_WINDOW_MS;

    m->speedRps = (float)delta_counts / (m->ticksPerRev * Te);
    m->speedRpm = m->speedRps * 60.0f;
}


// ---------- API impl ----------

void Motor_Init(Motor_t *m,
                TIM_HandleTypeDef *htimPwm, uint32_t chA, uint32_t chB,
                TIM_HandleTypeDef *htimEnc, uint32_t encA, uint32_t encB
                )
{
    if (m == NULL) return;

    m->htimPwm  = htimPwm;
    m->pwmChA   = chA;
    m->pwmChB   = chB;

    m->htimEnc  = htimEnc;
    m->encChA   = encA;
    m->encChB   = encB;

    m->ticksPerRev =TICK_PER_ROTOR_TURN*ENCODER_MULTIPLIER*GEAR_RATIO;

    m->encRaw  = 0;
    m->encPrev = (int32_t)__HAL_TIM_GET_COUNTER(htimEnc);
    m->speedRps = 0.0f;
    m->speedRpm = 0.0f;
    m->angleDeg = 0.0f;

    m->mode            = MOTOR_MODE_DISABLED;
    m->targetSpeedRpm  = 0.0f;
    m->targetAngleDeg  = 0.0f;
    m->angle_flag=0;
    m->cmd=0.0f;
    m->idx=0;

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
    m->cmd=0.0f;
    Motor_ApplyCommand(m);
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

    Motor_Init(&g_motors[0],MOT1_PWM_HTIM, MOT1_PWM_CH_A, MOT1_PWM_CH_B,MOT1_ENC_HTIM, MOT1_ENC_CH_A, MOT1_ENC_CH_B);
    Motor_Init(&g_motors[1],MOT2_PWM_HTIM, MOT2_PWM_CH_A, MOT2_PWM_CH_B,MOT2_ENC_HTIM, MOT2_ENC_CH_A, MOT2_ENC_CH_B);
    Motor_Init(&g_motors[2],MOT3_PWM_HTIM, MOT3_PWM_CH_A, MOT3_PWM_CH_B,MOT3_ENC_HTIM, MOT3_ENC_CH_A, MOT3_ENC_CH_B);
}


void MotorControlTask(float dt)
{
        // Boucle de contrôle
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++)
        {

            Motor_UpdateMeasurement(&g_motors[i], dt);

            switch (g_motors[i].mode)
            {
            case MOTOR_MODE_DISABLED:
            	g_motors[i].cmd = 0.0f;
                break;

            case MOTOR_MODE_SPEED:

            		g_motors[i].cmd = PID_Update(&g_speedPidParams,&g_motors[i].speedPid,g_motors[i].targetSpeedRpm, g_motors[i].speedRpm, dt);


                break;

            case MOTOR_MODE_POSITION:
            {
            	float angleError = g_motors[i].targetAngleDeg - g_motors[i].angleDeg;
            	if(angleError <= ANGLE_ERROR )
            	{
            		g_motors[i].cmd=0.0f;
            		g_motors[i].angle_flag=1;

            	}
            	else{
            		if(g_motors[i].angle_flag==1){
            			g_motors[i].angle_flag=0;
            			g_motors[i].angleDeg=0.0f;
            		}

            		g_motors[i].cmd = PID_Update(&g_posPidParams,&g_motors[i].posPid,g_motors[i].targetAngleDeg, g_motors[i].angleDeg, dt);
            	}



                break;
            }

            default:
            	g_motors[i].cmd = 0.0f;
                break;
            }

            Motor_ApplyCommand(&g_motors[i]);
        }

}




