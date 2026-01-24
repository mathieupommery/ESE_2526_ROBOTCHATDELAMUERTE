/*
 * OmniwheelCinematics.c
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#include "odometry.h"



static inline float wrap_angle_rad(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static inline float deg2rad(float deg)
{
    return deg * (M_PI / 180.0f);
}


void limit_vw_by_wheel_speed(MOTOR_COM * comstruct)
{
    float vL = comstruct->v - 0.5f * L_DIST * comstruct->w;
    float vR = comstruct->v + 0.5f * L_DIST * comstruct->w;

    float wL = vL / R_WHEEL;
    float wR = vR / R_WHEEL;

    float abs_wL = fabsf(wL);
    float abs_wR = fabsf(wR);
    float maxWheel = (abs_wL > abs_wR) ? abs_wL : abs_wR;

    if (maxWheel > WHEEL_W_MAX && maxWheel > 1e-6f)
    {
        float scale = WHEEL_W_MAX / maxWheel;
        comstruct->v = comstruct->v * scale;
        comstruct->w = comstruct->w * scale;
    }
}


static inline void turn_start(TurnController *tc,float theta_current_rad,float delta_deg)
{
    float delta_rad = deg2rad(delta_deg);
    float target = theta_current_rad + delta_rad;
    target = wrap_angle_rad(target);

    tc->target_theta = target;
    tc->integral     = 0.0f;
    tc->prev_error   = 0.0f;
    tc->active       = 1;
}

static float turn_step(TurnController *tc,float theta_current_rad,float dt,int *done)
{
    if (!tc->active)
    {
        *done = 1;
        return 0.0f;
    }

    float err = wrap_angle_rad(tc->target_theta - theta_current_rad);

    if (fabsf(err) < tc->angle_tol_rad)
    {
        tc->active = 0;
        *done = 1;
        return 0.0f;
    }

    *done = 0;

    tc->integral += err * dt;
    if (tc->Ki > 0.0f)
    {
        float I_max = tc->w_max / tc->Ki;
        if (tc->integral >  I_max) tc->integral =  I_max;
        if (tc->integral < -I_max) tc->integral = -I_max;
    }

    float deriv = (err - tc->prev_error) / dt;
    tc->prev_error = err;

    float w_cmd = tc->Kp * err + tc->Ki * tc->integral + tc->Kd * deriv;

    if (w_cmd >  tc->w_max) w_cmd =  tc->w_max;
    if (w_cmd < -tc->w_max) w_cmd = -tc->w_max;

    return w_cmd;
}


void robot_control_step(ODOM_struct *odom_struct,MOTOR_COM * comstruct)
{

    float dt = comstruct->rx_struct.f.actualtime * 1e-6f;  // µs -> s
    float w_meas = comstruct->rx_struct.f.actualw;
    float v_meas = comstruct->rx_struct.f.actualv;


    odom_struct->odom_w += w_meas * dt;
    odom_struct->odom_w = wrap_angle_rad(odom_struct->odom_w);

    odom_struct->odom_x += v_meas * cosf(odom_struct->odom_w) * dt;
    odom_struct->odom_y += v_meas * sinf(odom_struct->odom_w) * dt;

    float v = 0.0f;
    float w = 0.0f;

    int turn_done = 0;

    if (odom_struct->turn_struct.active)
    {
        // Rotation prioritaire (robot tourne sur place → v=0)
        w = turn_step(&odom_struct->turn_struct, odom_struct->odom_w, dt, &turn_done);
        v = 0.0f;
    }
    else
    {



        v = 0.0f;
        w = 0.0f;
    }

    limit_vw_by_wheel_speed(comstruct);

    comstruct->tx_struct.f.targetw=w;
    comstruct->tx_struct.f.targetv=v;

//    *v_cmd = v;
//    *w_cmd = w;
}

