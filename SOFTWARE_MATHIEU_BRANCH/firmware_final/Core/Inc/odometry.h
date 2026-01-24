/*
 * OmniwheelCinematics.h
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "main.h"
#include "mcc_com_master.h"

#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RPM_2_RAD_S(rpm)   ((rpm) * (2.0f * M_PI / 60.0f))

#define WHEEL_W_MAX (400.0f * (2.0f * M_PI / 60.0f))
#define R_WHEEL 0.02935f
#define L_DIST 0.184f

typedef struct
{
    float target_theta;     // angle cible absolu (rad, wrapé)
    float Kp, Ki, Kd;       // gains PI/PD/PID sur l'angle
    float integral;         // terme intégral
    float prev_error;       // pour calculer le dérivé
    float w_max;            // limite |w_cmd| (rad/s)
    float angle_tol_rad;    // tolérance de fin (rad)
    uint8_t active;         // 1 = rotation en cours, 0 = terminée/inactive
} TurnController;

typedef struct
{

	TurnController turn_struct;
	float odom_w;
	float odom_x;
	float odom_y;


}ODOM_struct;


void limit_vw_by_wheel_speed(MOTOR_COM * comstruct);


#endif /* INC_ODOMETRY_H_ */
