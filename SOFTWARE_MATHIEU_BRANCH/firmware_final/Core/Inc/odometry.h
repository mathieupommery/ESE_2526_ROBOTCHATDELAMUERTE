/*
 * OmniwheelCinematics.h
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "main.h"

#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct
{
    float r;              // rayon roue (m)
    float L;              // distance centre -> roue (m)

    float phi[3];         // angles des roues (rad) : ex 0, 120°, 240°
    float ux[3], uy[3];   // directions de roulement unitaires u_i dans repère robot

    // Inverse de A pour la cinématique directe complète :
    // A*[vx vy omega]^T = [r*w0 r*w1 r*w2]^T
    float Ainv[3][3];

    uint8_t rear_idx;     // index de la roue arrière (0..2)

    float w_out[3];

    float v_rear_out;
    float w_rear_out;

    float vx_out;
    float vy_out;
    float w_complete_out;

} Omni3_t;

int omni3_init_default(Omni3_t *o, float r, float L, uint8_t rear_idx);
void omni3_inverse_rear_axis(Omni3_t *o,float v_rear, float omega);
void omni3_direct_to_rear_axis(Omni3_t *o,float w[3]);
void omni3_inverse(Omni3_t *o,float vx, float vy, float omega);
void omni3_direct(Omni3_t *o,float w[3],float *vx, float *vy, float *omega);

#endif /* INC_ODOMETRY_H_ */
