/*
 * OmniwheelCinematics.c
 *
 *  Created on: Dec 2, 2025
 *      Author: lorenzo
 */

#include "odometry.h"



// ---------- utils ----------
static inline void omni3_build_u(Omni3_t *o)
{
    for (int i = 0; i < 3; i++) {
    	const float a = o->phi[i] + (float)M_PI;
        o->ux[i] = -sinf(a);
        o->uy[i] =  cosf(a);
    }
}

static int omni3_inv3x3(const float A[3][3], float invA[3][3])
{
    const float a=A[0][0], b=A[0][1], c=A[0][2];
    const float d=A[1][0], e=A[1][1], f=A[1][2];
    const float g=A[2][0], h=A[2][1], i=A[2][2];

    const float A11 = (e*i - f*h);
    const float A12 = -(d*i - f*g);
    const float A13 = (d*h - e*g);

    const float A21 = -(b*i - c*h);
    const float A22 = (a*i - c*g);
    const float A23 = -(a*h - b*g);

    const float A31 = (b*f - c*e);
    const float A32 = -(a*f - c*d);
    const float A33 = (a*e - b*d);

    const float det = a*A11 + b*A12 + c*A13;
    if (fabsf(det) < 1e-9f) return 0;

    const float invdet = 1.0f / det;

    invA[0][0]=A11*invdet; invA[0][1]=A21*invdet; invA[0][2]=A31*invdet;
    invA[1][0]=A12*invdet; invA[1][1]=A22*invdet; invA[1][2]=A32*invdet;
    invA[2][0]=A13*invdet; invA[2][1]=A23*invdet; invA[2][2]=A33*invdet;

    return 1;
}

// Init standard (0°,120°,240°) + choix de la roue arrière
int omni3_init_default(Omni3_t *o, float r, float L, uint8_t rear_idx)
{
    o->r = r;
    o->L = L;
    o->rear_idx = (uint8_t)(rear_idx % 3);

    o->phi[0] = 0.0f;
    o->phi[1] = 2.0f * (float)M_PI / 3.0f;
    o->phi[2] = 4.0f * (float)M_PI / 3.0f;

    omni3_build_u(o);

    float A[3][3] = {
        { o->ux[0], o->uy[0], -o->L },
        { o->ux[1], o->uy[1], -o->L },
        { o->ux[2], o->uy[2], -o->L },
    };
    return omni3_inv3x3(A, o->Ainv);
}

// =========================================================
// 1) COMMANDE "AXE ROUE ARRIÈRE" : (v_rear, omega) -> w[3]
// v_rear en m/s dans la direction u_rear (axe moteur roue arrière)
// omega en rad/s ; w_out en rad/s
// =========================================================
void omni3_inverse_rear_axis(Omni3_t *o,float v_rear, float omega)
{
    uint8_t b = o->rear_idx;

    // Translation imposée uniquement le long de l'axe de la roue arrière :
    // v = v_rear * u_rear
    float vx = v_rear * o->ux[b];
    float vy = v_rear * o->uy[b];

    for (int i = 0; i < 3; i++) {
        // r*w_i = u_i·v + L*omega
        float v_i = (o->ux[i]*vx + o->uy[i]*vy) + (o->L * omega);
        o->w_out[i] = v_i / o->r;
    }
}

// =========================================================
// 2) RETOUR "AXE ROUE ARRIÈRE" : w[3] -> (v_rear, omega)
// On reconstruit d'abord (vx,vy,omega) via Ainv, puis on projette sur u_rear
// =========================================================
void omni3_direct_to_rear_axis(Omni3_t *o,float w[3])
{
    float b0 = o->r * w[0];
    float b1 = o->r * w[1];
    float b2 = o->r * w[2];

    float vx    = o->Ainv[0][0]*b0 + o->Ainv[0][1]*b1 + o->Ainv[0][2]*b2;
    float vy    = o->Ainv[1][0]*b0 + o->Ainv[1][1]*b1 + o->Ainv[1][2]*b2;
    float om    = o->Ainv[2][0]*b0 + o->Ainv[2][1]*b1 + o->Ainv[2][2]*b2;

    uint8_t b = o->rear_idx;
    o->v_rear_out = o->ux[b]*vx + o->uy[b]*vy;  // projection sur l'axe de la roue arrière
    o->w_rear_out  = om;
}


// =========================================================
// CINÉMATIQUE INVERSE COMPLÈTE : (vx, vy, omega) -> w[3]
// vx,vy en m/s (repère robot), omega en rad/s, w en rad/s
// =========================================================
void omni3_inverse(Omni3_t *o,float vx, float vy, float omega)
{
    for (int i = 0; i < 3; i++)
    {
        // r*w_i = u_i·v + L*omega
        float v_i = (o->ux[i]*vx + o->uy[i]*vy) + (o->L * omega);
        o->w_out[i] = v_i / o->r;
    }
}

// =========================================================
// CINÉMATIQUE DIRECTE COMPLÈTE : w[3] -> (vx, vy, omega)
// =========================================================
void omni3_direct(Omni3_t *o,float w[3],float *vx, float *vy, float *omega)
{
    float b0 = o->r * w[0];
    float b1 = o->r * w[1];
    float b2 = o->r * w[2];

    o->vx_out    = o->Ainv[0][0]*b0 + o->Ainv[0][1]*b1 + o->Ainv[0][2]*b2;
    o->vy_out    = o->Ainv[1][0]*b0 + o->Ainv[1][1]*b1 + o->Ainv[1][2]*b2;
    o->w_complete_out = o->Ainv[2][0]*b0 + o->Ainv[2][1]*b1 + o->Ainv[2][2]*b2;
}

