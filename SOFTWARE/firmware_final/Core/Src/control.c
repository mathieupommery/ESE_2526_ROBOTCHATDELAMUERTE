/*
 * control.c
 *
 * Created on: Nov 30, 2025
 * Author: lorenzo
 */

#include "control.h"

//extern Robot_Cinematics_t cinematics;
//extern Cat_t cat;
//MotorSpeeds_t motor_speeds;
//
//// ────────────────────────────────────────────────────────────
//// ──────────────────────────── FUNCTIONS ───────────────────────────────
//// ────────────────────────────────────────────────────────────
//
//// Computes desired rotational speed using PID on orientation error,
//// then sets motor speeds = translation (from cinematics) + rotation component.
//// Returns the desired omega (deg/s). Call every DT_S (20 ms).
//float control_rotation_speed(void) {
//	static float integral   = 0.0f;
//	    static float prev_error = 0.0f;
//
//	    // 1. PID on orientation error (degrees)
//
//	    integral += cat.delta_orientation * DT_S;
//	    integral = MAX(MIN(integral,  INTEGRAL_LIMIT), -INTEGRAL_LIMIT);
//
//	    float derivative = (cat.delta_orientation - prev_error) / DT_S;
//	    prev_error = cat.delta_orientation;
//
//	    float omega_des_deg_s = PID_KP_ROT * cat.delta_orientation +
//	                            PID_KI_ROT * integral +
//	                            PID_KD_ROT * derivative;
//
//	    omega_des_deg_s = MAX(MIN(omega_des_deg_s, MAX_ROT_SPEED_DEG_S), -MAX_ROT_SPEED_DEG_S);
//
//	    // 2. Convert desired robot rotation → required wheel rotation contribution
//	    // Pure rotation on spot: every wheel must spin at:  ω_robot (rad/s) × R / r_wheel
//	    float omega_robot_rad_s = omega_des_deg_s * (M_PI / 180.0f);
//	    float rotation_rpm_contrib = (omega_robot_rad_s * ROBOT_RADIUS_MM) /
//	                                 (WHEEL_CIRCUMFERENCE_MM / 60.0f);   // → RPM
//
//	    // 3. Translation component: convert mm/s from cinematics → RPM
//	    float trans_rpm_0   = (float)cinematics.speed_bearing_0_mms   * 60.0f / WHEEL_CIRCUMFERENCE_MM;
//	    float trans_rpm_120 = (float)cinematics.speed_bearing_120_mms * 60.0f / WHEEL_CIRCUMFERENCE_MM;
//	    float trans_rpm_240 = (float)cinematics.speed_bearing_240_mms * 60.0f / WHEEL_CIRCUMFERENCE_MM;
//
//	    // 4. Final raw motor commands in RPM
//	    float raw1 = trans_rpm_0   + rotation_rpm_contrib;
//	    float raw2 = trans_rpm_120 + rotation_rpm_contrib;
//	    float raw3 = trans_rpm_240 + rotation_rpm_contrib;
//
//	    // 5. Saturation handling — scale down proportionally if any motor > 300 RPM
//	    float abs_max = MAX(MAX(fabsf(raw1), fabsf(raw2)), fabsf(raw3));
//	    if (abs_max > MAX_MOTOR_RPM)
//	    {
//	        float scale = MAX_MOTOR_RPM / abs_max;
//	        raw1 *= scale;
//	        raw2 *= scale;
//	        raw3 *= scale;
//	        integral *= scale;           // gentle anti-windup
//	    }
//
//	    // 6. Final clamp & assign
//	    motor_speeds.w1 = MAX(MIN(raw1,  MAX_MOTOR_RPM), -MAX_MOTOR_RPM);
//	    motor_speeds.w2 = MAX(MIN(raw2,  MAX_MOTOR_RPM), -MAX_MOTOR_RPM);
//	    motor_speeds.w3 = MAX(MIN(raw3,  MAX_MOTOR_RPM), -MAX_MOTOR_RPM);
//
//	    return omega_des_deg_s;   // for debugging / logging
//}
