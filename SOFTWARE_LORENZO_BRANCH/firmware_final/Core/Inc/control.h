/*
 * control.h
 *
 *  Created on: Dec 3, 2025
 *      Author: lorenzo
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#include "main.h"

// ────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINES FOR MOVEMENT CONTROL ────────────────────────────────
// ────────────────────────────────────────────────────────────

#ifndef DELTA_TIME_MS
#define DELTA_TIME_MS 20.0f
#endif

// ────────────────────────────────────────────────────────────
// ──────────────────────────── Variables ─────────────────────
// ────────────────────────────────────────────────────────────

typedef enum{
	SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly,
	MouseGoesZoomZoomLikeSpeedyGonzalesArribaArriba,
	PimpMaxxing
}GameState;

typedef struct {
    uint8_t bearing;
    float distance_mm;
    float rotation_degs;
} RobotTargetMovement_t;

typedef struct{
	GameState gamestate;
	RobotTargetMovement_t target;
}RobotControl_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── PROTOTYPES ────────────────────────────────
// ─────────────────────────────e───────────────────────────────

void control_movement(Lidar_t *lidar, RobotControl_t *control);
#endif /* INC_CONTROL_H_ */
