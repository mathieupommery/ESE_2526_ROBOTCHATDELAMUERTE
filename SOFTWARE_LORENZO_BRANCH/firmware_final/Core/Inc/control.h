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

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include "main.h"

typedef struct Lidar Lidar_t;
typedef struct OmniOdometry OmniOdometry_t;
typedef struct TOF TOF_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── DEFINES FOR MOVEMENT CONTROL ────────────────────────────────
// ────────────────────────────────────────────────────────────

#ifndef DELTA_TIME_MS
#define DELTA_TIME_MS 20.0f
#endif

#define KEEP_ROBOT_AT_BAY 75

// ────────────────────────────────────────────────────────────
// ──────────────────────────── Variables ─────────────────────
// ────────────────────────────────────────────────────────────

typedef enum{
	SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly,
	MouseGoesZoomZoomLikeSpeedyGonzalesArribaArriba,
	PimpMaxxingRobotGoesBoomBamTchaTchaRealSmoothFlexOnTheHatingPlebsIGotALabubuKingAndLGTVQPlusLedsYouWishYOuHadButYouAreARobotFromTikTokMarketplace
}GameState;

typedef struct RobotTargetMovement{
    uint8_t bearing;
    float distance_mm;
    float rotation_degs;
} RobotTargetMovement_t;

typedef struct{
	GameState gamestate;
	uint8_t enemy;
	RobotTargetMovement_t target;
}RobotControl_t;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── PROTOTYPES ────────────────────────────────
// ─────────────────────────────e───────────────────────────────

void Omnicontrol_movement(Lidar_t *lidar, RobotControl_t *control);
void Staticcontrol_movement(Lidar_t *lidar, RobotControl_t *control, OmniOdometry_t *odom, TOF_t *tofs);
#endif /* INC_CONTROL_H_ */
