/*
 * control.c
 *
 * Created on: Nov 30, 2025
 * Author: lorenzo
 */

#include "control.h"
#include "Lidar.h"
#include "odometry.h"
#include "vl53l0x.h"

void Staticcontrol_movement(Lidar_t *lidar, RobotControl_t *control, ODOM_struct *odom, TOF_t *tofs){
	Target_t *targetedobject = &lidar->cluster_struct.smooth_can;
	int16_t delta;
	switch(control->gamestate){
	case SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly:
		if (&lidar->cluster_struct.can_valid == 0) {
			control->target.speed_rpm = 0;
			control->target.rotation_degs = 0;
			control->enemy = (control->enemy + 1)%MAX_TARGETS;
			targetedobject = &lidar->cluster_struct.frame_b.targets[control->enemy];
			break;
		}

		if(tofs->tof[0].tof_int && tofs->tofs_cooldown){
			control->target.speed_rpm = 0;
//			control->enemy = (control->enemy + 1)%MAX_TARGETS;
//			targetedobject = &lidar->cluster_struct.frame_b.targets[control->enemy];
			tofs->tof[0].tof_int = 0;
			break;
		}
		delta = (int16_t)targetedobject->angle - 180;

		// Compute delta angle to aim for enemy
//		control->target.speed_rpm = 0;
		control->target.rotation_degs = delta;
		if (fabs(delta)<=5){
			control->target.rotation_degs = 0;
			control->target.speed_rpm = targetedobject->distance;
			break;
		}
		break;
	case MouseGoesZoomZoomLikeSpeedyGonzalesArribaArriba:
		if (&lidar->cluster_struct.can_valid == 0) {
			control->target.speed_rpm = 0;
			control->target.rotation_degs = 0;
			control->enemy = (control->enemy + 1)%MAX_TARGETS;
			targetedobject = &lidar->cluster_struct.frame_b.targets[control->enemy];
			return;
		}

		if(tofs->tof[0].tof_int && tofs->tofs_cooldown){
			control->target.speed_rpm = 0;
			control->target.rotation_degs = fmodf(odom->odom_w + targetedobject->angle, 360.0f);
			tofs->tof[0].tof_int = 0;
			break;
		}

		// Compute delta angle to aim for enemy
		delta = ((int16_t)targetedobject->angle >= 180) ? (int16_t)targetedobject->angle : 360 - (int16_t)targetedobject->angle;
		control->target.speed_rpm = 0;
		control->target.rotation_degs = delta;
		if (fabs(delta)<=5){
			control->target.rotation_degs = 0;
			control->target.speed_rpm = targetedobject->distance;
		}
		control->target.rotation_degs = 0;
		control->target.speed_rpm = fabs(targetedobject->distance - KEEP_ROBOT_AT_BAY);

	case PimpMaxxingRobotGoesBoomBamTchaTchaRealSmoothFlexOnTheHatingPlebsIGotALabubuKingAndLGTVQPlusLedsYouWishYOuHadButYouAreARobotFromTikTokMarketplace:

		if(tofs->tof[0].tof_int && tofs->tofs_cooldown){
			control->target.speed_rpm = 0;
			control->target.rotation_degs = 0;
			tofs->tof[0].tof_int = 0;
			control->gamestate = SideWinderCatModeOfUltimateDoomAndDestructionAKATheAnnihilatorOfAllHumanandNonHumanLifeInTheWholeWideWorldNayTheUniverseNayOfTHeMultiverseBecauseCatSoStrongBeautifulAndDeadly;
		}

		control->target.rotation_degs = 20;
		control->target.speed_rpm = 200;

	default:
		control->target.rotation_degs = 0;
		control->target.speed_rpm = 0;
	}


}
