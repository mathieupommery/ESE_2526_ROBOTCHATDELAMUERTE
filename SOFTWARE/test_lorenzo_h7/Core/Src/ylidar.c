/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#include "ylidar.h"
#include "main.h"

uint16_t ylidar_read_index=0;
uint16_t ylidar_write_index=0;

uint8_t ylidar_circular_buffer[YLIDAR_CIRC_BUF_SIZE];

YLIDAR_STATE ydlidarstate=FSM_STATE_0;
CLUSTERIZATION_STATE clusterstate=SEARCHING;

uint8_t LSN=0;
uint8_t CT=0;

uint16_t ylidar_finalbuffer[359];

float FSA_float=0.0;
float LSA_float=0.0;

uint16_t FSA=0;
uint16_t LSA=0;

uint16_t CHECKSUM=0;
uint16_t XOR=0;

uint16_t ANGLEINIT=0;

typedef struct {
	uint16_t first_angle;   // angle de début
	uint16_t last_angle;    // angle de fin
	uint16_t distance;    // distance minimale (mm)
	float size;        // largeur physique estimée (mm)
} Object;

Object objects[10];

void ylidar_fsm(void)
{
	switch (ydlidarstate)
	{
	case FSM_STATE_0:

		if((ylidar_circular_buffer[ylidar_read_index]==0xAA)&& (ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]==0x55)){
			ydlidarstate=FSM_STATE_1;
			ylidar_read_index=(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;
		}
		else{
			ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
		}

		break;

	case FSM_STATE_1:

		if(!(ylidar_circular_buffer[ylidar_read_index]&0x01)){
			ydlidarstate=FSM_STATE_2;
			CT = ylidar_circular_buffer[ylidar_read_index];
			ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
		}
		else{
			ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			ydlidarstate=FSM_STATE_0;
		}

		break;

	case FSM_STATE_2:

		LSN=ylidar_circular_buffer[ylidar_read_index];
		uint16_t available=0;

		if (ylidar_write_index >= ylidar_read_index) {
			available = ylidar_write_index - ylidar_read_index;
		}
		else {
			available = (YLIDAR_CIRC_BUF_SIZE - ylidar_read_index) + ylidar_write_index;
		}

		uint16_t needed=LSN*2+6;

		if(needed<=available){
			ylidar_read_index=(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			ydlidarstate=FSM_STATE_3;
		}

		break;

	case FSM_STATE_3:

		FSA=(((ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[ylidar_read_index]));
		LSA=(((ylidar_circular_buffer[(ylidar_read_index+3)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE]));
		CHECKSUM= (uint16_t) (ylidar_circular_buffer[(ylidar_read_index+4)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index+5)%YLIDAR_CIRC_BUF_SIZE];
		ylidar_read_index=(ylidar_read_index+6)%YLIDAR_CIRC_BUF_SIZE;

		//        	XOR = 0xAA55 ^ ((CT << 8) | (LSN)) ^
		//        			(((FSA & 0x00FF )<< 8) | ((FSA & 0xFF00) >> 8)) ^
		//					(((LSA & 0x00FF) << 8) | ((LSA & 0xFF00) >> 8));

		//			for (int i = 0; i < LSN*2; i+=2){
		//			  XOR ^= (uint16_t)((ylidar_circular_buffer[(ylidar_read_index + i)] << 8)|
		//					  (ylidar_circular_buffer[(ylidar_read_index + i + 1) % YLIDAR_CIRC_BUF_SIZE]));
		//			}

		//			if (CHECKSUM == XOR){
		FSA_float = (float)((FSA /*& 0xFFFE*/) >> 1) / 64;
		LSA_float = (float)((LSA /*& 0xFFFE*/) >> 1) / 64;
		if (LSA_float < FSA_float) {
			LSA_float += 360.0;
		}
		ydlidarstate = FSM_STATE_4;
		//			}
		//			else{
		//				ylidar_read_index = (ylidar_read_index+(LSN*2)-1)%YLIDAR_CIRC_BUF_SIZE;
		//				ydlidarstate = FSM_STATE_0;
		//			}

	case FSM_STATE_4:

		uint16_t distance=(uint16_t)((ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index)%YLIDAR_CIRC_BUF_SIZE])/4;
		float AngleCorrect1 = ((distance) == 0) ? 0 : atanf(21.8 * ((155.3 - distance) / (155.3 * distance)));
		uint16_t Angle1 = ((uint16_t)floorf(FSA_float+AngleCorrect1))%360;
		ylidar_finalbuffer[Angle1]= (distance>=1000) ? 0 : distance ;
		distance=(uint16_t) ((ylidar_circular_buffer[(ylidar_read_index+LSN*2)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index+LSN*2-1)%YLIDAR_CIRC_BUF_SIZE])/4;
		float AngleCorrectLSN = ((distance) == 0) ? 0 : atanf(21.8 * ((155.3 - distance) / (155.3 * distance)));
		uint16_t Angle2 = ((uint16_t)floorf(LSA_float+AngleCorrectLSN))%360;
		ylidar_read_index=(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;

		for(int i=2;i<LSN-2;i++) {

			distance=(uint16_t) floorf(((ylidar_circular_buffer[(ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | ylidar_circular_buffer[(ylidar_read_index)%YLIDAR_CIRC_BUF_SIZE])/4);

			ylidar_finalbuffer[(Angle1+i)%360]= (distance>=LidarMaxDepth) ? 0 : distance;

			ylidar_read_index=(ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;

		}
		ydlidarstate=FSM_STATE_0;

		break;

	default:
		ydlidarstate=FSM_STATE_0;
		break;
	}
	//    ylidar_finalbuffer[720]='\n';
	//    ylidar_finalbuffer[721]='\r';
}

void trackObject(void) {
	uint16_t start = 0;
	uint16_t end = 0;
	uint8_t nb_objects = 0;
	uint16_t temp_cluster[180];
	uint16_t avg_distance;
	float size;
	uint16_t j = 0;
    memset(temp_cluster, 0, sizeof(temp_cluster));


	for (int i = 0; i < 358; i++) {

		switch (clusterstate){

		case SEARCHING:
			j = 0;
			if ( (ylidar_finalbuffer[i] - ylidar_finalbuffer[(i-1)%359]) >= -ClusterThreshold ) {
				start = i;
				temp_cluster[j] = ylidar_finalbuffer[i];
				j+=1;
				clusterstate=OBJECT;
			}
			break;

		case OBJECT:
			temp_cluster[j] = ylidar_finalbuffer[i];
			j+=1;

			if (j >= 180) {
			    clusterstate = SEARCHING;
			    j = 0;
			    memset(temp_cluster, 0, sizeof(temp_cluster));
			    break;
			}

			if ((ylidar_finalbuffer[i+1] - ylidar_finalbuffer[i]) >= ClusterThreshold) {
				end = i;
				uint16_t sum=0;
				uint16_t count=0;
				for (int i = 0; i < j; i++) {
					sum += temp_cluster[i];
					count++;
				}
				avg_distance = (uint16_t) floor(sum/count);
				size = sqrtf(powf(temp_cluster[0], 2) + powf(temp_cluster[j], 2) - 2 * temp_cluster[0] * temp_cluster[j] * cosf((end - start)*(M_PI / 180.0f)));
				clusterstate=FILTER_OBJECT;
			}
			break;

		case FILTER_OBJECT:
			if (nb_objects < 10 && size<=250 && size >=150) {

				objects[nb_objects].first_angle = start;
				objects[nb_objects].last_angle  = end;
				objects[nb_objects].distance    = avg_distance;
				objects[nb_objects].size		= size;
				nb_objects++;
				j=0;
				memset(temp_cluster, 0, sizeof(temp_cluster));
				clusterstate=SEARCHING;
			}
			clusterstate=SEARCHING;
			break;

		default:
			clusterstate=SEARCHING;
			break;
		}
	}
}
