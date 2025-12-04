/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Lorenzo
 */

#include "LidarTracking.h"

// ────────────────────────────────────────────────────────────
// ──────────────────────────── Variables ────────────────────────────────
// ────────────────────────────────────────────────────────────

LiDARParsing_t laserscan;
TempClusters_t tempclusters;
LidarClusterList_t lidar;

// ────────────────────────────────────────────────────────────
// ──────────────────────────── INIT ────────────────────────────────
// ────────────────────────────────────────────────────────────

void LidarTracking_Init(void){

	// LiDAR laserscan instance INIT //
	laserscan.ylidar_read_index=0;
	laserscan.ylidar_write_index=0;
	memset(laserscan.ylidar_circular_buffer, 0, sizeof(laserscan.ylidar_circular_buffer));
	laserscan.ydlidarstate=FSM_STATE_0;
	laserscan.LSN=0;
	laserscan.CT=0;
	memset(laserscan.ylidar_finalbuffer, 0, sizeof(laserscan.ylidar_finalbuffer));
	laserscan.FSA_float=0.0;
	laserscan.LSA_float=0.0;
	laserscan.FSA=0;
	laserscan.LSA=0;
	laserscan.CHECKSUM=0;
	laserscan.XOR=0;
	laserscan.ANGLEINIT=0;

	// Clusterization tempclusters instance INIT //
	memset(tempclusters.temp_clusters, 0, sizeof(tempclusters.temp_clusters));
	memset(tempclusters.cluster_buf, 0, sizeof(tempclusters.cluster_buf));
	tempclusters.temp_cluster_cnt = 0;

	memset(&lidar, 0, sizeof(lidar));
	lidar.scan_id = 0;
	lidar.new_data = false;
}


//-----Ylidar Parsing State Machine-----//

void ylidar_fsm(void)
{
	switch (laserscan.ydlidarstate)
	{
	case FSM_STATE_0:

		if((laserscan.ylidar_circular_buffer[laserscan.ylidar_read_index]==0xAA)&& (laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]==0x55)){
			laserscan.ydlidarstate=FSM_STATE_1;
			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;
		}
		else{
			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
		}

		break;

	case FSM_STATE_1:

		if(!(laserscan.ylidar_circular_buffer[laserscan.ylidar_read_index]&0x01)){
			laserscan.ydlidarstate=FSM_STATE_2;
			laserscan.CT = laserscan.ylidar_circular_buffer[laserscan.ylidar_read_index];
			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
		}
		else{
			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			laserscan.ydlidarstate=FSM_STATE_0;
		}

		break;

	case FSM_STATE_2:

		laserscan.LSN=laserscan.ylidar_circular_buffer[laserscan.ylidar_read_index];
		uint16_t available=0;

		if (laserscan.ylidar_write_index >= laserscan.ylidar_read_index) {
			available = laserscan.ylidar_write_index - laserscan.ylidar_read_index;
		}
		else {
			available = (YLIDAR_CIRC_BUF_SIZE - laserscan.ylidar_read_index) + laserscan.ylidar_write_index;
		}

		uint16_t needed=laserscan.LSN*2+6;

		if(needed<=available){
			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			laserscan.ydlidarstate=FSM_STATE_3;
		}

		break;

	case FSM_STATE_3:

		laserscan.FSA=(((laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[laserscan.ylidar_read_index]));
		laserscan.LSA=(((laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+3)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE]));
		laserscan.CHECKSUM= (uint16_t) (laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+4)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+5)%YLIDAR_CIRC_BUF_SIZE];
		laserscan.ylidar_read_index=(laserscan.ylidar_read_index+6)%YLIDAR_CIRC_BUF_SIZE;

		//        	XOR = 0xAA55 ^ ((CT << 8) | (LSN)) ^
		//        			(((FSA & 0x00FF )<< 8) | ((FSA & 0xFF00) >> 8)) ^
		//					(((LSA & 0x00FF) << 8) | ((LSA & 0xFF00) >> 8));

		//			for (int i = 0; i < LSN*2; i+=2){
		//			  XOR ^= (uint16_t)((ylidar_circular_buffer[(ylidar_read_index + i)] << 8)|
		//					  (ylidar_circular_buffer[(ylidar_read_index + i + 1) % YLIDAR_CIRC_BUF_SIZE]));
		//			}

		//			if (CHECKSUM == XOR){
		laserscan.FSA_float = (float)((laserscan.FSA /*& 0xFFFE*/) >> 1) / 64;
		laserscan.LSA_float = (float)((laserscan.LSA /*& 0xFFFE*/) >> 1) / 64;
		if (laserscan.LSA_float < laserscan.FSA_float) {
			laserscan.LSA_float += 360.0;
		}
		laserscan.ydlidarstate = FSM_STATE_4;
		//			}
		//			else{
		//				ylidar_read_index = (ylidar_read_index+(LSN*2)-1)%YLIDAR_CIRC_BUF_SIZE;
		//				ydlidarstate = FSM_STATE_0;
		//			}

	case FSM_STATE_4:

		uint16_t distance=(uint16_t)((laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index)%YLIDAR_CIRC_BUF_SIZE])/4;
		float AngleCorrect1 = ((distance) == 0) ? 0 : atanf(21.8 * ((155.3 - distance) / (155.3 * distance)));
		uint16_t Angle1 = ((uint16_t)floorf(laserscan.FSA_float+AngleCorrect1))%360;
		laserscan.ylidar_finalbuffer[Angle1]= (distance>=1000) ? 0 : distance ;
		distance=(uint16_t) ((laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+laserscan.LSN*2)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+laserscan.LSN*2-1)%YLIDAR_CIRC_BUF_SIZE])/4;
		//		float AngleCorrectLSN = ((distance) == 0) ? 0 : atanf(21.8 * ((155.3 - distance) / (155.3 * distance)));
		//		uint16_t Angle2 = ((uint16_t)floorf(LSA_float+AngleCorrectLSN))%360;
		laserscan.ylidar_read_index=(laserscan.ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;

		for(int i=2;i<laserscan.LSN-2;i++) {

			distance=(uint16_t) floorf(((laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]<<8) | laserscan.ylidar_circular_buffer[(laserscan.ylidar_read_index)%YLIDAR_CIRC_BUF_SIZE])/4);

			laserscan.ylidar_finalbuffer[(Angle1+i)%360]= (distance>=LidarMaxDepth) ? 0 : distance;

			laserscan.ylidar_read_index=(laserscan.ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;

		}
		laserscan.ydlidarstate=FSM_STATE_0;

		break;

	default:
		laserscan.ydlidarstate=FSM_STATE_0;
		break;
	}
	//    ylidar_finalbuffer[720]='\n';
	//    ylidar_finalbuffer[721]='\r';
}

//-----CLUSTER Detection-----//

// Fonction de calcul de taille ultra-précise et stable
static inline float calc_size_mm(uint16_t avg_dist_mm, float angle_deg)
{
	if (avg_dist_mm < 100 || angle_deg < 3.0f || angle_deg > 150.0f) {
		return 0.0f;  // filtre anti-bruit
	}
	float rad = angle_deg * (float)M_PI / 180.0f;
	return 2.0f * (float)avg_dist_mm * tanf(rad/2.0f);
}


/*------------------------------------------------------------------*/
void LidarClusterTracker_ProcessScan(void)
{
	uint32_t start = DWT->CYCCNT;
	lidar.count = 0;
	lidar.new_data = false;
	lidar.scan_id++;

	tempclusters.temp_cluster_cnt = 0;

	uint16_t cluster_start = 0;
	uint16_t cluster_len = 0;
	uint32_t cluster_sum = 0;
	uint16_t first_point = 0;
	uint16_t last_point = 0;
	uint16_t holes_used = 0;  // Track number of holes in current cluster

	enum { SEARCH, IN_CLUSTER } state = SEARCH;

	for (uint16_t i = 0; i <= LIDAR_POINTS; i++)
	{
		uint16_t dist = (i < LIDAR_POINTS) ? laserscan.ylidar_finalbuffer[i] : 0xFFFF;

		switch (state)
		{
		case SEARCH:
			if (dist >= MIN_VALID_DISTANCE_MM && dist <= MAX_VALID_DISTANCE_MM)
			{
				cluster_start = i;
				cluster_len = 1;
				cluster_sum = dist;
				first_point = dist;
				last_point = dist;
				holes_used = 0;
				state = IN_CLUSTER;
			}
			break;

		case IN_CLUSTER:
		{
			bool end_cluster = false;

			if (dist < MIN_VALID_DISTANCE_MM || dist > MAX_VALID_DISTANCE_MM)
			{
				// --- Handle holes ---
				if (holes_used >= MAX_HOLE_COUNT)
				{
					end_cluster = true;
					break;
				}

				uint16_t hole_points = 1;
				uint16_t k = (i + 1) % LIDAR_POINTS;

				while (hole_points <= MAX_HOLE_POINTS &&
						(laserscan.ylidar_finalbuffer[k] < MIN_VALID_DISTANCE_MM ||
								laserscan.ylidar_finalbuffer[k] > MAX_VALID_DISTANCE_MM))
				{
					hole_points++;
					k = (k + 1) % LIDAR_POINTS;
				}

				if (hole_points <= MAX_HOLE_POINTS)
				{
					cluster_len += hole_points;
					cluster_sum += (uint32_t)last_point * hole_points;
					i += hole_points - 1;
					holes_used++;
					continue;
				}
				else
				{
					end_cluster = true;
				}
			}
			else
			{
				// --- Check cluster continuity ---
				uint16_t prev_idx = (i + LIDAR_POINTS - 1) % LIDAR_POINTS;
				if (abs((int)dist - (int)laserscan.ylidar_finalbuffer[prev_idx]) > CLUSTER_THRESHOLD_MM)
					end_cluster = true;
				else
				{
					cluster_len++;
					cluster_sum += dist;
					last_point = dist;
				}
			}

			if (end_cluster || i == LIDAR_POINTS)
			{
				if (cluster_len >= MIN_CLUSTER_POINTS)
				{
					uint16_t avg = cluster_sum / cluster_len;
					float angle_deg = (float)(i - cluster_start);
					float size = calc_size_mm(avg, angle_deg);

					if (size >= MIN_CLUSTER_SIZE_MM && size <= MAX_CLUSTER_SIZE_MM &&
							tempclusters.temp_cluster_cnt < MAX_TEMP_CLUSTERS)
					{
						tempclusters.temp_clusters[tempclusters.temp_cluster_cnt++] = (TempCluster_t){
							.start_angle = cluster_start,
									.end_angle   = i - 1,
									.point_count = cluster_len,
									.avg_dist    = avg,
									.size_mm     = size,
									.first_point = first_point,
									.last_point  = last_point
						};
					}
				}

				cluster_len = 0;
				cluster_sum = 0;
				holes_used = 0;
				state = SEARCH;
			}
			break;
		}
		}
	}

	// === Wrap-around fusion and adding remaining clusters ===

	if (tempclusters.temp_cluster_cnt >= 2)
	{
		int last_idx = -1, first_idx = -1;

		for (int i = 0; i < tempclusters.temp_cluster_cnt; i++)
		{
			if (tempclusters.temp_clusters[i].start_angle >= 300) last_idx = i;
			if (tempclusters.temp_clusters[i].end_angle <= 60)  first_idx = i;
		}

		if (last_idx >= 0 && first_idx >= 0 && last_idx != first_idx)
		{
			int16_t dist_end = tempclusters.temp_clusters[last_idx].last_point;
			int16_t dist_start = tempclusters.temp_clusters[first_idx].first_point;

			if (abs(dist_end - dist_start) <= CLUSTER_THRESHOLD_MM)
			{
				float total_angle_deg = (360.0f - tempclusters.temp_clusters[last_idx].start_angle) +
						(tempclusters.temp_clusters[first_idx].end_angle + 1);
				uint16_t avg_dist = (tempclusters.temp_clusters[last_idx].avg_dist +
						tempclusters.temp_clusters[first_idx].avg_dist) / 2;
				float merged_size = calc_size_mm(avg_dist, total_angle_deg);

				if (merged_size >= MIN_CLUSTER_SIZE_MM && merged_size <= MAX_CLUSTER_SIZE_MM)
				{
					if (lidar.count < MAX_CLUSTERS)
					{
						uint16_t center = (tempclusters.temp_clusters[first_idx].end_angle + 360 +
								tempclusters.temp_clusters[last_idx].start_angle) % 360;

						LidarCluster_t *cluster = &lidar.clusters[lidar.count++];
						cluster->first_angle  = tempclusters.temp_clusters[last_idx].start_angle;
						cluster->last_angle   = tempclusters.temp_clusters[first_idx].end_angle;
						cluster->center_angle = center;
						cluster->distance_mm  = avg_dist;
						cluster->size_mm      = merged_size;
						cluster->point_count  = tempclusters.temp_clusters[last_idx].point_count +
								tempclusters.temp_clusters[first_idx].point_count;
						cluster->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
					}
					tempclusters.temp_cluster_cnt -= 2;
				}
			}
		}
	}

	// === Add remaining clusters ===
	for (int i = 0; i < tempclusters.temp_cluster_cnt && lidar.count < MAX_CLUSTERS; i++)
	{
		if (tempclusters.temp_clusters[i].size_mm >= MIN_CLUSTER_SIZE_MM &&
				tempclusters.temp_clusters[i].size_mm <= MAX_CLUSTER_SIZE_MM)
		{
			LidarCluster_t *cluster = &lidar.clusters[lidar.count++];
			cluster->first_angle  = tempclusters.temp_clusters[i].start_angle;
			cluster->last_angle   = tempclusters.temp_clusters[i].end_angle;
			cluster->center_angle = (tempclusters.temp_clusters[i].start_angle + tempclusters.temp_clusters[i].end_angle) / 2;
			cluster->distance_mm  = tempclusters.temp_clusters[i].avg_dist;
			cluster->size_mm      = tempclusters.temp_clusters[i].size_mm;
			cluster->point_count  = tempclusters.temp_clusters[i].point_count;
			cluster->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
		}
	}

	lidar.new_data = (lidar.count > 0);

	uint32_t end = DWT->CYCCNT;
	lidar.CPU_cycles = end - start;
}
