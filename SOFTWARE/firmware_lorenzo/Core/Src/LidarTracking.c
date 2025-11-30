/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: mathi
 */

#include "LidarTracking.h"

//----- Ylidar Parsing variables-----//

uint16_t ylidar_read_index=0;
uint16_t ylidar_write_index=0;

uint8_t ylidar_circular_buffer[YLIDAR_CIRC_BUF_SIZE];

YLIDAR_STATE ydlidarstate=FSM_STATE_0;

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
} Cluster;

Cluster clusters[10];

//----- CLusterization variables-----//

#define CLUSTER_BUF_SIZE   (LIDAR_POINTS + 100)

// Buffer temporaire pour construire un cluster
static uint16_t cluster_buf[CLUSTER_BUF_SIZE];

// Stockage temporaire des clusters détectés (avant fusion wrap)
typedef struct {
    uint16_t start_angle;
    uint16_t end_angle;
    uint16_t point_count;
    uint16_t avg_dist;
    float    size_mm;
} TempCluster_t;

static TempCluster_t temp_clusters[20];
static uint8_t temp_cluster_cnt = 0;

//-----Lidar Task Code------//



//-----Ylidar Parsing State Machine-----//

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
//		float AngleCorrectLSN = ((distance) == 0) ? 0 : atanf(21.8 * ((155.3 - distance) / (155.3 * distance)));
//		uint16_t Angle2 = ((uint16_t)floorf(LSA_float+AngleCorrectLSN))%360;
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
void LidarClusterTracker_Init(void)
{
    temp_cluster_cnt = 0;
}

/*------------------------------------------------------------------*/
void LidarClusterTracker_ProcessScan(const uint16_t scan[LIDAR_POINTS], LidarClusterList_t *result)
{
    if (!scan || !result) return;

    result->count    = 0;
    result->new_data = false;
    result->scan_id++;
    temp_cluster_cnt = 0;

    uint16_t cluster_len = 0;
    uint16_t cluster_start = 0;

    enum { SEARCH, IN_CLUSTER } state = SEARCH;

    for (uint16_t i = 0; i < LIDAR_POINTS; i++)
    {
        uint16_t curr = scan[i];
        uint16_t prev = scan[(i + LIDAR_POINTS - 1) % LIDAR_POINTS];

        switch (state)
        {
            case SEARCH:
                if (curr >= MIN_VALID_DISTANCE_MM && curr <= MAX_VALID_DISTANCE_MM)
                {
                    cluster_start = i;
                    cluster_buf[0] = curr;
                    cluster_len = 1;
                    state = IN_CLUSTER;
                }
                break;

            case IN_CLUSTER:
            {
                bool end_cluster = false;

                // --- Gestion des petits trous ---
                if (curr < MIN_VALID_DISTANCE_MM || curr > MAX_VALID_DISTANCE_MM)
                {
                    uint16_t hole = 1;
                    uint16_t k = (i + 1) % LIDAR_POINTS;

                    while (hole <= MAX_HOLE_COUNT &&
                           (scan[k] < MIN_VALID_DISTANCE_MM || scan[k] > MAX_VALID_DISTANCE_MM))
                    {
                        hole++;
                        k = (k + 1) % LIDAR_POINTS;
                    }

                    uint16_t next_valid = scan[k];

                    if (hole <= MAX_HOLE_COUNT &&
                        next_valid >= MIN_VALID_DISTANCE_MM && next_valid <= MAX_VALID_DISTANCE_MM &&
                        abs((int)next_valid - (int)prev) <= HOLE_TOLERANCE_MM)
                    {
                        // On comble le trou
                        for (uint16_t h = 0; h < hole; h++)
                        {
                            if (cluster_len < CLUSTER_BUF_SIZE - 20)
                                cluster_buf[cluster_len++] = prev;
                        }
                        i += hole - 1;
                        continue;
                    }
                    else
                    {
                        end_cluster = true;
                    }
                }
                else
                {
                    if (abs((int)curr - (int)prev) > CLUSTER_THRESHOLD_MM)
                        end_cluster = true;
                    else if (cluster_len < CLUSTER_BUF_SIZE - 20)
                        cluster_buf[cluster_len++] = curr;
                }

                // --- Fin de cluster ---
                if (end_cluster || i == LIDAR_POINTS - 1)
                {
                    uint16_t end_angle = (i == LIDAR_POINTS - 1 && !end_cluster) ? i : (i + LIDAR_POINTS - 1) % LIDAR_POINTS;

                    if (cluster_len >= MIN_CLUSTER_POINTS)
                    {
                        // Calcul moyenne
                        uint32_t sum = 0;
                        for (uint16_t j = 0; j < cluster_len; j++) sum += cluster_buf[j];
                        uint16_t avg = sum / cluster_len;

                        float angle_deg = (float)(end_angle - cluster_start + 1);
                        float size = calc_size_mm(avg, angle_deg);

                        if (temp_cluster_cnt < 20)
                        {
                            temp_clusters[temp_cluster_cnt++] = (TempCluster_t){
                                .start_angle = cluster_start,
                                .end_angle   = end_angle,
                                .point_count = cluster_len,
                                .avg_dist    = avg,
                                .size_mm     = size
                            };
                        }
                    }

                    cluster_len = 0;
                    state = SEARCH;
                }
                break;
            }
        }
    }

    // === Phase 2 : Fusion des clusters qui wrappent autour de 0° ===
    if (temp_cluster_cnt >= 2)
    {
        uint8_t last_idx = 255, first_idx = 255;

        for (uint8_t i = 0; i < temp_cluster_cnt; i++)
        {
            if (temp_clusters[i].start_angle >= 300) last_idx = i;
            if (temp_clusters[i].end_angle   <= 60)  first_idx = i;
        }

        if (last_idx != 255 && first_idx != 255 && last_idx != first_idx)
        {
            // Vérifie continuité en distance
            uint16_t dist_end   = cluster_buf[temp_clusters[last_idx].point_count - 1];
            uint16_t dist_start = cluster_buf[0];  // premier point du cluster du début

            if (abs((int)dist_end - (int)dist_start) <= CLUSTER_THRESHOLD_MM)
            {
                // Fusion valide
                float total_angle_deg = (360.0f - temp_clusters[last_idx].start_angle) + (temp_clusters[first_idx].end_angle + 1);
                uint16_t avg_dist = (temp_clusters[last_idx].avg_dist + temp_clusters[first_idx].avg_dist) / 2;
                float merged_size = calc_size_mm(avg_dist, total_angle_deg);

                if (merged_size >= MIN_CLUSTER_SIZE_MM && merged_size <= MAX_CLUSTER_SIZE_MM)
                {
                    if (result->count < MAX_CLUSTERS)
                    {
                        uint16_t center = (temp_clusters[first_idx].end_angle + 360 + temp_clusters[last_idx].start_angle) % 360;

                        LidarCluster_t *cluster = &result->clusters[result->count++];
                        cluster->first_angle   = temp_clusters[last_idx].start_angle;
                        cluster->last_angle    = temp_clusters[first_idx].end_angle;
                        cluster->center_angle  = center;
                        cluster->distance_mm   = avg_dist;
                        cluster->size_mm       = merged_size;
                        cluster->point_count   = temp_clusters[last_idx].point_count + temp_clusters[first_idx].point_count;
                        cluster->timestamp_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }

                    // On retire les deux clusters fusionnés
                    for (uint8_t i = 0; i < temp_cluster_cnt; i++)
                    {
                        if (i == last_idx || i == first_idx)
                        {
                            for (uint8_t j = i; j < temp_cluster_cnt - 1; j++)
                                temp_clusters[j] = temp_clusters[j + 1];
                            temp_cluster_cnt--;
                            if (i == last_idx) last_idx--;
                            i--;
                        }
                    }
                }
            }
        }
    }

    // === Ajout des clusters non-wrappés ===
    for (uint8_t i = 0; i < temp_cluster_cnt && result->count < MAX_CLUSTERS; i++)
    {
        TempCluster_t *tc = &temp_clusters[i];
        if (tc->size_mm >= MIN_CLUSTER_SIZE_MM && tc->size_mm <= MAX_CLUSTER_SIZE_MM)
        {
            LidarCluster_t *cluster = &result->clusters[result->count++];
            cluster->first_angle   = tc->start_angle;
            cluster->last_angle    = tc->end_angle;
            cluster->center_angle  = (tc->start_angle + tc->end_angle) / 2;
            cluster->distance_mm   = tc->avg_dist;
            cluster->size_mm       = (float)tc->size_mm;
            cluster->point_count   = tc->point_count;
            cluster->timestamp_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    }

    result->new_data = (result->count > 0);
}
