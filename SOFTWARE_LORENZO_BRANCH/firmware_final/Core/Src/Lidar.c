/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Lorenzo
 */

#include <Lidar.h>

HAL_StatusTypeDef Lidar_Init(Lidar_t *lidar,UART_HandleTypeDef *huart,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	LiDARParsing_t *structlidar = &lidar->parse_struct;
	lidar->huart=huart;
	lidar->gpio_port=GPIOx;
	lidar->gpio_pin=GPIO_Pin;

	HAL_GPIO_WritePin(lidar->gpio_port,lidar->gpio_pin,GPIO_PIN_SET);


	HAL_UART_Abort(huart);
	if(HAL_UART_Receive_DMA(huart,structlidar->ylidar_circular_buffer, YLIDAR_CIRC_BUF_SIZE)!= HAL_OK){
		return HAL_ERROR;
	}
	__HAL_DMA_ENABLE_IT(lidar->huart->hdmarx,DMA_IT_HT);
	__HAL_DMA_ENABLE_IT(lidar->huart->hdmarx,DMA_IT_TC);

	structlidar->ylidar_read_index=0;
	structlidar->ylidar_write_index=0;

	return HAL_OK;

}


HAL_StatusTypeDef ylidar_fsm(Lidar_t *lidar)
{
	LiDARParsing_t *structlidar = &lidar->parse_struct;

	uint16_t available=0;

	if (structlidar->ylidar_write_index >= structlidar->ylidar_read_index) {
		available = structlidar->ylidar_write_index - structlidar->ylidar_read_index;
	}

	else {
		available = (YLIDAR_CIRC_BUF_SIZE - structlidar->ylidar_read_index) + structlidar->ylidar_write_index;
	}

	if(available<=2){
		return HAL_BUSY;
	}

	while(available > 2){

		if(available<=2){
			return HAL_BUSY;
		}


		if (structlidar->ylidar_write_index >= structlidar->ylidar_read_index) {
			available = structlidar->ylidar_write_index - structlidar->ylidar_read_index;
		}
		else {
			available = (YLIDAR_CIRC_BUF_SIZE - structlidar->ylidar_read_index) + structlidar->ylidar_write_index;
		}

		switch (structlidar->ydlidarstate)
		{
		case FSM_STATE_0:

			if((structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index]==0xAA)&& (structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]==0x55)){
				structlidar->ydlidarstate=FSM_STATE_1;
				structlidar->ylidar_read_index=(structlidar->ylidar_read_index+2)%YLIDAR_CIRC_BUF_SIZE;
			}
			else{
				structlidar->ylidar_read_index=(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			}

			break;

		case FSM_STATE_1:

			if(!(structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index]&0x01)){
				structlidar->ydlidarstate=FSM_STATE_2;
				structlidar->CT = structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index];
				structlidar->ylidar_read_index=(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
			}
			else{
				structlidar->ylidar_read_index=(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
				structlidar->ydlidarstate=FSM_STATE_0;
			}
			break;

		case FSM_STATE_2:

			structlidar->LSN=structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index];

			uint16_t needed=structlidar->LSN*2+6;

			if(needed<=available){
				structlidar->ylidar_read_index=(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE;
				structlidar->ydlidarstate=FSM_STATE_3;
			}

			break;

		case FSM_STATE_3:
			uint16_t FSA = 0;
			uint16_t LSA = 0;
			uint16_t distance = 0;

			FSA = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 1) % YLIDAR_CIRC_BUF_SIZE] << 8) |
					structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index]);
			LSA = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 3) % YLIDAR_CIRC_BUF_SIZE] << 8) |
					structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 2) % YLIDAR_CIRC_BUF_SIZE]);
			structlidar->CHECKSUM = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 5) % YLIDAR_CIRC_BUF_SIZE] << 8) |
					structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 4) % YLIDAR_CIRC_BUF_SIZE]);
			structlidar->ylidar_read_index = (structlidar->ylidar_read_index + 6) % YLIDAR_CIRC_BUF_SIZE;

			structlidar->FSA_float = FSA * (1.0f / 128.0f);
			structlidar->LSA_float = LSA * (1.0f / 128.0f);

			if (structlidar->LSA_float < structlidar->FSA_float) {
				structlidar->LSA_float += 360.0f;
			}

			float angle_start = structlidar->FSA_float;
			float angle_step = 0.0f;
			if (structlidar->LSN > 1) {
				angle_step = (structlidar->LSA_float - structlidar->FSA_float) / (float)(structlidar->LSN - 1);
			}

			uint16_t read_idx = structlidar->ylidar_read_index;

			for (uint16_t i = 0; i < structlidar->LSN; i++) {

				uint16_t hi = structlidar->ylidar_circular_buffer[(read_idx + 1) % YLIDAR_CIRC_BUF_SIZE];
				uint16_t lo = structlidar->ylidar_circular_buffer[read_idx];
				distance = (uint16_t)(((hi << 8) | lo) >> 2);   // /4 en entier

				float angle = angle_start + angle_step * (float)i;
				int angle_index = (int)angle;                   // équivalent à floor() pour angle >= 0

				if (angle_index >= 360) {
					angle_index = angle_index % 360;                         // plus rapide qu’un %
				}

				structlidar->ylidar_finalbuffer[angle_index] =
						(distance >= LidarMaxDepth) ? 0 : distance;

				read_idx =(read_idx + 2) % YLIDAR_CIRC_BUF_SIZE;
			}

			structlidar->ylidar_read_index = read_idx;

			structlidar->ydlidarstate=FSM_STATE_0;

			break;

		default:
			structlidar->ydlidarstate=FSM_STATE_0;
			break;
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef Clusterize(Lidar_t *lidar)
{
    LiDARParsing_t *structlidar = &lidar->parse_struct;
    ClusterParsing_t *clusterstruct = &lidar->cluster_struct;

    /* --- init --- */
    clusterstruct->target_cnt = 0;
    clusterstruct->temp_cluster_cnt = 0;
    clusterstruct->clusterizationstate = SEARCH; /* ensure defined */

    /* copy buffer: expect CLUSTER_BUF_SIZE == LIDAR_POINTS + WRAP_SIZE */
    memcpy(clusterstruct->cluster_buf, structlidar->ylidar_finalbuffer, sizeof(uint16_t) * LIDAR_POINTS);
    memcpy(clusterstruct->cluster_buf + LIDAR_POINTS, structlidar->ylidar_finalbuffer, sizeof(uint16_t) * WRAP_SIZE);

    /* init cluster zero */
    TempCluster_t *clusterzero = &clusterstruct->temp_clusters[0];
    clusterzero->avg_dist = 0;
    clusterzero->end_angle = 0;
    clusterzero->point_count = 0;
    clusterzero->size_mm = 0;
    clusterzero->start_angle = 0;
    clusterzero->hole_points = 0;
    clusterzero->hole_cnt = 0;
    memset(clusterzero->cluster, 0, sizeof(clusterzero->cluster)); /* consistent size */

    /* helper to get safe previous index */
    #define PREV_IDX(x) (((x) + CLUSTER_BUF_SIZE - 1) % CLUSTER_BUF_SIZE)

    for (int i = 0; i < CLUSTER_BUF_SIZE; i++)
    {
        switch (clusterstruct->clusterizationstate)
        {
            case SEARCH:
            {
                int prev = PREV_IDX(i);
                int diff_abs = abs((int)clusterstruct->cluster_buf[prev] - (int)clusterstruct->cluster_buf[i]);
                if (diff_abs > CLUSTER_THRESHOLD_MM)
                {
                    /* start a new cluster: keep temp_clusters[0] for wrapping */
                    if (clusterstruct->temp_cluster_cnt + 1 >= MAX_TEMP_CLUSTERS) {
                        /* defensive: avoid overflow */
                        clusterstruct->clusterizationstate = SEARCH;
                        break;
                    }
                    clusterstruct->temp_cluster_cnt += 1;
                    TempCluster_t *currentcluster = &clusterstruct->temp_clusters[clusterstruct->temp_cluster_cnt];
                    /* init current cluster */
                    currentcluster->point_count = 1;
                    currentcluster->cluster[0] = clusterstruct->cluster_buf[i];
                    currentcluster->start_angle = i % LIDAR_POINTS;
                    currentcluster->end_angle = 0;
                    currentcluster->avg_dist = 0;
                    currentcluster->size_mm = 0;
                    currentcluster->hole_points = 0;
                    currentcluster->hole_cnt = 0;
                    clusterstruct->clusterizationstate = IN_CLUSTER;
                }
                break;
            }

            case IN_CLUSTER:
            {
                /* ensure temp_cluster_cnt is valid */
                if (clusterstruct->temp_cluster_cnt == 0) {
                    /* nothing to append to; reset to SEARCH to be safe */
                    clusterstruct->clusterizationstate = SEARCH;
                    break;
                }
                TempCluster_t *currentcluster = &clusterstruct->temp_clusters[clusterstruct->temp_cluster_cnt];

                /* previous index safe */
                int prev = PREV_IDX(i);
                uint16_t a = clusterstruct->cluster_buf[i];
                uint16_t b = clusterstruct->cluster_buf[prev];
                uint16_t diff = (a > b) ? (a - b) : (b - a);

                if (diff > CLUSTER_THRESHOLD_MM)
                {
                    if (currentcluster->hole_cnt <= MAX_HOLE_COUNT)
                    {
                        uint8_t inhole = 1;
                        uint8_t k = 1;
                        /* check hole using adjacent samples at (i + k - 1) and (i + k) */
                        while (inhole == 1 && k < MAX_HOLE_SIZE && (i + k) < CLUSTER_BUF_SIZE)
                        {
                            int idx1 = i + k - 1;
                            int idx2 = i + k;
                            uint16_t v1 = clusterstruct->cluster_buf[idx1];
                            uint16_t v2 = clusterstruct->cluster_buf[idx2];
                            uint16_t diff_k = (v1 > v2) ? (v1 - v2) : (v2 - v1);

                            if (diff_k > CLUSTER_THRESHOLD_MM) {
                                k++;
                            } else {
                                inhole = 0;
                            }
                        }

                        if (inhole == 0)
                        {
                            /* ensure we don't write out of bounds */
                            if ((size_t)currentcluster->point_count + k + 1 < MAX_CLUSTER_POINTS) {
                                currentcluster->point_count += k;
                                currentcluster->cluster[currentcluster->point_count] = clusterstruct->cluster_buf[i + k];
                                currentcluster->hole_points += k;
                                /* increase hole count */
                                currentcluster->hole_cnt += 1;
                                i += k; /* skip ahead */
                                break;
                            } else {
                                /* overflow attempt — end cluster */
                                clusterstruct->clusterizationstate = END_CLUSTER;
                                break;
                            }
                        }
                        else
                        {
                            clusterstruct->clusterizationstate = END_CLUSTER;
                            break;
                        }
                    }
                    else
                    {
                        clusterstruct->clusterizationstate = END_CLUSTER;
                        break;
                    }
                }

                /* store current point as part of cluster, check bounds */
                if ((size_t)currentcluster->point_count + 1 < MAX_CLUSTER_POINTS) {
                    currentcluster->point_count += 1;
                    currentcluster->cluster[currentcluster->point_count] = clusterstruct->cluster_buf[i];
                } else {
                    /* cluster buffer overflow => end cluster defensively */
                    clusterstruct->clusterizationstate = END_CLUSTER;
                }
                break;
            }

            case END_CLUSTER:
            {
                /* get currentcluster pointer safely */
                if (clusterstruct->temp_cluster_cnt == 0) {
                    /* nothing to end */
                    clusterstruct->clusterizationstate = SEARCH;
                    break;
                }
                TempCluster_t *currentcluster = &clusterstruct->temp_clusters[clusterstruct->temp_cluster_cnt];
                TempCluster_t *firstcluster = &clusterstruct->temp_clusters[1];

                /* end angle is previous index of i */
                currentcluster->end_angle = (i - 1 + LIDAR_POINTS) % LIDAR_POINTS;

                /* wrapping handling: replace first cluster if needed */
                if (clusterstruct->temp_cluster_cnt > 1) {
                    if (currentcluster->end_angle <= firstcluster->end_angle &&
                        currentcluster->end_angle >= firstcluster->start_angle)
                    {
                        /* copy fields */
                        firstcluster->avg_dist = currentcluster->avg_dist;
                        firstcluster->end_angle = currentcluster->end_angle;
                        firstcluster->point_count = currentcluster->point_count;
                        firstcluster->size_mm = currentcluster->size_mm;
                        firstcluster->start_angle = currentcluster->start_angle;
                        firstcluster->hole_cnt = currentcluster->hole_cnt;
                        firstcluster->hole_points = currentcluster->hole_points;
                        memset(firstcluster->cluster, 0, sizeof(firstcluster->cluster));
                        memcpy(firstcluster->cluster, currentcluster->cluster, sizeof(firstcluster->cluster));
                        currentcluster = firstcluster;
                    }
                }

                /* move to clusterzero if appropriate */
                if (clusterstruct->temp_cluster_cnt > 1 &&
                    currentcluster->end_angle <= firstcluster->start_angle)
                {
                    TempCluster_t *clusterzero_dst = &clusterstruct->temp_clusters[0];
                    *clusterzero_dst = *currentcluster; /* shallow copy whole struct is fine if no pointers */
                    memcpy(clusterzero_dst->cluster, currentcluster->cluster, sizeof(clusterzero_dst->cluster));
                    /* clear the old currentcluster */
                    currentcluster->avg_dist = 0;
                    currentcluster->end_angle = 0;
                    currentcluster->point_count = 0;
                    currentcluster->size_mm = 0;
                    currentcluster->start_angle = 0;
                    currentcluster->hole_points = 0;
                    currentcluster->hole_cnt = 0;
                    memset(currentcluster->cluster, 0, sizeof(currentcluster->cluster));
                    currentcluster = clusterzero_dst;
                }

                /* sum distances */
                uint32_t sum = 0;
                if (currentcluster->point_count == 0) {
                    /* nothing to do; reset and continue */
                    clusterstruct->temp_cluster_cnt = (clusterstruct->temp_cluster_cnt > 0) ? clusterstruct->temp_cluster_cnt - 1 : 0;
                    clusterstruct->clusterizationstate = SEARCH;
                    break;
                }
                for (uint16_t j = 0; j < currentcluster->point_count; j++) {
                    sum += currentcluster->cluster[j];
                }

                /* safe division */
                currentcluster->avg_dist = sum / currentcluster->point_count;

                /* compute cluster size (angles are modulo 360 but your LIDAR_POINTS logic kept it simple) */
                float angle_deg = (float)(currentcluster->end_angle - currentcluster->start_angle);
                float rad = angle_deg * (float)M_PI / 180.0f;
                currentcluster->size_mm = 2.0f * currentcluster->avg_dist * tanf(rad / 2.0f);

                if (currentcluster->size_mm >= ((float)MIN_CLUSTER_SIZE_MM - (float)currentcluster->hole_points))
                {
                    if (clusterstruct->target_cnt >= MAX_TARGETS) {
                        clusterstruct->clusterizationstate = SEARCH;
                    } else {
                        /* add target, increment target count */
                        Target_t *target = &clusterstruct->targets[clusterstruct->target_cnt];
                        target->angle = currentcluster->start_angle + (currentcluster->end_angle - currentcluster->start_angle) / 2;
                        target->distance = currentcluster->avg_dist;
                        target->size = currentcluster->size_mm;

                        /* clear currentcluster */
                        currentcluster->avg_dist = 0;
                        currentcluster->end_angle = 0;
                        currentcluster->point_count = 0;
                        currentcluster->size_mm = 0;
                        currentcluster->start_angle = 0;
                        currentcluster->hole_points = 0;
                        currentcluster->hole_cnt = 0;
                        memset(currentcluster->cluster, 0, sizeof(currentcluster->cluster));

                        clusterstruct->temp_cluster_cnt = (clusterstruct->temp_cluster_cnt > 0) ? clusterstruct->temp_cluster_cnt - 1 : 0;
                        clusterstruct->target_cnt += 1; /* increment target count! */
                        clusterstruct->clusterizationstate = SEARCH;
                    }
                }
                else
                {
                    /* cluster too small -> wipe and continue */
                    currentcluster->avg_dist = 0;
                    currentcluster->end_angle = 0;
                    currentcluster->point_count = 0;
                    currentcluster->size_mm = 0;
                    currentcluster->start_angle = 0;
                    memset(currentcluster->cluster, 0, sizeof(currentcluster->cluster));

                    clusterstruct->temp_cluster_cnt = (clusterstruct->temp_cluster_cnt > 0) ? clusterstruct->temp_cluster_cnt - 1 : 0;
                    clusterstruct->clusterizationstate = SEARCH;
                }

                break;
            }

            default:
                clusterstruct->clusterizationstate = SEARCH;
                break;
        } /* end switch */
    } /* end for */

    /* clear remaining targets slots (use < MAX_TARGETS) */
    for (int i = clusterstruct->target_cnt; i < MAX_TARGETS; i++) {
        Target_t *target = &clusterstruct->targets[i];
        target->angle = 0;
        target->distance = 0;
        target->size = 0;
    }

    #undef PREV_IDX
    return HAL_OK;
}
