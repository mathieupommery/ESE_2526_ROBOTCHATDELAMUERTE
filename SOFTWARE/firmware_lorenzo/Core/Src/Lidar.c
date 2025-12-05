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


static inline float calc_size_mm(uint16_t avg_dist_mm, float angle_deg)
{
	float rad = angle_deg * (float)M_PI / 180.0f;
	return 2.0f * (float)avg_dist_mm * tanf(rad/2.0f);
}


/*------------------------------------------------------------------*/
HAL_StatusTypeDef Clusterize(Lidar_t *clusters, Lidar_t *lidar)
{
	LiDARParsing_t *structlidar = &lidar->parse_struct;
	ClusterParsing_t *clusterstruct = &clusters -> cluster_struct;
	clusterstruct->temp_cluster_cnt = 0;
	memcpy(clusterstruct->cluster_buf, structlidar->ylidar_finalbuffer,sizeof(structlidar->ylidar_finalbuffer));
	memcpy(clusterstruct->cluster_buf + LIDAR_POINTS, structlidar->ylidar_finalbuffer,sizeof(int) * WRAP_SIZE);

	for (uint16_t i = 0; i < CLUSTER_BUF_SIZE; i++)
	{
		switch (clusterstruct->clusterizationstate)
		{
		case SEARCH:
			if (clusterstruct->cluster_buf[i] >= MIN_VALID_DISTANCE_MM && clusterstruct->cluster_buf[i] <= MAX_VALID_DISTANCE_MM && clusterstruct->temp_cluster_cnt < MAX_TEMP_CLUSTERS)
			{
				clusterstruct->temp_cluster_cnt +=1; // Called early to keep temp_cluster[0] for wrapping
				TempCluster_t *currentcluster = &clusterstruct->temp_clusters[clusterstruct->temp_cluster_cnt];
				currentcluster->cluster[0] = clusterstruct->cluster_buf[i];
				currentcluster->point_count = 1;
				currentcluster->start_angle = i % LIDAR_POINTS;
				clusterstruct->clusterizationstate = IN_CLUSTER;
			}
			break;
		case IN_CLUSTER:
			TempCluster_t *currentcluster = &clusterstruct->temp_clusters[clusterstruct->temp_cluster_cnt];
			if (abs((int)clusterstruct->cluster_buf[i] - (int)clusterstruct->cluster_buf[(i + CLUSTER_BUF_SIZE - 1) % CLUSTER_BUF_SIZE]) > CLUSTER_THRESHOLD_MM){
				if (currentcluster->hole_cnt <= MAX_HOLE_COUNT)
				{
					uint8_t inhole = 1;
					uint8_t k = 1;
					// In short we check if the hole is too large, if it is not we skip the parsing to the end of the hole.
					while (inhole == 1 && k <= MAX_HOLE_SIZE && (i + k) < CLUSTER_BUF_SIZE)
					{
						if (abs((int)clusterstruct->cluster_buf[i-1] - (int)clusterstruct->cluster_buf[(i + k + CLUSTER_BUF_SIZE - 1) % CLUSTER_BUF_SIZE]) > CLUSTER_THRESHOLD_MM){
							k+=1;
						}
						else{
							inhole = 0;
						}
					}
					if (inhole == 0)
					{
						currentcluster->cluster[currentcluster->point_count + k] = clusterstruct->cluster_buf[i + k];
						currentcluster->hole_points += 1;
						currentcluster->point_count += k;
						i += k;
					}
					else{
						clusterstruct->clusterizationstate = END_CLUSTER;
					}
				}
				else
				{
					clusterstruct->clusterizationstate = END_CLUSTER;
				}
			}
			// Store cluster_buf[i] as part of the current cluster object
			currentcluster->cluster[currentcluster->point_count] = clusterstruct->cluster_buf[i];
			break;
		case END_CLUSTER:
			TempCluster_t *firstcluster = &clusterstruct->temp_clusters[1];
			// Store end of cluster angle in the cluster object
			currentcluster->end_angle = (i - 1) % LIDAR_POINTS;
			if (currentcluster->end_angle <= firstcluster->end_angle && currentcluster->end_angle >= firstcluster->start_angle)
			{
				// replace the first cluster if it is incomplete with the new complete one to achieve wrapping
				firstcluster->avg_dist=currentcluster->avg_dist;
				firstcluster->end_angle=currentcluster->end_angle;
				firstcluster->point_count=currentcluster->point_count;
				firstcluster->size_mm=currentcluster->size_mm;
				firstcluster->start_angle=currentcluster->start_angle;
				firstcluster->hole_cnt=currentcluster->hole_cnt;
				firstcluster->point_count=currentcluster->point_count;
				memset(firstcluster->cluster,0,sizeof(currentcluster->cluster));
				memcpy(firstcluster->cluster,currentcluster->cluster,sizeof(currentcluster->cluster));
				currentcluster = firstcluster; // from now on we consider firstcluster as the currentcluster

			}
			if (currentcluster->end_angle <= firstcluster->start_angle)
			{
				// Load the cluster in temp_cluster[0] to achieve wrapping
				TempCluster_t *clusterzero = &clusterstruct->temp_clusters[0];
				clusterzero->avg_dist=currentcluster->avg_dist;
				clusterzero->end_angle=currentcluster->end_angle;
				clusterzero->point_count=currentcluster->point_count;
				clusterzero->size_mm=currentcluster->size_mm;
				clusterzero->start_angle=currentcluster->start_angle;
				clusterzero->hole_points=currentcluster->hole_points;
				clusterzero->point_count=currentcluster->point_count;
				memcpy(clusterzero->cluster,currentcluster->cluster,sizeof(currentcluster->cluster));
				currentcluster->avg_dist=0;
				currentcluster->end_angle=0;
				currentcluster->point_count=0;
				currentcluster->size_mm=0;
				currentcluster->start_angle=0;
				memset(currentcluster->cluster,0,sizeof(currentcluster->cluster));
				currentcluster = clusterzero; // from now on we consider clusterzero as the currentcluster
			}
			// If the cluster is neither too small or too big:
			if (currentcluster->point_count >= MIN_CLUSTER_POINTS && currentcluster->point_count <= MAX_CLUSTER_POINTS)
			{
				// Compute sum of all distances but ignore the holes
				uint32_t sum = 0;
				for (uint16_t j = 0; j < currentcluster->point_count; j++) sum += currentcluster->cluster[j];

				// Store average distance in the cluster object
				currentcluster->avg_dist = sum / currentcluster->point_count;

				// Compute and store the cluster object's size
				float angle_deg = (float)(currentcluster->end_angle - currentcluster->start_angle);
				float rad = angle_deg * (float)M_PI / 180.0f;
				currentcluster->size_mm  =  2.0f * currentcluster->avg_dist * tanf(rad/2.0f);
				if (currentcluster->size_mm >= MIN_CLUSTER_SIZE_MM - currentcluster->hole_points)
				{
					if (clusterstruct->target_cnt >= MAX_TARGETS)
					{
						//						return HAL_TARGETS_OVERFLOW;
						clusterstruct->clusterizationstate = SEARCH;
						return HAL_ERROR;
					}
					else
					{
						// Set the cluster as target and wipe it clean
						Target_t *target = &clusterstruct->targets[clusterstruct->target_cnt];
						target->angle = (currentcluster->end_angle - currentcluster->start_angle)/2;
						target->distance = currentcluster->avg_dist;
						target->size = currentcluster->size_mm;

						currentcluster->avg_dist=0;
						currentcluster->end_angle=0;
						currentcluster->point_count=0;
						currentcluster->size_mm=0;
						currentcluster->start_angle=0;
						currentcluster->hole_points=0;
						currentcluster->point_count=0;
						memset(currentcluster->cluster,0,sizeof(currentcluster->cluster));

						clusterstruct->clusterizationstate = SEARCH;
						break;
					}
				}
				else
				{
					// Wipe the cluster clean and setback the count if the detected object is too small
					clusterstruct->temp_cluster_cnt -= 1;
					currentcluster->avg_dist=0;
					currentcluster->end_angle=0;
					currentcluster->point_count=0;
					currentcluster->size_mm=0;
					currentcluster->start_angle=0;
					memset(currentcluster->cluster,0,sizeof(currentcluster->cluster));
					clusterstruct->clusterizationstate = SEARCH;
				}
			}
			clusterstruct->clusterizationstate = SEARCH;
			break;
		default:
			clusterstruct->clusterizationstate = SEARCH;
			break;
		}
	}
	return HAL_OK;
}
