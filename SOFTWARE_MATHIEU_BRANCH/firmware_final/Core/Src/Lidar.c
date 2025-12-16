/*
 * ylidar.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Lorenzo
 */

#include <Lidar.h>




static inline float fast_atan_deg(float x)
{
	/* atan(x) rapide (radians), valide pour tout x */
    /* approximation pour |x|<=1 :
       atan(x) ≈ x*(π/4 + 0.273*(1-|x|))
       puis réduction pour |x|>1 : atan(x)=sign(x)*(π/2 - atan(1/|x|))
    */
    float ax = fabsf(x);
    float y;

    if (ax <= 1.0f) {
        y = x * ( (PI_F * 0.25f) + 0.273f * (1.0f - ax) );
    } else {
        float inv = 1.0f / ax;
        float base = inv * ( (PI_F * 0.25f) + 0.273f * (1.0f - inv) );
        y = (x >= 0.0f) ? (HALF_PI - base) : (-HALF_PI + base);
    }
    y=y*RAD2DEG;
    return y;
}

static inline float ang_correct_deg(uint16_t d_mm)
{
    if (d_mm == 0){
    	return 0.0f;
    }
    else{
    float d = (float)d_mm;
    float x = (21.8f * (155.3f - d)) / (155.3f * d);
    return fast_atan_deg(x);
    }
}


static inline uint8_t cl_dist_valid(uint16_t d)
{
    return (d >= CL_MIN_DIST_MM) && (d <= CL_MAX_DIST_MM);
}

static inline uint16_t cl_absdiff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline uint16_t cl_delta(uint16_t start, uint16_t end)
{
    return (uint16_t)((end + LIDAR_POINTS - start) % LIDAR_POINTS);
}

#if CL_ENABLE_MEDIAN3
static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t=a; a=b; b=t; }
    if (b > c) { uint16_t t=b; b=c; c=t; }
    if (a > b) { uint16_t t=a; a=b; b=t; }
    return b;
}
#endif

void ClusterizerPP_Init(ClusterizerPP_t *c)
{
    memset(c, 0, sizeof(*c));
    c->build = &c->frame_a;
    c->ready = &c->frame_b;
}

static inline void ClusterizerPP_Swap(ClusterizerPP_t *c)
{
    ClusterFrame_t *tmp = c->ready;
    c->ready = c->build;
    c->build = tmp;
    c->build->target_cnt = 0;
    c->ready_flag = 1;
}

static inline uint8_t ClusterizerPP_HasReady(const ClusterizerPP_t *c)
{
    return c->ready_flag;
}

static inline const ClusterFrame_t *ClusterizerPP_GetReady(const ClusterizerPP_t *c)
{
    return c->ready;
}

static inline void ClusterizerPP_ConsumeReady(ClusterizerPP_t *c)
{
    c->ready_flag = 0;
}



void ClusterizerPP_ProcessFullScan(ClusterizerPP_t *c,uint16_t * lidar_buf)
{
    /* ---- filtrage léger ---- */
    uint16_t buf[LIDAR_POINTS];

#if CL_ENABLE_MEDIAN3
    for (uint16_t i = 0; i < LIDAR_POINTS; i++) {
        uint16_t im1 = (i == 0) ? 359 : i - 1;
        uint16_t ip1 = (i == 359) ? 0 : i + 1;
        buf[i] = median3(lidar_buf[im1], lidar_buf[i], lidar_buf[ip1]);
    }
#else
    memcpy(buf, lidar_buf, sizeof(buf));
#endif

    TempCluster_t tmp[MAX_TARGETS];
    uint8_t tmp_cnt = 0;

    uint8_t in = 0;
    uint16_t start = 0, end = 0;
    uint32_t sum = 0;
    uint16_t min_d = 0xFFFF;
    uint16_t pts = 0;

    for (uint16_t i = 0; i < LIDAR_POINTS; i++)
    {
        uint16_t d = buf[i];
        uint16_t ip = (i == 0) ? 359 : i - 1;

        if (!in)
        {
            if (cl_dist_valid(d) && cl_dist_valid(buf[ip]) && cl_absdiff(d, buf[ip]) < CL_DERIV_THRESH_MM)
            {
                in = 1;
                start = ip;
                end = i;
                sum = buf[ip] + d;
                min_d = (buf[ip] < d) ? buf[ip] : d;
                pts = 2;
            }
        }
        else
        {
            if (!cl_dist_valid(d) || cl_absdiff(d, buf[end]) >= CL_DERIV_THRESH_MM)
            {
                in = 0;
            }
            else
            {
                end = i;
                sum += d;
                if (d < min_d) min_d = d;
                pts++;
            }

            if (!in && pts >= CL_MIN_CLUSTER_POINTS && tmp_cnt < MAX_TARGETS)
            {
                tmp[tmp_cnt++] = (TempCluster_t){
                    .start = start,
                    .end = end,
                    .avg_dist = (uint16_t)(sum / pts),
                    .min_dist = min_d,
                    .points = pts
                };
                pts = 0;
            }
        }
    }

    /* ---- fusion wrap 359°/0° ---- */
    if (tmp_cnt >= 2)
    {
        TempCluster_t *first = &tmp[0];
        TempCluster_t *last  = &tmp[tmp_cnt - 1];

        if (first->start <= CL_WRAP_MERGE_ANGLE &&
            last->end >= (359 - CL_WRAP_MERGE_ANGLE) &&
            cl_absdiff(first->avg_dist, last->avg_dist) < CL_DERIV_THRESH_MM)
        {
            first->start = last->start;
            first->points += last->points;
            first->avg_dist = (uint16_t)((first->avg_dist + last->avg_dist) / 2);
            first->min_dist = (first->min_dist < last->min_dist) ? first->min_dist : last->min_dist;
            tmp_cnt--;
        }
    }
    /* ---- conversion en targets finales ---- */
    c->build->target_cnt = 0;

    for (uint8_t i = 0; i < tmp_cnt && c->build->target_cnt < MAX_TARGETS; i++)
    {
        TempCluster_t *cl = &tmp[i];
        uint16_t width = cl_delta(cl->start, cl->end);
        float theta = (float)width * (2.0f * M_PI / 360.0f);
        float size = 2.0f * cl->avg_dist * tanf(theta * 0.5f);

        if (size < CL_MIN_CLUSTER_SIZE_MM)
            continue;

        uint16_t mid = (uint16_t)((cl->start + width / 2) % 360);

        Target_t *t = &c->build->targets[c->build->target_cnt++];
        t->angle = mid;
        t->distance = cl->min_dist;  // meilleur pour tracking
        t->width = width;
        t->points = cl->points;
    }

    ClusterizerPP_Swap(c);
}




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

			if((structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index]==0xAA) && (structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index+1)%YLIDAR_CIRC_BUF_SIZE]==0x55)){
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

			if (structlidar->LSN == 0 || structlidar->LSN > 360) {
			    structlidar->ydlidarstate = FSM_STATE_0;
			    break;
			}

			uint16_t needed = (uint16_t) structlidar->LSN * 2u + 6u;

		    if (needed > available) {
		        return HAL_BUSY;
		    }
		    structlidar->ylidar_read_index = (structlidar->ylidar_read_index + 1) % YLIDAR_CIRC_BUF_SIZE;
		    structlidar->ydlidarstate = FSM_STATE_3;
			break;

		case FSM_STATE_3:
			uint16_t FSA = 0;
			uint16_t LSA = 0;
			uint16_t distance = 0;
			uint16_t first_distance=0;
			uint16_t last_distance=0;
			uint16_t hi=0;
			uint16_t lo=0;

			FSA = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 1) % YLIDAR_CIRC_BUF_SIZE] << 8) |structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index]);
			LSA = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 3) % YLIDAR_CIRC_BUF_SIZE] << 8) |structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 2) % YLIDAR_CIRC_BUF_SIZE]);
			structlidar->CHECKSUM = (uint16_t)((structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 5) % YLIDAR_CIRC_BUF_SIZE] << 8) |structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 4) % YLIDAR_CIRC_BUF_SIZE]);
			structlidar->ylidar_read_index = (structlidar->ylidar_read_index + 6) % YLIDAR_CIRC_BUF_SIZE;

			hi = structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + 1) % YLIDAR_CIRC_BUF_SIZE];
			lo = structlidar->ylidar_circular_buffer[structlidar->ylidar_read_index];

			first_distance = (uint16_t)(((hi << 8) | lo) >> 2);

			hi = structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + (structlidar->LSN * 2) - 1) % YLIDAR_CIRC_BUF_SIZE];
			lo = structlidar->ylidar_circular_buffer[(structlidar->ylidar_read_index + ((structlidar->LSN)-1) * 2) % YLIDAR_CIRC_BUF_SIZE];

			last_distance = (uint16_t)(((hi << 8) | lo) >> 2);





			structlidar->FSA_float =(float) FSA  * 0.0078125f + ang_correct_deg(first_distance);
			structlidar->LSA_float =(float) LSA  * 0.0078125f + ang_correct_deg(last_distance);
			uint8_t wrap_packet = (structlidar->LSA_float < structlidar->FSA_float);
			if (wrap_packet) {
			    structlidar->LSA_float += 360.0f;
			}

			float angle_start = structlidar->FSA_float;
			float angle_step = 0.0f;
			if (structlidar->LSN > 1) {
				angle_step = (structlidar->LSA_float - structlidar->FSA_float) / (float)(structlidar->LSN - 1);
			}
			uint8_t did_cluster = 0;
			uint16_t read_idx = structlidar->ylidar_read_index;

			for (uint16_t i = 0; i < structlidar->LSN; i++) {

				hi = structlidar->ylidar_circular_buffer[(read_idx + 1) % YLIDAR_CIRC_BUF_SIZE];
				lo = structlidar->ylidar_circular_buffer[read_idx];
				distance = (uint16_t)(((hi << 8) | lo) >> 2);

				float angle = angle_start + angle_step * (float)i + ang_correct_deg(distance);
				int angle_index =(int)(angle + 0.5f);
			    if (wrap_packet && !did_cluster && angle >= 360.0f)
			    {
			        ClusterizerPP_ProcessFullScan(&lidar->cluster_struct, structlidar->ylidar_finalbuffer);
			        did_cluster = 1;
			    }

				if (angle_index >= 360){
					angle_index -= 360;
				}
				if (angle_index >= 360){
					angle_index -= 360;
				}
				if (angle_index < 0){
					angle_index += 360;
				}

				structlidar->ylidar_finalbuffer[angle_index] =(distance >= LidarMaxDepth) ? 0 : distance;

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






