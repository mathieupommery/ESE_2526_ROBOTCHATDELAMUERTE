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

// angle diff [0..180]
static inline uint16_t angle_diff_deg(uint16_t a, uint16_t b)
{
    uint16_t d = cl_absdiff(a, b);
    return (d > 180u) ? (360u - d) : d;
}

// signed diff [-180..180]
static inline int16_t angle_diff_signed(uint16_t cur, uint16_t prev)
{
    int16_t d = (int16_t)cur - (int16_t)prev;
    if (d > 180)  d -= 360;
    if (d < -180) d += 360;
    return d;
}

static inline uint16_t angle_wrap_0_359(int32_t a)
{
    a %= 360;
    if (a < 0) a += 360;
    return (uint16_t)a;
}

// IIR entier: y += (x-y)/2^shift
static inline int32_t iir_s32(int32_t y, int32_t x, uint8_t shift)
{
    return y + ((x - y) >> shift);
}

static inline uint16_t angle_iir(uint16_t prev, uint16_t meas, uint8_t shift)
{
    int16_t d = angle_diff_signed(meas, prev);
    int32_t a = (int32_t)prev + ((int32_t)d >> shift);
    return angle_wrap_0_359(a);
}


void ClusterizerPP_Init(ClusterizerPP_t *c)
{
    memset(c, 0, sizeof(*c));
    c->build = &c->frame_a;
    c->ready = &c->frame_b;

    c->can_valid = 0;
    c->can_missed = 0;
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

static void ClusterizerPP_SelectCanAndSmooth(ClusterizerPP_t *c)
{
    ClusterFrame_t *cur = c->build;

    // aucune détection
    if (cur->target_cnt == 0)
    {
        if (c->can_valid) {
            if (++c->can_missed > CAN_MISS_MAX) {
                c->can_valid = 0;
                c->can_missed = 0;
            }
        }
        cur->target_cnt = 0;
        return;
    }

    int best = -1;
    uint32_t best_cost = 0xFFFFFFFFu;

    for (uint8_t i = 0; i < cur->target_cnt; i++)
    {
        Target_t *t = &cur->targets[i];

        // fenêtre canette
        if (t->distance < CAN_MIN_DIST_MM || t->distance > CAN_MAX_DIST_MM) continue;
        if (t->width < CAN_MIN_MM || t->width > CAN_MAX_MM) continue;

        uint32_t size_err = (t->width > CAN_DIAM_MM) ? (t->width - CAN_DIAM_MM) : (CAN_DIAM_MM - t->width);

        // cohérence avec track existant
        uint32_t coher = 0;
        if (c->can_valid)
        {
            uint16_t da = angle_diff_deg(t->angle, c->smooth_can.angle);
            uint16_t dd = cl_absdiff(t->distance, c->smooth_can.distance);
            coher = (uint32_t)da * 50u + (uint32_t)dd;  // poids angle fort
        }

        // coût global (à ajuster)
        uint32_t cost = size_err * 200u + coher;

        // bonus: plus de points = mieux
        if (t->points < 8) cost += (8u - t->points) * 200u;

        if (cost < best_cost) {
            best_cost = cost;
            best = (int)i;
        }
    }

    if (best < 0)
    {
        if (c->can_valid) {
            if (++c->can_missed > CAN_MISS_MAX) {
                c->can_valid = 0;
                c->can_missed = 0;
            }
        }
        cur->target_cnt = 0;
        return;
    }

    Target_t meas = cur->targets[best];

    // init
    if (!c->can_valid)
    {
        c->smooth_can = meas;
        c->can_valid = 1;
        c->can_missed = 0;
    }
    else
    {
        // gating anti-sauts
        uint16_t da = angle_diff_deg(meas.angle, c->smooth_can.angle);
        uint16_t dd = cl_absdiff(meas.distance, c->smooth_can.distance);

        if (da > CAN_GATE_MAX_DA_DEG || dd > CAN_GATE_MAX_DD_MM)
        {
            // mesure trop loin : on ignore ce scan
            if (++c->can_missed > CAN_MISS_MAX) {
                c->can_valid = 0;
                c->can_missed = 0;
            }
            cur->target_cnt = 0;
            return;
        }

        // IIR
        c->smooth_can.angle = angle_iir(c->smooth_can.angle, meas.angle, CAN_SMOOTH_SHIFT_ANG);

        int32_t dD = (int32_t)meas.distance;
        c->smooth_can.distance = (uint16_t)iir_s32((int32_t)c->smooth_can.distance, dD, CAN_SMOOTH_SHIFT_DIST);

        int32_t dS = (int32_t)meas.width;
        c->smooth_can.width = (uint16_t)iir_s32((int32_t)c->smooth_can.width, dS, CAN_SMOOTH_SHIFT_SIZE);

        c->smooth_can.points = meas.points;
        c->can_missed = 0;
    }

    // sortie: 1 target (lissée)
    cur->targets[0] = c->smooth_can;
    cur->target_cnt = 1;
}


void ClusterizerPP_ProcessFullScan(ClusterizerPP_t *c, const uint16_t *lidar_buf)
{
    // 1) filtre spatial léger
    uint16_t buf[LIDAR_POINTS];

#if CL_ENABLE_MEDIAN3
    for (uint16_t i = 0; i < LIDAR_POINTS; i++) {
        uint16_t im1 = (i == 0) ? (LIDAR_POINTS - 1) : (i - 1);
        uint16_t ip1 = (i + 1u >= LIDAR_POINTS) ? 0u : (i + 1u);
        buf[i] = median3(lidar_buf[im1], lidar_buf[i], lidar_buf[ip1]);
    }
#else
    memcpy(buf, lidar_buf, sizeof(buf));
#endif

    // 2) extraction clusters temporaires
    TempCluster_t tmp[MAX_TARGETS];
    uint8_t tmp_cnt = 0;

    uint8_t  in = 0;
    uint16_t start = 0, end = 0;
    uint32_t sum = 0;
    uint16_t min_d = 0xFFFF;
    uint16_t pts = 0;

    uint8_t  hole_run = 0;
    uint16_t last_valid = 0;
    uint8_t  has_last_valid = 0;

    for (uint16_t i = 0; i < LIDAR_POINTS; i++)
    {
        uint16_t d  = buf[i];
        uint16_t ip = (i == 0) ? (LIDAR_POINTS - 1) : (i - 1);

        if (!in)
        {
            if (cl_dist_valid(d) && cl_dist_valid(buf[ip]) &&
                cl_absdiff(d, buf[ip]) < CL_DERIV_THRESH_MM)
            {
                in = 1;
                start = ip;
                end   = i;
                sum   = (uint32_t)buf[ip] + (uint32_t)d;
                min_d = (buf[ip] < d) ? buf[ip] : d;
                pts   = 2;

                hole_run = 0;
                last_valid = d;
                has_last_valid = 1;
            }
        }
        else
        {
            if (!cl_dist_valid(d))
            {
                if (hole_run < CL_MAX_HOLE_RUN) {
                    hole_run++;
                } else {
                    in = 0;
                }
            }
            else
            {
                if (!has_last_valid) {
                    last_valid = d;
                    has_last_valid = 1;
                }

                if (hole_run > 0)
                {
                    if (cl_absdiff(d, last_valid) > CL_HOLE_REJOIN_THRESH) {
                        in = 0;
                    } else {
                        hole_run = 0;
                    }
                }

                if (in)
                {
                    if (cl_absdiff(d, buf[end]) >= CL_DERIV_THRESH_MM) {
                        in = 0;
                    } else {
                        end = i;
                        sum += d;
                        if (d < min_d) min_d = d;
                        pts++;
                        last_valid = d;
                        hole_run = 0;
                        has_last_valid = 1;
                    }
                }
            }

            if (!in && pts >= CL_MIN_CLUSTER_POINTS && tmp_cnt < MAX_TARGETS)
            {
                tmp[tmp_cnt++] = (TempCluster_t){
                    .start    = start,
                    .end      = end,
                    .avg_dist = (uint16_t)(sum / (uint32_t)pts),
                    .min_dist = min_d,
                    .points   = pts
                };

                pts = 0; sum = 0; min_d = 0xFFFF;
                hole_run = 0; has_last_valid = 0;
            }
        }
    }

    // flush fin de scan
    if (in && pts >= CL_MIN_CLUSTER_POINTS && tmp_cnt < MAX_TARGETS)
    {
        tmp[tmp_cnt++] = (TempCluster_t){
            .start    = start,
            .end      = end,
            .avg_dist = (uint16_t)(sum / (uint32_t)pts),
            .min_dist = min_d,
            .points   = pts
        };
    }

    // 3) merge wrap 359/0
    if (tmp_cnt >= 2)
    {
        TempCluster_t *first = &tmp[0];
        TempCluster_t *last  = &tmp[tmp_cnt - 1];

        if (first->start <= CL_WRAP_MERGE_ANGLE &&
            last->end >= (uint16_t)((LIDAR_POINTS - 1) - CL_WRAP_MERGE_ANGLE) &&
            cl_absdiff(first->avg_dist, last->avg_dist) < CL_DERIV_THRESH_MM)
        {
            first->start = last->start;
            first->points += last->points;

            uint32_t wsum = (uint32_t)first->avg_dist * first->points + (uint32_t)last->avg_dist * last->points;
            first->avg_dist = (uint16_t)(wsum / (uint32_t)first->points);

            first->min_dist = (first->min_dist < last->min_dist) ? first->min_dist : last->min_dist;
            tmp_cnt--;
        }
    }

    // 4) conversion en candidats Target_t (c->build)
    c->build->target_cnt = 0;

    for (uint8_t k = 0; k < tmp_cnt && c->build->target_cnt < MAX_TARGETS; k++)
    {
        TempCluster_t *cl = &tmp[k];

        uint16_t delta_deg = cl_delta(cl->start, cl->end);
        uint16_t mid = (uint16_t)((cl->start + (delta_deg / 2u)) % LIDAR_POINTS);

        float theta_rad = (float)delta_deg * (2.0f * (float)M_PI / (float)LIDAR_POINTS);
        float size_mm_f = 2.0f * (float)cl->avg_dist * tanf(theta_rad * 0.5f);
        uint16_t size_mm = (size_mm_f <= 0.0f) ? 0u : (uint16_t)(size_mm_f + 0.5f);

        if (size_mm < CL_MIN_CLUSTER_SIZE_MM || size_mm > CL_MAX_CLUSTER_SIZE_MM)
            continue;

        Target_t *t = &c->build->targets[c->build->target_cnt++];
        t->angle    = mid;
        t->distance = cl->min_dist;   // meilleur pour obstacle
        t->width    = size_mm;        // taille mm
        t->points   = cl->points;
    }

    ClusterizerPP_SelectCanAndSmooth(c);
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

	ClusterizerPP_Init(&lidar->cluster_struct);

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






