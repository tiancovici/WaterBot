#ifndef UTIL_H
#define UTIL_H
//================== Includes ======================================//
//============== Type Definiton ========================================//
typedef signed short s16_t;
typedef unsigned short u16_t;
typedef unsigned int u32_t;
typedef signed int s32_t;
typedef float f32_t;
typedef double f64_t;

typedef struct
{
	f64_t x;
	f64_t y;
	f64_t t;
	f64_t w;
}Pos_t;
//============== Symbolic Constants ================================//
#define PI 3.14159265 	/* pi */

const f32_t MAP_WIDTH   = 53.9f;
const f32_t MAP_HEIGHT  = 10.6f;
const f32_t MAP_PPM     = 10.0f;
const f32_t LASER_MAX   =  5.0f;
#define PI 3.14159265 	/* pi */
#define NUM_PARTICLES 1800
#define K 5			  	/* # Samples */

#define CCW_TIME 100
#define FW_TIME 10
#define TOTAL_TIME (CCW_TIME + FW_TIME)


const f64_t std_hit   = 0.5f;
const f64_t z_hit   = 0.9f;
const f64_t z_max   = 0.05f;
const f64_t z_rand  = 0.05f;
#endif /* End of ROBOT_H*/
