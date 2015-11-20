#ifndef UTIL_H
#define UTIL_H
//================== Includes ======================================//
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "sensor_msgs/Joy.h"
//============== Type Definiton ========================================//
typedef signed short s16_t;
typedef unsigned short u16_t;
typedef unsigned int u32_t;
typedef signed int s32_t;
typedef float f32_t;
typedef double f64_t;

typedef sensor_msgs::LaserScan Laser_t;
typedef geometry_msgs::Twist Twist_t;
typedef nav_msgs::OccupancyGrid Map_t;

typedef enum
{
	A = 0,
	B,
	X,
	Y,
	LB,
	RB,
	BACK,
	START,
	POWER,
	LEFT,
	RIGHT
}button_t;

typedef struct
{
	f32_t x;
	f32_t y;
	f32_t t;
	f32_t w;
}Particle_t;
//============== Symbolic Constants ================================//
#define PI 3.14159265 	/* pi */

#endif /* End of ROBOT_H*/
