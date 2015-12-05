#ifndef PARTICLE_H
#define PARTICLE_H

//================== Includes ======================================//
/* Ros Libraries */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/* Tom's libraries */
#include "types.h"
//============== Internal Functions Definitions ========================//
inline f64_t gauss(f64_t x, f64_t mu, f64_t sigma){	
	return 1.0f / (sigma * 2.0f * PI)*exp(-pow(x - mu, 2) / (2.0f * pow(sigma, 2)));	}
//============== Type Definiton ====================================//
class particle
{
public:
	f64_t x;
	f64_t y;
	f64_t t;
	f64_t w;
private:
	inline f64_t p_Z_hit(f64_t z, f64_t z_star){
		return gauss(z, z_star, std_hit);}
	inline u32_t p_Z_k_max(f64_t z){
		return (z == LASER_MAX);}
	inline f64_t p_Z_k_rand(f64_t z)
	{
		if(z < 5.0f)
			return 1.0f/z;
		else
			return 0;
	}
	f64_t p_Z_k_hit(f64_t z, f64_t k);
public:
	particle();
	particle(f32_t pos_x,f32_t pos_y, f32_t pos_t, f32_t weight);

	/* Beam Range Model */
	void sense(sensor_msgs::LaserScan *Z_1);
	void move(geometry_msgs::Twist *U);

};

#endif
