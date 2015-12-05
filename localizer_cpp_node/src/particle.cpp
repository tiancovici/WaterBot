
//================== Includes ======================================//
#include "mcl_tools.h"

/* C++ Libraries */
#include <random>
#include <math.h>
using namespace std;
//============== Internal Functions Definitions ========================//
particle::particle()
{
	static default_random_engine gen;
	static uniform_real_distribution<f64_t> xDist(0.0f, (f64_t) MAP_WIDTH);
	static uniform_real_distribution<f64_t> yDist(0.0f, (f64_t) MAP_HEIGHT);
	static uniform_real_distribution<f64_t> tDist(0.0f, (f64_t) 2.0f*PI);
	static uniform_real_distribution<f64_t> wDist(0.0f, 1.0f);

	x = xDist(gen);
	y = yDist(gen);
	t = tDist(gen);
	w = wDist(gen);
}

particle::particle(f32_t pos_x,f32_t pos_y, f32_t pos_t, f32_t weight)
{
	x = pos_x; y = pos_y; t = pos_t; w = weight;
}

f64_t particle::p_Z_k_hit(f64_t z, f64_t k)
{
	/*
		k = 0 --> angle = -90, -pi/2
	   	k = 3 --> angle = 	0,  0
	   	k = 5 --> angle =  90,-pi/2
	*/
	f64_t phi = k*(PI/4.0f) - PI/2.0f;
	return z_hit*p_Z_hit(z, map_range({x, y, t}, phi)); /*map_range({x, y, t}*/
}

/* Beam Range Model */
void particle::sense(sensor_msgs::LaserScan *Z_1)
{
	w = 1.0f;
	u32_t k = 0;
	for(vector<float>::iterator z_k = Z_1->ranges.begin(); z_k != Z_1->ranges.end(); z_k++, k++)
	{
		w *= z_hit*p_Z_k_hit(*z_k, k) + z_max*p_Z_k_max(*z_k) + z_rand*p_Z_k_rand(*z_k);
	}
}

void particle::move(geometry_msgs::Twist *U)
{
	f64_t r_new, theta_new;
	static std::default_random_engine xSeed, tSeed;
	std::normal_distribution<f64_t> xRandGauss(U->linear.x,
		sqrt(U->linear.x*U->linear.x + U->angular.z*U->angular.z)/6.0f);
	std::normal_distribution<f64_t> tRandGauss(U->angular.z,
		sqrt(U->linear.x*U->linear.x + U->angular.z*U->angular.z)/6.0f);

	r_new += xRandGauss(xSeed);
	theta_new += tRandGauss(tSeed);

	theta_new = theta_new + t;
	x = r_new*cos(theta_new+t) + x;
	y = r_new*sin(theta_new+t) + y;
	
}
