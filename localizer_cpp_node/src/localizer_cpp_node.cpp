



//============== Inclusion ========================================//
/* ROS */
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/* OpenCV */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

/* C/C++ Libraries */
#include <iostream>
#include <math.h>
#include "string.h"
using namespace std;

/* In-House Libraries */
#include "types.h" 
#include "mcl_tools.h"
#include "particle.h"
//============== Symbolic Constants ================================//
#define SIMS
//#define REAL
/*============== External Data =====================================*/
vector<particle> particles;
/*============== Internal data =====================================*/
ros::Subscriber sub;  	 /* Robot sensor   */
ros::Subscriber subOdo;  /* Robot odometry   */
ros::Publisher pub;   	 /* Robot actuator */
sensor_msgs::LaserScan Z_t;  /* Robot Sensor Data Buffer */
nav_msgs::Odometry Odo_t;
geometry_msgs::Twist U_t, U_real;

ros::Time last_time; /* Time Stamp   */
ros::Duration elapsed;                  /* Time Elapsed */
/*===========Internal Function Declaration =========================*/
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
void got_od(const nav_msgs::Odometry::ConstPtr& msg);
/*============== Internal definition ===============================*/



/*
 *  Sensor Scanner
 *
 */
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  Z_t = *msg;
}


/*
 *  Odo Receiever
 *
 */
void got_odo(const nav_msgs::Odometry::ConstPtr& msg)
{
  Odo_t = *msg;
  U_t.linear.x = Odo_t.twist.twist.linear.x;
  U_t.linear.y = Odo_t.twist.twist.linear.y;
  U_t.angular.z = Odo_t.twist.twist.angular.z;
}

/*
 *  Particle Filter
 *
 */
void particle_filter(geometry_msgs::Twist *U_1, sensor_msgs::LaserScan *Z_1)
{
  
  srand(7);

  vector<f64_t>  weights_cdf; /* Weights vector */
  vector<particle>::iterator p_idx;   /* Particle index */
  geometry_msgs::Twist U_real;  /* Real motion as a function of simulation motion */

  /* Move actual robot */ 
  elapsed = ros::Time::now() - last_time;
  //cout << "time: " << ros::Time::now() << endl;
  last_time = ros::Time::now();
  //cout << "elapsed time: " << elapsed.nsec/1000000000.0f << endl;


  U_real.linear.x = U_t.linear.x*(elapsed.nsec/1000000000.0f);
  U_real.linear.y = U_t.linear.y*(elapsed.nsec/1000000000.0f);
  U_real.angular.z = U_t.angular.z*(elapsed.nsec/1000000000.0f);

  for(p_idx=particles.begin(); p_idx != particles.end(); p_idx++)
  {
    /* Move */
    p_idx->move(&U_real);
    /* Sense */
    p_idx->sense(Z_1);
  }

  /* Normalize */
  weights_cdf = norm_cdf_vector(particles);
  /* Resample */
  particles = resample(particles, weights_cdf); 

  /* Clean bad particles */
  cleanBadParticles();

}


int main(int argc, char **argv)
{
  
  u32_t p_idx;   /*Particle Index */

  /* Initialize ROS */
  ros::init(argc, argv, "localizer_cpp_node");
  ros::NodeHandle nh;
  last_time = ros::Time::now();

    /* Initialize OpenGL */
  initMclTools(argc, argv);
  mcl_run_viz();

  sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, got_scan);
  subOdo = nh.subscribe<nav_msgs::Odometry>("/odom", 1, got_odo);

	/* Create particles */
	for(p_idx=0; p_idx<NUM_PARTICLES; p_idx++)
		particles.push_back(particle());

  //Setup rate for 0.1 seconds

  while(ros::ok())
  { 
    ros::spinOnce();
    
    particle_filter(&U_real, &Z_t);
    
  }

  return 0;
}