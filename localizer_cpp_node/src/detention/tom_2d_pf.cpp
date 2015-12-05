



//============== Inclusion ========================================//

/* OpenCV */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

/* C/C++ Libraries */
#include <iostream>
#include <math.h>
#include "string.h"
using namespace std;

/* Tom's Libraries */
#include "mcl_tools.h"
#include "particle.h"
#include "util.h"



/*============== External Data =====================================*/
vector<particle> particles;
/*============== Internal data =====================================*/
ros::Subscriber sub;  /* Robot sensor   */
ros::Publisher pub;   /* Robot actuator */
sensor_msgs::LaserScan Zsense;  /* Robot Sensor Data Buffer */

ros::Time last_time; /* Time Stamp   */
ros::Duration elapsed;                  /* Time Elapsed */
/*===========Internal Function Declaration =========================*/
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
void particle_filter(geometry_msgs::Twist *U_1, sensor_msgs::LaserScan *Z_1);
void policy(geometry_msgs::Twist *U, sensor_msgs::LaserScan *Z);
void moveForward(geometry_msgs::Twist *U);
void moveCCW(geometry_msgs::Twist *U);
/*============== Internal definition ===============================*/
int main(int argc, char **argv)
{
  
  u32_t p_idx;   /*Particle Index */
  geometry_msgs::Twist U;     /* Motion */

  /* Initialize ROS */
  ros::init(argc, argv, "tom_2d_pf");
  ros::NodeHandle nh;
  last_time = ros::Time::now();
  pub = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
  sub = nh.subscribe<sensor_msgs::LaserScan>("/robot/base_scan", 1, got_scan);

  /* Initialize OpenGL */
  initMclTools(argc, argv);
  mcl_run_viz();

  //Setup rate for 0.1 seconds
  ros::Rate r(5); // 100 hz

  /* Create particles */
  for(p_idx=0; p_idx<NUM_PARTICLES; p_idx++)
    particles.push_back(particle());

  while(ros::ok())
  { 
    ros::spinOnce();
    
    policy(&U, &Zsense);
    particle_filter(&U, &Zsense);
    
    r.sleep();
  }

  return 0;
}


/*
 *  Sensor Scanner
 *
 */
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  Zsense = *msg;
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

  U_real.linear.x = U_1->linear.x*(elapsed.nsec/1000000000.0f);
  U_real.linear.y = U_1->linear.y*(elapsed.nsec/1000000000.0f);
  U_real.angular.z = U_1->angular.z*(elapsed.nsec/1000000000.0f);


  //cout << "x :" << U_real.linear.x << endl;
  //cout << "z :" << U_real.angular.z << endl;

  pub.publish(*U_1);

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