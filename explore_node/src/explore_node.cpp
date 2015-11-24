



//============== Inclusion ========================================//
/* ROS */
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "create_node/TurtlebotSensorState.h"
#include "sensor_msgs/Joy.h"
#include "tf/transform_datatypes.h"
/* OpenCV */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

/* C/C++ Libraries */
#include <iostream>
#include <math.h>
#include <vector>
#include "string.h"
using namespace std;

/* Tom's Libraries */
#include "types.h" 
//============== Symbolic Constants ================================//
#define SIM

#define STRAIGHT     0
#define LEFT 		90
#define RIGHT 	   -90

#define MAX_ANGLE 240

#define PRESSED 1
/*===========Internal Function Declaration =========================*/
/* Callback functions */
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
void got_odo(const nav_msgs::Odometry::ConstPtr& msg);
void got_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void got_create_sensor(const create_node::TurtlebotSensorState::ConstPtr& msg );
/* Explore strategy function */
void policy(geometry_msgs::Twist *U, sensor_msgs::LaserScan *Z, nav_msgs::Odometry *Odo);
void bumpPolicy(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo);

/* Move functions */
void moveCW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s);
void moveCCW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s);
void moveFW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s);
void moveBW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s);

/* Math Functions */
u32_t SensorAngleIdx(sensor_msgs::LaserScan *Z, u32_t angle);
f32_t AvgRange(f32_t minTheta, f32_t maxTheta, sensor_msgs::LaserScan *Z);
f32_t AveBeam(f32_t theta, sensor_msgs::LaserScan *Z);


/*============== External Data =====================================*/
/*============== Internal data =====================================*/

/* Subscribers/Publishers */
ros::Subscriber sub;  	 /* Robot sensor     */
ros::Subscriber subOdo;  /* Robot odometry   */
ros::Subscriber joy_sub;  /* Xbox Controller  */
ros::Subscriber subCreate;
ros::Publisher pub;   	 /* Robot actuator   */

/* Global data via topics  */
sensor_msgs::LaserScan Z_t;  	/* Robot Sensor Data Buffer */
nav_msgs::Odometry Odo_g;		/* Robot Odometry */
nav_msgs::OccupancyGrid Map_g;	/* Most confident Map */
create_node::TurtlebotSensorState CreateSensor_g;

ros::Time last_time; 			/* Time Stamp   */
ros::Duration elapsed;          /* Time Elapsed */

/* Configuraiton */
bool autoExploreMode;
/*===========Internal Function Definition ==========================*/

/*
 *  Get Index Intesity of a 240 degree angle sensor
 *
 */
u32_t SensorAngleIdx(sensor_msgs::LaserScan *Z, u32_t degrees)
{
	u32_t idx;
	const f32_t theta2idx = ((f32_t)Z->ranges.size())/((f32_t)MAX_ANGLE);
	const u32_t zeroIdx = (u32_t)(theta2idx* 120.0f);
	/* 			 Straight
				   120
	
		
	left  210              30   Right

				240		0
	*/
	if(degrees == STRAIGHT)
		idx =  zeroIdx;
	else if(degrees == LEFT)
		idx = zeroIdx + (u32_t)theta2idx*90.0f;
	else if(degrees == RIGHT)
		idx = zeroIdx + (u32_t)(theta2idx * -90.0f);
	else
		idx = zeroIdx + (u32_t)theta2idx*degrees;

	return idx;
}
/*  Name: AveRange
 *
 *  Purpose: Integrate the laser magnitued across a range of degrees, and average it
 *	 Return Average value
 */
f32_t AvgRange(f32_t minTheta, f32_t maxTheta, sensor_msgs::LaserScan *Z)
{
	u32_t i;
	u32_t init_idx = SensorAngleIdx(Z, minTheta);
	u32_t end_idx = SensorAngleIdx(Z, maxTheta);

	f32_t sum = 0;

	for(i=init_idx, sum=0; i<end_idx; i++)
	{
		sum += Z->ranges[i];
	}
	return sum/(end_idx - init_idx);
}
/*  Name: AveBeam
 *
 *  Purpose: Integrate the laser magnitued across a small range of the degree of interest
 *			 to reduce noise when looking at a specific angle
 *	 Return Average value
 */
f32_t AveBeam(f32_t theta, sensor_msgs::LaserScan *Z)
{
	u32_t i;
	u32_t init_idx = SensorAngleIdx(Z, theta-1);
	u32_t end_idx = SensorAngleIdx(Z, theta+1);

	f32_t sum = 0;

	for(i=init_idx, sum=0; i<end_idx; i++)
	{
		sum += Z->ranges[i];
	}
	return sum/(end_idx - init_idx);
}

/*  Name: moveCW
 *
 *  Purpose: To move clockwise
 *	 Input 		speed_m_s, speed (rad/s)
 *				U, pointer to a twist data structure
 *				Odo, pointer to odometry data
 *			  
 */
void moveCW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s)
{
	f32_t vel = -speed_m_s;

	U->linear.x = 0.0f;
	U->linear.y = 0.0f;
	U->angular.z = vel;

	pub.publish(*U);
}
/*  Name: moveFW
 *
 *  Purpose: To move forward
 *	 Input  	 speed_m_s, speed (rad/s)
 *				 U, pointer to a twist data structure
 *				 Odo, pointer to odometry data
 *			  
 */
void moveFW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s)
{
	f32_t vel = speed_m_s;

	U->linear.x = vel;
	U->linear.y = 0.0f;
	U->angular.z = 0.0f;

	pub.publish(*U);
}
/*  Name: moveFW
 *
 *  Purpose: To move backward
 *	 Input  	 speed_m_s, speed (rad/s)
 *				 U, pointer to a twist data structure
 *				 Odo, pointer to odometry data
 *			  
 */
void moveBW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s)
{
	f32_t vel = -speed_m_s;

	U->linear.x = vel;
	U->linear.y = 0.0f;
	U->angular.z = 0.0f;

	pub.publish(*U);
}

/*  Name: moveCCW
 *
 *  Purpose: To move counter clockwise
 *	 Input		speed_m_s, speed (rad/s)
 *				U, pointer to a twist data structure
 *				Odo, pointer to odometry data
 *			  
 */
void moveCCW(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo, f32_t speed_m_s)
{
	f32_t vel = speed_m_s;

	U->linear.x = 0.0f;
	U->linear.y = 0.0f;
	U->angular.z = vel;

	pub.publish(*U);
}
/*  Name: ExploreLeft
 *
 *  Purpose: To determine whether it's wroth exploring left
 *	 Return 1, Yes
 *			0, No
 */
inline bool ExploreLeft(sensor_msgs::LaserScan *Z)
{
	bool expLeft = false;
	expLeft = AveBeam(45.0f, Z) > 4.0f;
	return expLeft;
}
/*  Name: ForwardWall
 *
 *  Purpose: To check if there's a wall infront within vicinity of -20 to 20 degrees.
 *	 Return 1, Yes
 *			0, No
 */
inline bool ForwardWall(sensor_msgs::LaserScan *Z)
{
	return (AvgRange(-15.0f, 15.0f, Z) < 2.0f)		/* Forward Wall */
		|| (AvgRange( 30.0f, 32.0f, Z) < 2.0f)		/* Diagnoal obstacles */ 
		|| (AvgRange( -32.0f, -30.0f, Z) < 2.0f);
}
/*  Name: bumpPolicy
 *
 *  Purpose: To move in a certain sequence when bumping into an object
 *	 Input		U, pointer to a twist data structure
 *				Odo, pointer to odometry data
 *			  
 */
void bumpPolicy(geometry_msgs::Twist *U, nav_msgs::Odometry *Odo)
{
	moveBW(U, Odo, 0.5f);
	moveBW(U, Odo, 0.5f);
	moveBW(U, Odo, 0.5f);
	moveCW(U, Odo, 0.5f);
	moveCW(U, Odo, 0.5f);
	moveCW(U, Odo, 0.5f);
	moveCW(U, Odo, 0.5f);
}
/*
 *  Exploration Policy
 *
 */
void policy(geometry_msgs::Twist *U, sensor_msgs::LaserScan *Z, nav_msgs::Odometry *Odo)
{
	f64_t roll, pitch, yaw;
	
	/* Case we have no data, do nothing */
	if(Z->ranges.size() == 0)
		return;

/* No bumper in simulation */
#if defined(SIM)

	/* Turn right if there's a wall infront */
	if(ForwardWall(Z))
		moveCW(U, Odo, 2.0f);
	else if(ExploreLeft(Z))
		moveCCW(U, Odo, 4.0f);
	/* When there are no obstacles infront, move forward */
	else
		moveFW(U, Odo, 4.0f);

		tf::Quaternion q(Odo->pose.pose.orientation.x,
						Odo->pose.pose.orientation.y,
						Odo->pose.pose.orientation.z,
						Odo->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		//cout << " yaw: " << yaw/PI*180.0f << endl;
		//cin.get();
#else
	if(CreateSensor_g.bumps_wheeldrops)
	{
		bumpPolicy(U, Odo);
	}
	/* Turn right if there's a wall infront */
	if(ForwardWall(Z))
		moveCW(U, Odo, 0.1f);
	else if(ExploreLeft(Z))
		moveCCW(U, Odo, 0.1f);
	/* When there are no obstacles infront, move forward */
	else
		moveFW(U, Odo, 0.5f);
#endif
	/* For debugging purpose, wait for enter key */

}
/*
 *  Sensor Scanner
 */
void got_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  Z_t = *msg;
}
/*
 *  Odo Receiver
 */
void got_odo(const nav_msgs::Odometry::ConstPtr& msg)
{
  Odo_g = *msg;
}
/*
 *  Map Receiver
 */
void got_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  Map_g = *msg;
}
/*
 *  Create Sensor Receiver
 */
void got_create_sensor(const create_node::TurtlebotSensorState::ConstPtr& msg )
{
	CreateSensor_g = *msg;
}
/*
 *  Joystick Receiver
 */
void joyCallback (const sensor_msgs::Joy::ConstPtr& msg)
{
   /*  		/joy.buttons
    *  Idx 			Button Name
	*	0				A
	*	1				B
	*	2				X
	*	3				Y
	*	4				LB
	*	5				RB
	*	6				Back				(BACK)
	*	7				Start 				(START)
	*	8				Power				(POWER)
	*	9				Button Stick Left 	(LEFT)
	*	10				Button Stick Right 	(RIGHT)
	*/

	if(msg->buttons[RB] == PRESSED)
	{
		autoExploreMode = true;
		cout << "Button RB: detected, Mode: "<<  autoExploreMode << endl;
	}
	else if(msg->buttons[LB] == PRESSED)
	{
		autoExploreMode = false;
		cout << "Button RB: detected, Mode: "<<  autoExploreMode << endl;
	}
}


int main(int argc, char **argv)
{
  
  u32_t p_idx;   /*Particle Index */
  geometry_msgs::Twist U;     /* Motion */

  /* Initialize ROS */
  ros::init(argc, argv, "explore_node");
  ros::NodeHandle nh;
  last_time = ros::Time::now();
#if defined(SIM)
  
  pub = nh.advertise<geometry_msgs::Twist>("/stage/cmd_vel", 1);
  
  sub = nh.subscribe<sensor_msgs::LaserScan>("/stage/base_scan", 1, got_scan);

  subOdo = nh.subscribe<nav_msgs::Odometry>("/stage/odom", 1, got_odo);

  /* Enable Autonmous Wonder Mode on Start */
  autoExploreMode = true;
#else

  /* Whatever the Create 2 movement is */
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  /* Whatever the Hokoyu LIDAR scan is */
  sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, got_scan);

  subCreate = nh.subscribe("/turtlebot_node/sensor_state", 1, got_create_sensor);

  /* Whatever the Create 2 odometry is
  subOdo = nh.subscribe<nav_msgs::Odometry>("/stage/odom", 1, got_odo); */
  /* Listen to joystick joy topic */
  joy_sub = nh.subscribe("joy", 1000, joyCallback);

  /* Disable Autonmous Wonder Mode on Start */
  autoExploreMode = false;
#endif
  /* Initialize OpenGL */
  //initMclTools(argc, argv);
  //mcl_run_viz();

  //Setup rate for 0.1 seconds
  //ros::Rate r(100); // 100 hz

  while(ros::ok())
  { 
    ros::spinOnce();
    
    if(autoExploreMode)
    {
    	/* Autonmous Wonder mode */
    	policy(&U, &Z_t, &Odo_g);
    }
    else
    {
    	/* Only respond to Xbox controller */
    }
   //r.sleep();
  }

  return 0;
}