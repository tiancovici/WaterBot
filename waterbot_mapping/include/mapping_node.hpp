#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <string>

class mapper
{
public:
	mapper(const std::string &, const std::string &, const std::string &, ros::NodeHandle*);
	~mapper();
	void update_map();
private:
	std::string _map_topic;
	std::string _scan_topic;
	std::string _pose_topic;
	ros::NodeHandle *nh;
	boost::thread subscribers_thread;
	nav_msgs::Odometry ground_truth;
	nav_msgs::OccupancyGrid map;
	sensor_msgs::LaserScan last_scan;
	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;
	ros::Publisher map_pub;
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
	void pose_callback(const nav_msgs::Odometry::ConstPtr&);
	void start_subscribers();
};
