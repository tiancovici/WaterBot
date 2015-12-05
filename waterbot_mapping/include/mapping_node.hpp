#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class mapper
{
public:
	mapper(const std::string &, const std::string &, ros::NodeHandle*);
	~mapper();
	void update_map();
	tf::TransformListener tf_;
	tf::Transform worldToMap;
	tf::Transform mapToWorld;
private:
	int width;
	int height;
	double resolution;
	std::string _map_frame;
	std::string _map_topic;
	std::string _scan_topic;
	ros::NodeHandle *nh;
	//boost::thread subscribers_thread;
	nav_msgs::OccupancyGrid map;
	sensor_msgs::LaserScan last_scan;
	ros::Subscriber laser_sub;
	ros::Publisher map_pub;
	tf::StampedTransform last_pose;
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
	void enlarge();
	void start_subscribers();
};
