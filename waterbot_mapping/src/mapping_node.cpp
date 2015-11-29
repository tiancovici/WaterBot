#include "mapping_node.hpp"

mapper::mapper(const std::string &map_topic, const std::string &scan_topic, const std::string &pose_topic, ros::NodeHandle *handle) : _map_topic(map_topic),
			   _scan_topic(scan_topic), _pose_topic(pose_topic), nh(handle), subscribers_thread(boost::bind(&mapper::start_subscribers, this))
{
	map_pub = nh->advertise<nav_msgs::OccupancyGrid>(_map_topic, 1, true);
}

mapper::~mapper()
{
	subscribers_thread.join();
}

void mapper::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	last_scan = *msg;
}

void mapper::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	ground_truth = *msg;
}

void mapper::start_subscribers()
{
	laser_sub = nh->subscribe(_scan_topic, 1, &mapper::laser_callback, this);
	pose_sub = nh->subscribe(_pose_topic, 1, &mapper::pose_callback, this);
	ros::spin();
}

void mapper::update_map()
{
	ROS_INFO("Updating map!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    mapper the_mapper("/map", "/scan", "/stage/base_pose_ground_truth", &nh);
    ros::Rate rate(1);
    while(ros::ok())
    {
    	the_mapper.update_map();
    	rate.sleep();
    }
}
