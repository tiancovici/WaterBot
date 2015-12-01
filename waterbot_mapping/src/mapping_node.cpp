#include "mapping_node.hpp"
#include <math.h>

int mapper::poseToCell(double x, double y)
{
	return static_cast<int>(floor((x- map.info.origin.position.x)/map.info.resolution) +
							(floor(-(map.info.origin.position.y - y)/map.info.resolution) * map.info.width));
}

mapper::mapper(const std::string &map_topic, const std::string &scan_topic, const std::string &pose_topic, ros::NodeHandle *handle) : _map_topic(map_topic),
			   _scan_topic(scan_topic), _pose_topic(pose_topic), nh(handle)//, subscribers_thread(boost::bind(&mapper::start_subscribers, this))
{
	start_subscribers();
	map_pub = nh->advertise<nav_msgs::OccupancyGrid>(_map_topic, 1, true);
	height = 1000;
	width = 1000;
	resolution = 0.1;
	map.header.frame_id = "map";
	map.info.height = height;
	map.info.width = width;
	map.info.resolution = resolution;
	map.info.origin.position.x = -(width * resolution) / 2;
	map.info.origin.position.y = -(height * resolution) / 2;
	for (int i = 0; i < height*width; i++)
		map.data.push_back(-1);
}

mapper::~mapper()
{
	//subscribers_thread.join();
}

void mapper::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	last_scan = *msg;
}

void mapper::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	ground_truth = *msg;
	update_map();
}

void mapper::start_subscribers()
{
	laser_sub = nh->subscribe(_scan_topic, 100, &mapper::laser_callback, this);
	pose_sub = nh->subscribe(_pose_topic, 100, &mapper::pose_callback, this);
}

void mapper::update_map()
{
	map_pub.publish(map);
}

void mapper::enlarge()
{
	//TODO: duplicate? map size
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    mapper the_mapper("/map", "/scan", "/stage/base_pose_ground_truth", &nh);
    ros::spin();
    /*ros::Rate rate(1);
    while(ros::ok())
    {
    	the_mapper.update_map();
    	rate.sleep();
    }*/
}
