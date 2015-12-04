#include "mapping_node.hpp"
#include <math.h>
#include <boost/foreach.hpp>
#include <iostream>

int mapper::poseToCell(double x, double y)
{
	return static_cast<int>(floor((x- map.info.origin.position.x)/map.info.resolution) +
							(floor(-(map.info.origin.position.y - y)/map.info.resolution) * map.info.width));
}

void mapper::cellToPose(int cell, double &x, double &y)
{

}

std::vector<int> mapper::cellsInView(double x, double y, double theta)
{

}

int mapper::rayHittingCell(double x, double y, double theta, int cell)
{
}

mapper::mapper(const std::string &map_topic, const std::string &scan_topic, const std::string &pose_topic, ros::NodeHandle *handle) : _map_topic(map_topic),
			   _scan_topic(scan_topic), _pose_topic(pose_topic), nh(handle), _map_frame("map")//, subscribers_thread(boost::bind(&mapper::start_subscribers, this))
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
	tf::poseMsgToTF (map.info.origin, worldToMap);
	mapToWorld = worldToMap.inverse();
	//std::cout<<"world to map: "<<worldToMap << '\n';
	//std::cout<<"map to world: "<<mapToWorld << '\n';
	update_map();
}

mapper::~mapper()
{
	//subscribers_thread.join();
}

void mapper::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if (!tf_.waitForTransform(_map_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0)))
	    {
	      ROS_WARN_STREAM ("Timed out waiting for transform from map to scan");
	      return;
	    }
    tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
    tf::StampedTransform odom_pose;
    tf_.lookupTransform(_map_frame, msg->header.frame_id, ros::Time(0), odom_pose);
    tfScalar roll, pitch, yaw;
    tf::Matrix3x3(odom_pose.getRotation()).getRPY(roll, pitch, yaw);
    tf::Point pose_point;
    tf::Point point_in_map;
    double looking_at;
    double map_res = map.info.resolution;
    int max_cell = msg->range_max / map_res;
    int i;
    double a, cos, sin;
    double increment;
    int index;
    for (i = 0, a = msg->angle_min; i < (msg->ranges).size(); i++, a += msg->angle_increment)
    {
    	looking_at = yaw + a;
    	//ROS_INFO("looking at %f", looking_at );
    	sin = std::sin(looking_at);
    	cos = std::cos(looking_at);
    	pose_point.setX(0);//odom_pose.getOrigin().getX());
    	pose_point.setY(0);//odom_pose.getOrigin().getY());
    	pose_point.setZ(odom_pose.getOrigin().getZ());
    	for (int j = 0; j < max_cell; j++)
    	{
    		increment = j*map_res;
    		pose_point.setX(pose_point.getX() + (increment * sin));
    		pose_point.setY(pose_point.getY() + (increment * cos));
			index = floor((pose_point.getX() - map.info.origin.position.x) / map.info.resolution) + (floor((pose_point.getY() - map.info.origin.position.y)/map.info.resolution) * map.info.width);
			//ROS_INFO("index = %d", index);
			if (increment < msg->ranges[i])
    		{
				//ROS_INFO("FREE");
    			//ROS_INFO("FREE CELL AT INDEX %d", occupancy_grid_utils::pointIndex(map.info, pose_point));
    			//map.data[occupancy_grid_utils::pointIndex(map.info, pose_point)] = 0;
				if (index >= 0 && index < map.data.size())
					map.data[index] = 0;
				//else
				//	ROS_INFO("out of bounds, size: %lu, index: %d", map.data.size(), index);
    		}
    		else
    		{
    			//ROS_INFO("OCCUPIED");
    			//ROS_INFO("OCC CELL AT INDEX %d", occupancy_grid_utils::pointIndex(map.info, pose_point));
    			if (index >= 0 && index < map.data.size())
    				map.data[index] = 100;
    			//else
    			//	ROS_INFO("out of bounds, size: %lu, index: %d", map.data.size(), index);
    		}
    	}
    }
    last_scan = *msg;
    update_map();
}

void mapper::start_subscribers()
{
	laser_sub = nh->subscribe(_scan_topic, 100, &mapper::laser_callback, this);
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
