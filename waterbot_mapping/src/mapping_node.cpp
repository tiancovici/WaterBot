#include "mapping_node.hpp"
#include <math.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <limits>


mapper::mapper(const std::string &map_topic, const std::string &scan_topic, ros::NodeHandle *handle) : _map_topic(map_topic),
			   _scan_topic(scan_topic), nh(handle), _map_frame("map")//, subscribers_thread(boost::bind(&mapper::start_subscribers, this))
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
	last_pose.setIdentity();
	last_pose.getOrigin().setValue(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
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
	if (odom_pose.getOrigin().distance(last_pose.getOrigin()) >= 0.1 || odom_pose.getRotation().angle(last_pose.getRotation()) >= 0.1 ){
		last_pose = odom_pose;
		tfScalar roll, pitch, yaw;
		tf::Matrix3x3(odom_pose.getRotation()).getRPY(roll, pitch, yaw);
		tf::Point pose_point;
		tf::Point point_in_map;
		double looking_at;
		double map_res = map.info.resolution;
		int max_cell = msg->range_max / map_res;
		int i;
		double a, scan_cos, scan_sin;
		double increment;
		int array_index;
		for (i = 0, a = msg->angle_min; i < (msg->ranges).size(); i++, a += (msg->angle_increment))
		{
			looking_at = yaw + a;
			//ROS_INFO("looking at %f", looking_at );
			scan_sin = std::sin(looking_at);
			scan_cos = std::cos(looking_at);
			pose_point.setX(odom_pose.getOrigin().getX());
			pose_point.setY(odom_pose.getOrigin().getY());
			pose_point.setZ(odom_pose.getOrigin().getZ());

			for (int j = 0; j < max_cell; j++)
			{
				increment = (float)j * (map_res);
				//ROS_INFO("map_res %f, increment %f", map_res, increment);
				ros::Duration d(0.5);
				//d.sleep();
				//Flipped cos and sin, seemed to help?
				pose_point.setX(pose_point.getX() + (map_res * scan_cos));
				pose_point.setY(pose_point.getY() + (map_res * scan_sin));
				array_index = floor((pose_point.getX() - map.info.origin.position.x) / map.info.resolution) + (floor((pose_point.getY() - map.info.origin.position.y)/map.info.resolution) * map.info.width);
				//ROS_INFO("x = %f, y = %f", pose_point.getX(), pose_point.getY());
				if (map.data[array_index] < 0 || map.data[array_index] > 100)
					map.data[array_index] = 50;
				if (increment < msg->ranges[i])
				{
					//ROS_INFO("FREE");
					if (array_index >= 0 && array_index < map.data.size() )
					{
						map.data[array_index]--;
						map.data[array_index] < 0 ? 0 : map.data[array_index];
					}
					//else
					//	enlarge map, then do that
				}
				else
				{
					//ROS_INFO("OCCUPIED");
					if (array_index >= 0 && array_index < map.data.size())
					{
						map.data[array_index]++;
						map.data[array_index] > 100 ? 100 : map.data[array_index];
					}
					//else
					//	enlarge map, then do that
				}
			}
		}
		update_map();
	}
    last_scan = *msg;
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
    mapper the_mapper("/map", "/scan", &nh);
    ros::spin();
}
