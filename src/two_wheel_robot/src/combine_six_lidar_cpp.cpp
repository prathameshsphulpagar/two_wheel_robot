#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <fstream>

geometry_msgs::Twist front_lidar_twist;
geometry_msgs::Twist front_bottom_twist, front_mid_twist, front_top_twist, left_twist, right_twist;
std_msgs::Int8 flag_wall_sensing_stopping;

ros::Publisher pub_combine_six; // this publisher will publish the final result.


ros::Subscriber sub_left; // subscribing to the front bottom lidar data
ros::Subscriber sub_back; // subscribing to the front bottom lidar data
ros::Subscriber sub_right; // subscribing to the front bottom lidar data
ros::Subscriber sub_front_mid; // subscribing to the front mid lidar data
ros::Subscriber sub_front_top; // subscribing to the front top lidar data
ros::Subscriber sub_front_bottom; // subscribing to the front bottom lidar data
ros::Subscriber sub_wall_sensing_stopping; // subscribing to the front bottom lidar data

void flush();
void lidar_master();
void sensing_stoping_lidar_left_callback(const geometry_msgs::Twist &msg_left);
void sensing_stoping_lidar_back_callback(const geometry_msgs::Twist &msg_back);
void sensing_stoping_lidar_right_callback(const geometry_msgs::Twist &msg_right);
void sensing_stoping_lidar_front_mid_callback(const geometry_msgs::Twist &msg_mid);
void sensing_stoping_lidar_front_top_callback(const geometry_msgs::Twist &msg_top);
void sensing_stoping_lidar_front_bottom_callback(const geometry_msgs::Twist &msg_bottom);


void flush()
{
		front_lidar_twist.linear.x = 0;
		front_lidar_twist.linear.y = 0;
		front_lidar_twist.linear.z = 0;
		
		front_lidar_twist.angular.x = 0;
}
void lidar_master()
{
	// lidar_flags.linear.x = flag_left;
	// lidar_flags.linear.y = flag_front;
	// lidar_flags.linear.z = flag_right;
	if (flag_wall_sensing_stopping.data != 1)
	{
		if ((front_bottom_twist.linear.x + front_mid_twist.linear.x + front_top_twist.linear.x + left_twist.linear.z) != 0)
		{
			front_lidar_twist.linear.x = 1;
		}
		else
		{
			front_lidar_twist.linear.x = 0;
		}
	}

	if ((front_bottom_twist.linear.y + front_mid_twist.linear.y + front_top_twist.linear.y) != 0)
	{
		front_lidar_twist.linear.y = 1;
	}
	else
	{
		front_lidar_twist.linear.y = 0;
	}

	if (flag_wall_sensing_stopping.data != 2)
	{
		if ((front_bottom_twist.linear.z + front_mid_twist.linear.z + front_top_twist.linear.z + right_twist.linear.x) != 0)
		{
			front_lidar_twist.linear.z = 1;
		}
		else
		{
			front_lidar_twist.linear.z = 0;
		}
	}
	//	front_lidar_twist.linear.x = front_bottom_twist.linear.x + front_mid_twist.linear.x + front_top_twist.linear.x + left_twist.linear.x;

	// front_lidar_twist.linear.y = front_bottom_twist.linear.y + front_mid_twist.linear.y + front_top_twist.linear.y;

	// front_lidar_twist.linear.z = front_bottom_twist.linear.z + front_mid_twist.linear.z + front_top_twist.linear.z + right_twist.linear.x;
}

void sensing_stoping_lidar_front_bottom_callback(const geometry_msgs::Twist &msg_bottom)
{
	front_bottom_twist = msg_bottom;
}

void sensing_stoping_lidar_front_mid_callback(const geometry_msgs::Twist &msg_mid)
{
	front_mid_twist = msg_mid;
}

void sensing_stoping_lidar_front_top_callback(const geometry_msgs::Twist &msg_top)
{
	front_top_twist = msg_top;
}

void sensing_stoping_lidar_left_callback(const geometry_msgs::Twist &msg_left)
{
	left_twist = msg_left;
}

void sensing_stoping_lidar_right_callback(const geometry_msgs::Twist &msg_right)
{
	right_twist = msg_right;
}

void sensing_stoping_lidar_back_callback(const geometry_msgs::Twist &msg_back)
{
	front_lidar_twist.angular.x = msg_back.angular.x;
}

void wall_sensing_stopping_callback(const std_msgs::Int8 &msg)
{
	flag_wall_sensing_stopping.data = msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "combine_six_lidar_node"); // node name
	ros::NodeHandle n;
	ROS_INFO("combine six code");
	pub_combine_six = n.advertise<geometry_msgs::Twist>("combine_six_twist_topic", 1);

	sub_front_bottom = n.subscribe("/sensing_stoping_lidar_front_bottom_topic", 1, sensing_stoping_lidar_front_bottom_callback); 

	sub_front_mid = n.subscribe("/sensing_stoping_lidar_front_mid_topic", 1, sensing_stoping_lidar_front_mid_callback);

	sub_front_top = n.subscribe("/sensing_stoping_lidar_front_top_topic", 1, sensing_stoping_lidar_front_top_callback);

	sub_left = n.subscribe("/sensing_stoping_lidar_left_topic", 1, sensing_stoping_lidar_left_callback);

	sub_right = n.subscribe("/sensing_stoping_lidar_right_topic", 1, sensing_stoping_lidar_right_callback);

	sub_back = n.subscribe("/sensing_stoping_lidar_back_topic", 1, sensing_stoping_lidar_back_callback);
	
	sub_wall_sensing_stopping = n.subscribe("/wall_sensing_stopping", 1, wall_sensing_stopping_callback);
while (ros::ok())
{
	lidar_master();
	pub_combine_six.publish(front_lidar_twist);
	flush();
	ros::spinOnce();
}
	ros::spin();
	return 0;
}
