#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include "std_msgs/String.h"
int count_round = 0 ;
std_msgs::Int32 encoder_value, encoder_possition_twist;

int one_rotation_distance_coverd = 628;//MM  // dimention are 200 mm , circumfarance = 3.14 * 200;

void identify_direction();
void normal_encoder_position_giving_callback(const geometry_msgs::Twist& msg);

void normal_encoder_position_giving_callback(const geometry_msgs::Twist& msg)
{
}
void normal_encoder_callback(const std_msgs::Int32& msg) // this is callback function taking geometry msgs 
{	
	encoder_possition_twist = msg; //storing from local veriable to globle veriable
	
	/*ROS_INFO(":linear-: ");
	ROS_INFO(":X- value: %f", msg_twist.linear.x);
	ROS_INFO(":Y-: %f", msg_twist.linear.y);
	ROS_INFO(":Z-: %f", msg_twist.linear.z);

    ROS_INFO(":angular-: ");
	ROS_INFO(":X- RPM: %f", msg_twist.angular.x);
	ROS_INFO(":Y- MPS: %f", msg_twist.angular.y);
	ROS_INFO(":Z- KMPH: %f", msg_twist.angular.z);*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_encoder_node"); // declaring encoder node
    ros::NodeHandle n;
    
    ROS_INFO("normal enoder code");
 
	ros::Publisher pub_normal_encoder_position = n.advertise <geometry_msgs::Twist> ("normal_encoder_positioon_topic", 1);

  ros::Subscriber sub_absolute_encoder_angle = n.subscribe("/updated_mov_cmd_topic", 1, normal_encoder_position_giving_callback); // it ****update name into this*********

	ros::Subscriber sub_normal_encoder = n.subscribe("/normal_encoder_position_topic", 1, normal_encoder_callback); // it is subscribing to the encoder publisher which is on teensy side. this is in geometry format
    
    ros::spin();
    return 0;
}
