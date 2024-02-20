#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include "std_msgs/String.h"

void absolute_encoder_angle_giving_callback(const geometry_msgs::Twist::ConstPtr& msg);

void absolute_encoder_angle_giving_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
}
int main(int argc, char **argv)

{
    ros::init(argc, argv, "absolute_encoder_node"); // declaring encoder node
    ros::NodeHandle n;
    
    ROS_INFO("absolute enoder code");

ros::Publisher pub_absolute_encoder_angle = n.advertise<geometry_msgs::Twist> ("absolute_encoder_angle_topic", 1000);
    
ros::Subscriber sub_absolute_encoder_angle = n.subscribe<geometry_msgs::Twist>("/absolute_encoder_angle_giving_topic", 1000, absolute_encoder_angle_giving_callback); // it
    ros::spin();
    return 0;
}
