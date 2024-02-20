
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdbool.h>

// function termios
int kbhit(void)
{
  static bool initflag = false;
  static const int STDIN = 0;

  if (!initflag)
  {
    // Use termios to turn off line buffering
    struct termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initflag = true;
  }

  int nbbytes;
  ioctl(STDIN, FIONREAD, &nbbytes);
  return nbbytes;
}

// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_restart_node");
  ros::NodeHandle n;

  // loop
 // while (!kbhit())
  //{
    ROS_WARN_STREAM("Killed node is restarting");

    system("rosrun rosserial_python serial_node.py /dev/ttyACM0"); // connect teensy
    
    //   system("roslaunch three_wheel_robot three_wheel_robot_master_launch.launch");

    // =====================================================================
    //system("rosrun rplidar_ros rplidarNode"); //node lidar (main) launcher

   //system("rosrun three_wheel_robot lidar_client1_executable");//node lidar (main) launcher
  // system("rosrun three_wheel_robot lidar_client_front_mid_executable");//node lidar (main) launcher
 //  system("rosrun rplidar_ros_front_mid rplidarNode_front_mid_executabl"); //node lidar

    // system("rosrun rplidar_ros_front_top rplidarNode_front_top_executable"); //node lidar

    //	system("rosrun rplidar_ros_left rplidarNode_left_executable"); //node lidar

    //	system("rosrun rplidar_ros_right rplidarNode_right_executable"); //node lidar

    //	system("rosrun rplidar_ros_back rplidarNode_back_executable"); //node lidar*/

    // system("roslaunch three_wheel_robot three_wheel_robot_launch.launch "); //node lidar (main) launcher
 // }
}
// end
