//teensy support 4096 data for eeprom
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include "ros/time.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/String.h"
#include<Adafruit_MCP4725.h>
Adafruit_MCP4725 dac,dacl,dacr;

#include <geometry_msgs/Twist.h> //header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h> //uint16 data
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "Kinematics.h"
#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include <Encoder.h>//#include "Encoder.h"
#define ROSSERIAL_ARDUINO_TCP
#define ROS_SERIAL_BAUD_RATE 57600

//#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
//#define DEBUG_RATE 5
 
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

ros::NodeHandle  nh;
