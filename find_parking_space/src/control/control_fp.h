#ifndef CONTROL_H
#define CONTROL_H
#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;                              /* for newline command when writing into file */

// include Messages
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define PI             3.14159265

class control_fp
{
public:
  control_fp();
  
  ros::NodeHandle nh_;
  ros::Publisher control_servo_pub_;              /* to publish servo commands */
  ros::Publisher start_parking_task_;             /* to communicate with parking node */
  ros::Subscriber front_laser_sub_;               /* to get data from front laser scanner */
  ros::Subscriber back_laser_sub_;                /* to get data from back laser scanner */
  ros::Subscriber robot_position_sub_;            /* to get actual robot position */
  ros::Subscriber wii_communication_sub;          /* to receive wii inputs */
  
  int find_min;                                   /* flag whether to search for local minimum */
  
  double val_front[3];                            /* last three mean values from front laser scanner */
  double mean_val_front;                          /* current sum of the five most left values from front laser scanner */
  double scan_data[90];                           /* smoothed data from front laser scanner */
  
  double val_back[3];                             /* last three mean values from back laser scanner */
  double mean_val_back;                           /* current sum of the five most left values from back laser scanner */
  
  int min;                                        /* local minimum angle */
  //double initial_distance;                        /* start distance to check if the car is driving straight ahead */
  double rob_pos_y;                               /* current y position of the robot */
  
  geometry_msgs::Vector3 control_servo;           /* message for servo commands */
  std_msgs::Float32 start_parking_task;           /* massage for communication with parking node */
  
  std_msgs::Int16 control_Brake;                  /* flag for brake */
  std_msgs::Int16 control_Mode;                   /* flag for car mode: manual or autonomous */
  
private:
  /* subscribe the sensor message from front laser scanner */
  void front_laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  
  /* subscribe the sensor message from back laser scanner */
  void back_laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  
  /* subscribe the robot pose massage */
  void robot_positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  
  /* check the wii states and switch the flag for manual mode and autonomous mode */
  void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
};
#endif
