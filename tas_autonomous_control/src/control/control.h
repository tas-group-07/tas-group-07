// Andreas Pfeuffer & Fabian Sonntag

#ifndef CONTROL_H
#define CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
//change Fabian
#include "sensor_msgs/LaserScan.h"

#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

class control
{
public:
    control();

    ros::NodeHandle nh_;
    ros::Publisher control_servo_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber wii_communication_sub;
	ros::Subscriber enable_driving_sub;
    ros::Subscriber sensor_sub;

    std_msgs::Int16 control_Brake; /* flag for brake */
    std_msgs::Int16 control_Mode; /* flag for car mode: manual or autonomous */

    double cmd_linearVelocity;
    double cmd_angularVelocity;
    double cmd_steeringAngle; 
	// added by Andreas
	double enable_driving;

    //changes Fabian
    double velocity_value;
    double initialization_counter;
    double velocity_value_old[20];
    //changes Fabian Ende

    double odom_linearVelocity;
    double odom_angularVelocity;
    double odom_steeringAngle;

    geometry_msgs::Vector3 control_servo;

private:
    /* subscribe the cmd message from move_base */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

	// added by Andreas
	void enable_drivingCallback(const std_msgs::Float32::ConstPtr& msg);

    /* subscribe the virtual odom message as a feedback for controller */
    void odomCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* check the wii states and switch the flag for manual mode and autonomous mode */
    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

    //Code Fabian: Check LaserRanges and compute maximum Velocity
    void LaserScan_Velocity_Calculation(const sensor_msgs::LaserScan::ConstPtr& msg);

};

#endif // CONTROL_H
