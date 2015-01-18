/************************************************/
/* Name       : control_p.h						*/
/* Function   : declare Funcions & variables	*/
/* Author     : Tanja Weber						*/
/* Last update: 16.01.2015						*/
/************************************************/

#ifndef CONTROL_H
#define CONTROL_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float32.h"

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
	ros::Subscriber scanSubFront;
	ros::Subscriber scanSubBack;
	ros::Subscriber parkSpot_detected;

    std_msgs::Int16 control_Brake; /* flag for brake */
    std_msgs::Int16 control_Mode; /* flag for car mode: manual or autonomous */

    double cmd_linearVelocity; 
    double cmd_angularVelocity;
    double cmd_steeringAngle;

    double odom_linearVelocity;
    double odom_angularVelocity;
    double odom_steeringAngle;
	

	// own variables
    int park_state;		// parking state of the car
    int counter;		// counter variable for park state 0
    int counter_state;	// auxiliary variable to start parking process
    static int const scanBack_halfData = 260; // values on the right side of the rear sensor
    double old_average_scanBack[520];		  // array for the moving average, old value, rear sensor
    double average_scanBack[520];			  // array for the moving average, rear sensor
    double old_average_scanFront[720];		  
    double average_scanFront[520];

    bool wii_state;			// mode to start collecting sensor data	
    bool start_program;		// start to execute task

    geometry_msgs::Vector3 control_servo;

private:
    /* subscribe the cmd message from move_base */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* subscribe the virtual odom message as a feedback for controller */
    void odomCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /* check the wii states and switch the flag for manual mode and autonomous mode */
    void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

	/* subscribe to get Laserscanner Data from the Sensor in the Front */
	void scanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	/* subscribe to get Laserscanner Data from the Sensor in the Back */
	void scanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);

	/* function to check the parking state conditions and switch between the states */
    void check_parkingState(double,double,double);

	/* subscribe to get Message if parking spot have been found */
    void parkSpot_detected_Callback(const std_msgs::Float32::ConstPtr& done);

	};

#endif // CONTROL_H
