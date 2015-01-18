#include "control_p.h"

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

	scanSubFront = nh_.subscribe<sensor_msgs::LaserScan>("/scan",10,&control::scanFront_Callback,this);

	scanSubBack = nh_.subscribe<sensor_msgs::LaserScan>("/scan_back",10,&control::scanBack_Callback,this);

    parkSpot_detected = nh_.subscribe<std_msgs::Float32>("parking_done",10,&control::parkSpot_detected_Callback,this);


	park_state = 0;

    old_average_scanBack[0]= 0;
    average_scanBack[0]= 0;
    old_average_scanFront[0]= 0;
    average_scanFront[0]= 0;


//    Fp = 10;// need to test! defult:125

//    current_ServoMsg.x = 1500;
//    current_ServoMsg.y = 1500;

//    previous_ServoMsg.x = 1500;
//    previous_ServoMsg.y = 1500;

}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers
void control::odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    odom_linearVelocity = msg->linear.x;
    odom_angularVelocity = msg->angular.z;

    odom_steeringAngle = 180/PI*atan(odom_angularVelocity/odom_linearVelocity*CAR_LENGTH);

    odom_steeringAngle = 1500 + 500/30*odom_steeringAngle;

    if(odom_steeringAngle > 2000)
    {
        odom_steeringAngle = 2000;
    }
    else if(odom_steeringAngle < 1000)
    {
        odom_steeringAngle = 1000;
    }
}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;
        //
    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }
}
// a flag method that tells us if we are controlling the car manually or automatically
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}

// Laserscanner Back
void control::scanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    double angle_middle;

    if(wii_state==1){

		// set minimum value (car to wall)
        double min_scanBack = scan->ranges[0];

        // Filter Laser data
        for(int i=0; i< scan->ranges.size(); i++){

			//discard outlayers
			if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max){
               // if(i != 0){
                    scan->ranges[i] = old_average_scanBack[i];
                            //(scan->ranges[i-1]+scan->ranges[i+1])/2;}
                //else
                //	scan->ranges[i] == scan->ranges[i+1];
			}
            //moving average_scanBack
            average_scanBack[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanBack[i];
            old_average_scanBack[i]= average_scanBack[i];

			// check closest value on the left side
			if(i < scanBack_halfData){
				//search minimum value between car and wall
				if( average_scanBack[i] < min_scanBack)
					min_scanBack = average_scanBack[i];
			}
        }
	
		// Angle in the middle of the rear sensor (average value)
        for(int i=-5; i< 6; i++){
                angle_middle += average_scanBack[scanBack_halfData + i]/11;
		}

        check_parkingState(angle_middle, min_scanBack);
    }
}

// Laserscanner Front
void control::scanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

 if(wii_state==1){
    double angle_middle;

    if(park_state >2){
		// set minimum value (car to wall)
        double min_scanFront = scan->ranges[0];

		// Filter Laser data
        for(int i=0; i< scan->ranges.size(); i++){

			//discard outlayers
			if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max){
        /*        if(scan->ranges[i] != 0){
					scan->ranges[i] == (scan->ranges[i-1]+scan->ranges[i+1])/2;}
				else
                    scan->ranges[i] == scan->ranges[i+1];*/
                scan->ranges[i] = old_average_scanBack[i-1];

			}
			//moving average_scanBack
            average_scanFront[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanFront[i];
            old_average_scanFront[i]= average_scanFront[i];

			//search minimum value between car and evironment
			if( average_scanFront[i] < min_scanFront)
				min_scanFront = average_scanFront[i];
        }

        check_parkingState(angle_middle, min_scanFront);
        }
 }

}

void control::check_parkingState(double angle_middle,double min_scan){
	
    angle_middle = angle_middle/100;

    switch(park_state){
		case 0:
        ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);

			// if car reached position one
            if((angle_middle < 85 && angle_middle > 74) || ( min_scan > 40 && min_scan < 50)){
        //		park_state = 4;
				// sleep for 1s
           //     ros::Duration(1).sleep();
                ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);
				park_state = 1;
                 wait_straight = true;
            }
                if(done->data == 1.0){
                    start_program = true;
                    ros::Duration(2).sleep();
                }
                return;
            }
            else
                wait_straight = false;
			break;
        case 1:
            if(min_scan < 40 && min_scan > 30){
           //     park_state = 4;
				// sleep for 1s
           //     ros::Duration(1).sleep();
                park_state= 2;
                ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);

                wait_straight = true;
           }
           else
               wait_straight = false;
            break;
        case 2:
            if((angle_middle < 66 && angle_middle > 57) || (min_scan < 26 && min_scan > 16)){
           //     park_state = 4;
                // sleep for 1s
          //      ros::Duration(1).sleep();
                ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);

                park_state= 3;
                wait_straight = true;
            }
            else
                wait_straight = false;
			break;
		case 3: 
            if(min_scan < 13 && min_scan > 8){
				park_state = 4;
			}
			break;
        }
    return;
}

    void control::parkSpot_detected_Callback(const std_msgs::Float32::ConstPtr& done){
        if(done->data == 2.0){
            start_program = true;
            ros::Duration(2).sleep();
        }
        return;
    }

//


//geometry_msgs::Vector3 control::P_Controller()
//{
//    current_ServoMsg.x = previous_ServoMsg.x + Fp*(cmd_linearVelocity - odom_linearVelocity);

//    current_ServoMsg.y = cmd_steeringAngle;


//    if(current_ServoMsg.x > 1580)
//    {
//        current_ServoMsg.x = 1580;
//    }
//    else if(current_ServoMsg.x < 1300)
//    {
//        current_ServoMsg.x = 1300;
//    }

//    if(current_ServoMsg.y > 2000)
//    {
//        current_ServoMsg.y = 2000;
//    }
//    else if(current_ServoMsg.y < 1000)
//    {
//        current_ServoMsg.y = 1000;
//    }

//    previous_ServoMsg = current_ServoMsg;

//    return current_ServoMsg;
//}
