/************************************************/
/* Name       : control_p.cpp					*/
/* Function   : inteprete the data of the car	*/
/* Author     : Tanja Weber						*/
/* Last update: 16.01.2015						*/
/************************************************/

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

	// initalize with the starting state
	park_state = -1;

	// disable starting the program
	start_program= false;

	// initialize arrays as empty
	for(int i=0; i<520; i++){
	old_average_scanBack[i]= 0;
	average_scanBack[i]= 0;}
	for(int i=0; i<720; i++){
	old_average_scanFront[i]= 0;
	average_scanFront[i]= 0;}

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
    
    if(wii_state==1){

        if(park_state <3){
            // set minimum value (car to wall)
            double min_scanBack = scan->ranges[0];
            double max_scanBack = scan->ranges[0];
			double angle_toWall=0;
			int j=0;

            // Filter Laser data
            for(int i=0; i< scan->ranges.size(); i++){

                //discard outlayers
                if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max){
                  //moving average_scanBack
                    average_scanBack[i] = old_average_scanBack[i];
                }
                else{
                    //moving average_scanBack
                    average_scanBack[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanBack[i];
                }
                old_average_scanBack[i]= average_scanBack[i];


                // check closest value on the left side
                if(i < scanBack_halfData){
                    //search minimum value between car and wall
                    if( average_scanBack[i] < min_scanBack && average_scanBack[i]!= 0)
                        min_scanBack = average_scanBack[i];
                    if(average_scanBack[i] > max_scanBack )
                        max_scanBack = (average_scanBack[i-7]< average_scanBack[i]-0.03 && average_scanBack[i+7] < average_scanBack[i]-0.03)? average_scanBack[i]:max_scanBack;
                }
        
			// state to correct starting position
			if(park_state == -1){
				if(i< 10 && i <20){
					if(average_scanBack[i-1]+0.09 < average_scanBack[i])
						//set any value to compare with
						angle_toWall = 200;
					}
				}
			else{
				// Angle of the rear sensor always referring to the wall (average value)
				if(i < (scanBack_halfData/2+1) && i > 105){
						//angle between 35°-45° --> 90/260= 0.346; 2,8889
						if(average_scanBack[i]!=0){
							angle_toWall += average_scanBack[i];
							j++;
						}						
				}angle_toWall = angle_toWall/j;				
			}	
			}
					
    //ROS_INFO("%f min_scan", min_scanBack);
           check_parkingState(angle_toWall, min_scanBack,max_scanBack);	
		}

	}

 }


// Laserscanner Front
void control::scanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

 if(wii_state==1){
    double angle_toWall;

	// Filter Laser data
        for(int i=0; i< scan->ranges.size(); i++){

			//discard outlayers
            if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
                //moving average_scanBack
                average_scanFront[i] = old_average_scanFront[i];
            else
                //moving average_scanBack
                average_scanFront[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanFront[i];
			old_average_scanFront[i]= average_scanFront[i];

			if(park_state >2){
				// set minimum value (car to wall)
				double min_scanFront = scan->ranges[0];
				double max_scanFront = scan->ranges[0];

				//search minimum value between car and evironment
				if(i > 360){
					if( average_scanFront[i] < min_scanFront)
						min_scanFront = average_scanFront[i];
					if(average_scanFront[i] > max_scanFront)
						max_scanFront = (average_scanFront[i-7]< average_scanFront[i]-0.03 && average_scanFront[i+7] < average_scanFront[i]-0.03)? average_scanFront[i]:max_scanFront;
				}
				check_parkingState(angle_toWall, min_scanFront, max_scanFront);
			}        
        }
 }

}

void control::check_parkingState(double angle_toWall,double min_scan, double max_scan){
	ROS_INFO("%f: max_scan, %f min_scan", max_scan, min_scan);

    switch(park_state){
		case -1:
			if(angle_toWall == 200)
				park_state = 0;
			break;
		case 0:
			// if car reached position one
			if((angle_toWall < 0.805 || min_scan < 0.50) && max_scan < 0.7)
				park_state = 1;				
			break;
        case 1:
            if(min_scan < 0.40  && max_scan < 0.6)
				park_state= 2;
		//      ROS_INFO("%f: angle middle, %f min_scan", angle_toWall, min_scan);
            break;
        case 2:
                 if(max_scan < 0.3 || min_scan < 0.12 ){

    //        ROS_INFO("%f: angle middle, %f min_scan", angle_toWall, min_scan);
				park_state= 3;
            }
			break;
		case 3: 
            if(min_scan < 0.13  || max_scan < 0.31){
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