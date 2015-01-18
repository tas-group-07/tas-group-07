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
    park_state = 0;

	// set counter to zero
    counter_state=0;

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

//Subscribe to the Laserscanner Back
void control::scanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    
    if(wii_state==1){

        if(park_state <3){
            // set min and max variable of range values
            double min_scanBack = scan->ranges[0];
            double max_scanBack = scan->ranges[0];
			double angle_middle;

            // Filter Laser data
            for(int i=0; i< scan->ranges.size(); i++){

                //discard outlayers
                if((scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)){
					// use old range value
                    average_scanBack[i] = old_average_scanBack[i];
                }
                else{
                    //moving average_scanBack
                    average_scanBack[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanBack[i];
                }
				// store the new range average values
                old_average_scanBack[i]= average_scanBack[i];


                // check closest value on the left side
                if(i < scanBack_halfData){
                    // search minimum range value between car and wall
                    if( average_scanBack[i] < min_scanBack && average_scanBack[i]!= 0)
                        min_scanBack = average_scanBack[i];
					//search maximum range value
                    if(average_scanBack[i] > max_scanBack )
						// use old value if measurement is not pointing into the parking spot
                        max_scanBack = (average_scanBack[i-7]< average_scanBack[i]-0.03 && average_scanBack[i+7] < average_scanBack[i]-0.03)? average_scanBack[i]:max_scanBack;
                }
            }

            // Angle in the middle of the rear sensor (average value)
            for(int i=-5; i< 6; i++){
                    angle_middle += average_scanBack[scanBack_halfData + i]/11;
            }
			//ROS_INFO("%f min_scan", min_scanBack);

			//call function to switch between the parking states (transfer range data)
            check_parkingState(angle_middle, min_scanBack,max_scanBack);
		}
    }
}

// Subscribe to the Laserscanner Front
void control::scanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

	if(wii_state==1){
    
        // set minimum value (car to wall)
        double min_scanFront = scan->ranges[0];
        double max_scanFront = scan->ranges[0];
		double angle_middle;

        // Filter Laser data
        for(int i=0; i< scan->ranges.size(); i++){

            //discard outlayers
            if(scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
                  // use old range value
                  average_scanFront[i] = old_average_scanFront[i];
            else
                  // build the moving average
                  average_scanFront[i] = 0.75 * scan->ranges[i] + 0.25 * old_average_scanFront[i];
			// store the new range average values
            old_average_scanFront[i]= average_scanFront[i];


			// enable the sensor data to influence the parking behaviour
			if(park_state >2){
				//search minimum and maximum value between car and evironment
				if(i > 360){
					if( average_scanFront[i] < min_scanFront)
						min_scanFront = average_scanFront[i];
					if(average_scanFront[i] > max_scanFront)
						max_scanFront = (average_scanFront[i-7]< average_scanFront[i]-0.03 && average_scanFront[i+7] < average_scanFront[i]-0.03)? average_scanFront[i]:max_scanFront;
				}
			// call function and transfer range data
			check_parkingState(angle_middle, min_scanFront, max_scanFront);
			}
        }
	}
}

// Function to switch between the parking states
void control::check_parkingState(double angle_middle,double min_scan, double max_scan){

	// switch between the different parking states using distances provided by the laser scanner
    switch(park_state){

        case 0:
			// ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);

			// first drive backwards 
			counter_state++;

			// if car reached position one
		    /*      if(((angle_middle < 0.82 && angle_middle > 0.77) || ( min_scan > 0.40 && min_scan < 0.50)) && ( max_scan < 0.7 && max_scan > 0.66)){ */
			if(((angle_middle < 0.82 || min_scan < 0.50) && max_scan < 0.7) && counter_state>45){
				/* park_state = 4;
                   // sleep for 1s
				   ros::Duration(1).sleep();*/
                ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);
                park_state = 1;
                 wait_straight = true;
            }
            else
                wait_straight = false;
            break;

        case 1:
			// Position 2 is reached change parking state, turn wheels
           if(min_scan < 0.40  && max_scan < 0.6){
                park_state= 2;
                // ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);
		   }
           break;

        case 2:
			// Position 3 is reached change to drive forward
            if(max_scan < 0.3 || min_scan < 0.12 ){
                // ROS_INFO("%f: angle middle, %f min_scan", angle_middle, min_scan);
                park_state= 3;
            }
            break;

        case 3:
			// End Position is reached, stop
			if(min_scan < 0.13  || max_scan < 0.31){
                park_state = 4;
            }
            break;
        }
    return;
}

// Subsribe to get Information about the parking spot
void control::parkSpot_detected_Callback(const std_msgs::Float32::ConstPtr& done){

	// Parking spot has been found
	if(done->data == 2.0){
		// enable the parking program
		start_program = true;
		// wait 2 seconds before starting
		ros::Duration(2).sleep();
	}
	return;
}