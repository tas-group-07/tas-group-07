// Andreas Pfeuffer & Fabian Sonntag

#include "control.h"

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

    sensor_sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &control::LaserScan_Velocity_Calculation,this); // FS

	enable_driving_sub = nh_.subscribe<std_msgs::Float32>("enable_driving",1000,&control::enable_drivingCallback,this); // AP

	enable_driving = 0.0; // AP


//    Fp = 10;// need to test! default:125

//    current_ServoMsg.x = 1500;
//    current_ServoMsg.y = 1500;

//    previous_ServoMsg.x = 1500;
//    previous_ServoMsg.y = 1500;

}

// Andreas
void control::enable_drivingCallback(const std_msgs::Float32::ConstPtr& msg){
	if (msg->data > 0.5){
		enable_driving = 1.0;
	}
}

// Andreas End

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

//changes Fabian
void control::LaserScan_Velocity_Calculation(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Declaration of Variables for Tuning
    double min_range_core = 5;
    double min_range_sides = 5;
    double min_range = 0;
    int msg_mid_index = 360;			//90 deg Index of Laser Scan Message
    int index_angle_delta_side = 720/180 * 25;	//Area to be considered: -25deg to 25deg
    int index_angle_delta_core = 720/180 * 15;	//Core Area to be considered: -15deg to 15deg
    int max_vel_value = 1580;			//Maximum Velocity Value that can be computed by function
    double velocity_value_buffer1 = 1550;
    double velocity_value_buffer2 = 1550;

    double min_safe_distance = 0.5;	//Safety Distance to Car between 0deg and 65deg or 115deg and 180deg
    double side_threshold = 1.4;//If an Obstacle is within this Distance in Side Area, Velocity is not adapted
    double core_threshold = 1.8;//If an Obstacle is within this Distance in Core Area, Velocity is not adapted
    int periphery_clear = 2;	//Velocity is only adapted if Periphery (between 0deg and 65deg or 115deg and 180deg in respect to min_safe_distance) is clear

    if (initialization_counter != 45678)
    {
        //Initialization
        ROS_INFO("INITIALISATION, initialisation_counter= %f", initialisation_counter);
        for (int i = 0; i < 20;++i)
        {
            	velocity_value_old[i]=1550;
        }
        initialisation_counter = 45678;
    }

    periphery_clear = 1;
    //Determination of Distance to nearest Object in considered Area
    for (int i=0; i <= 720 ; ++i)
    {
        if(msg->ranges[i]>0.01)
        {
            if (i > (msg_mid_index + index_angle_delta_side))	//check of Periphery
            {
                min_safe_distance = 0.3 + (1.1 * ((720.0-i)/(360.0 - index_angle_delta_side))); //Computation of min_safe_distance
                if (msg->ranges[i] <= min_safe_distance)
                {
                    periphery_clear = 0;
                }
            }
            else if ((i > (msg_mid_index + index_angle_delta_core))&&((i< (msg_mid_index + index_angle_delta_side))&&(min_range_sides > msg->ranges[i]))) // Right Side (btw. 105 and 115deg)
            {
                    min_range_sides = msg->ranges[i];
            }
            else if ((i < (msg_mid_index + index_angle_delta_core))&&(i > (msg_mid_index - index_angle_delta_core))&&(msg->ranges[i] < min_range_core)) // Core (btw. 75 and 105deg)
            {
                    min_range_core = msg->ranges[i];
            }
            else if ((i < (msg_mid_index - index_angle_delta_core))&&(i> (msg_mid_index - index_angle_delta_side))&&(msg->ranges[i] < min_range_sides)) // Left Side (btw. 65 and 75deg)
            {
                    min_range_sides = msg->ranges[i];
            }
            else if (i < (msg_mid_index - index_angle_delta_side))	//check of Periphery
            {
                min_safe_distance = 0.3 + (1.1 * (i/(360.0 - index_angle_delta_side)));
                if (msg->ranges[i] <= min_safe_distance)
                {
                    periphery_clear = 0;
                }
            }
        }
    }

    	ROS_INFO("Minimaler Laser_value Core: %f", min_range_core);
    	ROS_INFO("Minimaler Laser_value Sides: %f", min_range_sides);

	//Determination of maximum Velocity that guarantees safety of car

	//If an Obstacle is immediately in front of the Car, it should not go on
    	if ((min_range_core < 0.15)||(min_range_sides < 0.16)) //Immediate proximity to car
	{
		velocity_value_buffer1 = 1500;
		ROS_INFO("Obstacle detected, distance < 15cm -> Braking");
		velocity_value = 1500;
		return;
	}
	//if there is no Obstacle within 4.8m in the core-area or 4.4m in the side areas and Periphery is also clear -> Full-Speed
    	else if ((min_range_core > (core_threshold + 3.0))&&(min_range_sides > (side_threshold + 3.0))&&(periphery_clear==1)) //No Obstacle near the car -> Full-Speed
	{
		velocity_value_buffer1 = max_vel_value;
		ROS_INFO("No Obstacle detected in near proximity -> Full Speed");
	}
	//if there is an obstacle between 1.8m and 4.8m (in the core area) or between 1.4m and 4.4m (in the side areas)
	//and Periphery is clear -> adapt velocity seamlessly in respect to the closest obstacle
    	else if ((min_range_core > core_threshold)&&(min_range_sides > side_threshold)&&(periphery_clear==1)) //Obstacle far enough away to accelerate
	{
		if ((min_range_core - core_threshold) < (min_range_sides - side_threshold))
		{
			min_range = min_range_core - core_threshold;
		}
		else
		{
			min_range = min_range_sides - side_threshold;
		}
        	double delta = (min_range/3.0) * (double(max_vel_value) - 1550);
       		velocity_value_buffer1 = 1550 + delta;
        	ROS_INFO("Obstacle detected, distance not critical -> Adapting Speed");
	}
	//Else -> do not adapt speed and drive slowly
	else
	{
		velocity_value_buffer1 = 1550;
		ROS_INFO("Obstacle detected, distance < 2m -> Driving Slowly");
	}

	//Filter to avoid abrupt acceleration
    velocity_value_buffer2 = velocity_value_buffer1;
    for (int i = 0; i < 20; ++i)
	{
        velocity_value_buffer2 += velocity_value_old[i];
	}
    velocity_value = velocity_value_buffer2/21;
    ROS_INFO("Velocity_value: %f", velocity_value);

	// Safeguard against malcomputing
    if ((velocity_value > max_vel_value)||(velocity_value < 1500))
	{
        velocity_value = 1550;
        ROS_INFO("ERROR: computed invalid velocity_value");
	}

	// Shifting old velocity_values
    for (int i=19; i > 0; --i)
	{
		velocity_value_old[i]=velocity_value_old[i-1];
	}
    velocity_value_old[0] = velocity_value_buffer1;
    ROS_INFO("Velocity_value: %f", velocity_value);

}
//changes Fabian ende

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
