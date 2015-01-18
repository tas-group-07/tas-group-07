#include "control/control.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    
	//rename autonomous_control!
    ros::init(argc, argv, "autonomous_control_park");
    control autonomous_control;

  //  ros::NodeHandle nh;
  //  ros::Subscriber scanSub;

    ros::Rate loop_rate(50);
    ros::Time time_now = ros::Time::now()-ros::Duration(100);

    autonomous_control.park_state = 0;


    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
				switch(autonomous_control.park_state){
					case 0:
						//drive backwards
                        autonomous_control.control_servo.x = 1375;
						ROS_INFO("Driving Backwards");
				
						//towards the left wall
						autonomous_control.control_servo.y = 1900;
						ROS_INFO("wheels turned right"); 
						break;
					case 1:
						//drive backwards
                        autonomous_control.control_servo.x = 1375;
						ROS_INFO("Driving Backwards");
				
						//towards the left wall, straight
						autonomous_control.control_servo.y = 1500;
						ROS_INFO("straight"); 
						break;
					case 2:
						//drive backwards
                        autonomous_control.control_servo.x = 1375;
						ROS_INFO("Driving Backwards");

						// towards the left wall, left
						autonomous_control.control_servo.y = 1100;
						ROS_INFO("wheels turned left");
						break;
					case 3:
						//drive forward
                        autonomous_control.control_servo.x = 1540;
                        ROS_INFO("Driving forward");

						// correction
						autonomous_control.control_servo.y = 1100;
						ROS_INFO("wheels turned left");
						break;
					case 4:
						// default position
						autonomous_control.control_servo.x = 1500;
						autonomous_control.control_servo.y = 1500;
						break;
					default:
						// default position
						autonomous_control.control_servo.x = 1500;
						autonomous_control.control_servo.y = 1500;
						break;
				}
            }
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }
        ros::spinOnce();
        loop_rate.sleep();


    }

    return 0;

}
