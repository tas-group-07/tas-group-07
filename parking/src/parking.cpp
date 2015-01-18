/****************************************/
/* Name       : parkin.cpp				*/
/* Function   : main function to park	*/
/* Author     : Tanja Weber				*/
/* Last update: 16.01.2015				*/
/****************************************/

#include "control/control_p.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "parking");
    control autonomous_control;

    ros::Rate loop_rate(50);
    ros::Time time_now = ros::Time::now()-ros::Duration(100);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
        //    ROS_INFO("Manually Control!");
            autonomous_control.wii_state = 0;
        }
        else
        {

		if(autonomous_control.start_program == true || autonomous_control.counter == 2){
			// set counter to two: default if start_program gets resetted
			autonomous_control.counter=2;

            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
				//set variable which enables to collect laser scanner data
                autonomous_control.wii_state = 1;
		
					//checks the parking state, the car is in
                    switch(autonomous_control.park_state){
						case -1:
							//drive straight backwards to correct starting position
							//drive backwards
                            autonomous_control.control_servo.x = 1415;
                            ROS_INFO("-1: Driving Backwards");

                            //straight
                            autonomous_control.control_servo.y = 1500;
                            ROS_INFO("straight");
                            break;
                        case 0:
                            //drive backwards
                            autonomous_control.control_servo.x = 1400;
                            ROS_INFO("Driving Backwards");

                            //towards the left wall
                            autonomous_control.control_servo.y = 2000;
                            ROS_INFO("wheels turned right");
                            break;
                        case 1:
                            //drive backwards
                            autonomous_control.control_servo.x = 1415;
                            ROS_INFO("1: Driving Backwards");

                            //towards the left wall, straight
                            autonomous_control.control_servo.y = 1500;
                            ROS_INFO("straight");
                            break;
                        case 2:
                            //drive backwards
                            autonomous_control.control_servo.x = 1415;
                            ROS_INFO("2: Driving Backwards");

                            // towards the left wall, left
                            autonomous_control.control_servo.y = 1100;
                            ROS_INFO("wheels turned left");
                            break;
                        case 3:
                            //drive forward
                            autonomous_control.control_servo.x = 1540;
                            ROS_INFO("3: Driving forward");

                            // correction
                            autonomous_control.control_servo.y = 2000;
                            ROS_INFO("wheels turned left");
                            break;
                        case 4:
                            // stop
                            // endposition is reached
							autonomous_control.control_servo.x = 1500;
                            autonomous_control.control_servo.y = 1500;
                            ROS_INFO("4: Stop");
                            break;
                        default:
                            // default position
                            autonomous_control.control_servo.x = 1500;
                            autonomous_control.control_servo.y = 1500;
                            ROS_INFO("default");
                            break;
                    }
                }
            }
		   // publish command to servo
           autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
        }

		// update
        ros::spinOnce();
		// sleep for 50
        loop_rate.sleep();
    }

    return 0;
}
