// Andreas Pfeuffer & Fabian Sonntag

#include "control/control.h"

#include <std_msgs/Float32.h>




int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;
	ros::NodeHandle n;

	ros::Publisher chatter_wii_info = n.advertise<std_msgs::Float32>("wii_info", 1000);
	std_msgs::Float32 wii_info;
	float wii_information;

    ros::Rate loop_rate(50);
    ros::Time time_now = ros::Time::now()-ros::Duration(100);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
			wii_info.data = 0.0;
        }
        else
        {
			wii_info.data = 1.0;	

			if (autonomous_control.enable_driving == 1.0){		

				if(autonomous_control.control_Brake.data==1)
				{
				    autonomous_control.control_servo.x=1500;
				    autonomous_control.control_servo.y=1500;
				}
				else
				{
                    ROS_INFO("Automatic Control!");
                    if(autonomous_control.cmd_linearVelocity>0)
                    {
                        autonomous_control.control_servo.x = autonomous_control.velocity_value;
                        ROS_INFO("Driving Forward");
                    }
                    else if(autonomous_control.cmd_linearVelocity<0)
                    {
                        autonomous_control.control_servo.x = 1300;
                        ROS_INFO("Driving Backwards");
                    }
                    else
                    {
                        autonomous_control.control_servo.x = 1500;
                    }

                    autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
                    if (autonomous_control.cmd_steeringAngle > 1900)
                    {
                        ROS_INFO("Driving to the right, linear_vel = %f",autonomous_control.control_servo.x);
                    }
                    else if (autonomous_control.cmd_steeringAngle < 1100)
                    {
                        ROS_INFO("Driving to the left, linear_vel = %f",autonomous_control.control_servo.x);
                    }
                    else
                    {
                        ROS_INFO("Driving straight forward");
                    }
		        }
            }
				else {
					ROS_INFO("AP: wait for signal from initionalPose Compution!");
				}
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);



		chatter_wii_info.publish(wii_info);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
