// Andreas Pfeuffer

#include "ros/ros.h"
#include <sstream>

// Inlcude Messages

#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"

void sending_initialPose(geometry_msgs::Pose pose){

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);// chatter

	// Erstellen des Headers
	std_msgs::Header header;

	header.seq = 1;
	header.frame_id = "0";
	//header.stamp.sec = 1.0;
	//header.stamp.nsec = 0.0;
	header.stamp = ros::Time::now();

	// Initialisierung PoseWithCovariance

	geometry_msgs::PoseWithCovariance pose_withCovariance;

	pose_withCovariance.pose = pose;
	// Erstellen der Covariance-Matrix, mit der Abweichung der Roboterposition
	for (int i = 0; i++; i<36){
		pose_withCovariance.covariance[i] = 0.0;
	}
	pose_withCovariance.covariance[0] = 0.25;
	pose_withCovariance.covariance[7] = 0.25;
	pose_withCovariance.covariance[35] = 0.25;

	// Initialisierung PoseWithCovarianceStamped:

    geometry_msgs::PoseWithCovarianceStamped msg;

	msg.header = header;
	msg.pose = pose_withCovariance;

    // Versenden der Nachricht
    chatter_pub.publish(msg);
	ROS_INFO("AP: Initial Pose was sent to ACRM!!");

}


int main(int argc, char **argv)
{

ros::init(argc, argv, "setting_initional_pose"); // Initialisation of ROS
ros::NodeHandle n;

// Initialisation of the node
ros::Rate loop_rate(10);

// Initialisierung Roboterposition #############################################################
																							//##
	geometry_msgs::Pose pose;																//##
																							//##	
	pose.position.x = 11.2;																	//##
	pose.position.y = 19.7;																	//##
	pose.position.z = 0.0;																	//##
																							//##
	pose.orientation.x = 0.0;																//##
	pose.orientation.y = 0.0;																//##
	pose.orientation.z = -0.7;																//##
	pose.orientation.w = 0.7;																//##
																							//##
//##############################################################################################

// specify a frequency that you would like to loop at (in Herz)
// Tell the master that we are going to be publishing a message of type std_msgs/String on the
// topic "chatter". Maximal 1000 Messages werden gespeichert, falls die Message nicht versendet
// werden konnte.

//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("chatter", 1000);
ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);// chatter

// This loop is repeated every 0.1s until ROS is closed

// Sending Initial Pose (5 times)

int loop = 0;


while (ros::ok())
{
     // AP: maximal 20 mal hintereinander InitialPosition senden
	
	if (loop < 19){
		//sending_initialPose(pose);

		// Erstellen des Headers
		std_msgs::Header header;

		header.seq = 1;
		header.frame_id = "0";
		//header.stamp.sec = 1.0;
		//header.stamp.nsec = 0.0;
		header.stamp = ros::Time::now();

		// Initialisierung PoseWithCovariance

		geometry_msgs::PoseWithCovariance pose_withCovariance;

		pose_withCovariance.pose = pose;
		// Erstellen der Covariance-Matrix, mit der Abweichung der Roboterposition
		for (int i = 0; i++; i<36){
			pose_withCovariance.covariance[i] = 0.0;
		}
		pose_withCovariance.covariance[0] = 0.25;
		pose_withCovariance.covariance[7] = 0.25;
		pose_withCovariance.covariance[35] = 0.25;

		// Initialisierung PoseWithCovarianceStamped:

		geometry_msgs::PoseWithCovarianceStamped msg;

		msg.header = header;
		msg.pose = pose_withCovariance;

		// Versenden der Nachricht
		chatter_pub.publish(msg);
		ROS_INFO("AP: Initial Pose was sent to ACRM!!");


		loop = loop + 1;
	}
	else {
		if (loop < 20){
			loop = loop + 1;
			ROS_INFO ("AP: Initial Position was sent 20-times.");
		}
	}
	

    // enables receiving any callbacks
    ros::spinOnce();
    // Prozess schläft, bis die 0.1s vorbei sind
    loop_rate.sleep();
    }

return 0;
}
