// Andreas Pfeuffer

#include "ros/ros.h"

// Include Messages

#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// Diese Funktion wird aufgerufen, wenn eine neue Message eingegangen ist
	ROS_INFO("------------------------------------------------------------NEW MESSAGE ---------------------------------------------------------------------");
	ROS_INFO("seg: %i",msg->header.seq);
	ROS_INFO("frame_id: %s", msg->header.frame_id.c_str());
	ROS_INFO("time %i.%i", msg->header.stamp.sec, msg->header.stamp.nsec);
	ROS_INFO("position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	ROS_INFO("orientation: [%f, %f, %f, %f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("covariance: see next line");
	int i = 0;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	i = 6;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	i = 12;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	i = 18;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	i = 24;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	i = 30;
	ROS_INFO("%f \t %f \t %f \t %f \t %f \t %f", msg->pose.covariance[i],  msg->pose.covariance[i+1],  msg->pose.covariance[i+2],  msg->pose.covariance[i+3],  msg->pose.covariance[i+4],  msg->pose.covariance[i+5]);
	ROS_INFO("------------------------------------------------------------------MESSAGE END-----------------------------------------------------------------------");

}

int main(int argc, char **argv)
{

ros::init(argc, argv, "getting_robot_pose");
ros::NodeHandle n;

// ruft Funktion chatterCallback auf, wenn neue Message eingegangen ist

//ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("chatter", 1000, chatterCallback);  
ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000, chatterCallback);
//ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1000, chatterCallback);


// enables receiving any callbacks
ros::spin();

return 0;
}
