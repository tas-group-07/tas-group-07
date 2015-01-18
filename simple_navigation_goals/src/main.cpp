// Andreas Pfeuffer

/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int corner_init = 0;
//std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void chatterCallback(const std_msgs::Float32::ConstPtr& msg){
	ROS_INFO("hier");
	corner_init = msg->data;
}


/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

   ros::NodeHandle n;
   ros::Subscriber corner_pub = n.subscribe<std_msgs::Float32>("corner_init",10, chatterCallback);

 //int corner_init = 1;

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (corner_init == 0 ){
	ROS_INFO("AP: Waiting for corner to be set!!");
	ros::Duration(0.01).sleep();
	ros::spinOnce();
    }



    geometry_msgs::Pose waypoint0;
    waypoint0.position.x = 10.1;
    waypoint0.position.y = 8.5;
    waypoint0.position.z = 0.000;
    waypoint0.orientation.x = 0.000;
    waypoint0.orientation.y = 0.000;
    waypoint0.orientation.z = -0.697128;
    waypoint0.orientation.w = 0.716946;

    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 12.0;
    waypoint1.position.y = 6.5;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = 0;
    waypoint1.orientation.w = 1;

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 23.4;
    waypoint2.position.y = 9.2;
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = 0.684173;
    waypoint2.orientation.w = 0.729319;

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 21.7;
    waypoint3.position.y = 19.1;
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = 1;
    waypoint3.orientation.w = 0;

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 18.0;
    waypoint4.position.y = 19.2;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = 1;
    waypoint4.orientation.w = 0;
    
    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 11.0;
    waypoint5.position.y = 16.0;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = -0.725095;
    waypoint5.orientation.w = 0.688649;

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 23.3731;
    waypoint6.position.y = 6.6041;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = 0.641540;
    waypoint6.orientation.w = 0.767090;

	// info

	// waypoint0: corner 2;
    // waypoint1: corner 3;
	// waypoint2: corner 4;
	// waypoint3: between corner 4 and 1;
	// waypoint4: corner 5;
	// waypoint5: between corner 1 and 2;
    // waypoint6: before corner 3;
	switch (corner_init) {

		case 2: // corner 2
           // waypoints.push_back(waypoint6);
			waypoints.push_back(waypoint1);
			waypoints.push_back(waypoint2);
			waypoints.push_back(waypoint3);
			waypoints.push_back(waypoint4);
			waypoints.push_back(waypoint5);
			waypoints.push_back(waypoint0);
            waypoints.push_back(waypoint1);

		case 3: // corner 3
			waypoints.push_back(waypoint2);
			waypoints.push_back(waypoint3);
			waypoints.push_back(waypoint4);
			waypoints.push_back(waypoint5);
			waypoints.push_back(waypoint0);
           // waypoints.push_back(waypoint6);
			waypoints.push_back(waypoint1);
            waypoints.push_back(waypoint2);

		case 4: // corner 4
			waypoints.push_back(waypoint3);	
			waypoints.push_back(waypoint4);
			waypoints.push_back(waypoint5);
			waypoints.push_back(waypoint0);
           // waypoints.push_back(waypoint6);
			waypoints.push_back(waypoint1);
			waypoints.push_back(waypoint2);
			waypoints.push_back(waypoint3);	
					
		default:  // corner 1
			waypoints.push_back(waypoint0);
          //  waypoints.push_back(waypoint6);
			waypoints.push_back(waypoint1);
			waypoints.push_back(waypoint2);
			waypoints.push_back(waypoint3);
			waypoints.push_back(waypoint4);
			waypoints.push_back(waypoint5);
	}
	

	
	//ROS_INFO("AP: size_waypoints: %d",waypoints.size());

    int goals_reached[8];
    for(int i=0; i<8; i++){
		goals_reached[i]=0;
	}

    
  

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i+1);
		goals_reached[i]=1;
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i+1);
        }
    }
    for(int i=0; i<8; i++){
		ROS_INFO("goals reached: %d", goals_reached[i]);}

    return 0;
}
