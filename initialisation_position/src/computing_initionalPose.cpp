// Andreas Pfeuffer

//	This node compute the initional pose of the car after the C-booton of the
//	WII was pressed. For this, both laser sensors are used. First, 10 messages
//	from each laser sensor are used to determine the filtered estimate of the
//	current	environment of the corner. The result is then saved in a point
//	cloud. Then, the point cloud is compared with all stored point clouds of 
//	the directory samples by means of the ICP Algorithm. It is assumed that
//	the car starts from the corner, which point cloud has the minimal Eucleadian
//	Error to the actual point cloud. After this, the corresponding corner
//	information are read in and the initional pose can be send to the Navigation
//	Stack. Furthermore, the corresponding corner is send to the

#include <ros/ros.h>
#include <iostream>
#include <fstream>  // zum Einlesen von Files
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cstdlib> // für Eingabe

#include <pcl/registration/icp.h>

// Include Messages

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Definition Constanten

const int max_Messages_to_filter = 10;
const float distance_lasers = 0.51;
const float distance_lasers_half = distance_lasers/2.0;
 
// globale Variabeln

int it = 0;
int message_laser_front; // gibt an, ob die Message laser_front schon verarbeitet wurde
int message_laser_back; // gibt an, ob die Message laser_back schon verarbeitet wurde
float filter_laser_back[512][2]; // unten auch werte verändern!!
float filter_laser_front[720][2]; // unten auch werte verändern!!
int ready_laserfront; // is 1, if all processing is finished for laser_front
int ready_laserback;  // is 1, if all processing is finished for laser_back
int ready;    // is 1, if all work is done

int wii = 1; // if wii = 1: the node wait, until the the C-button of the Wii was pressed.
             // if wii = 0: the node starts at once
int wii_pressed; // = 1, if the C-button of the wii was pressed

int corner;
int number;

PointCloud::Ptr cloud_umgebung (new PointCloud);

// Funktionendeklaration

void ScanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void ScanFront_Filter(const sensor_msgs::LaserScan::ConstPtr& msg);
void ScanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void ScanBack_Filter(const sensor_msgs::LaserScan::ConstPtr& msg);
void Wii_Callback(const std_msgs::Float32::ConstPtr& msg);
void init();
void start_localization();
void send_initional_Pose(int corner);

// Funktionen:

void init(){ 

	// Initialisation of cloud_umgebung
	cloud_umgebung->header.frame_id = "laser";
	cloud_umgebung->width = 0;
	cloud_umgebung->height = 0;

	// Initialisation global variable
	message_laser_front = 0;
	message_laser_back = 0;
	ready_laserfront = 0;
	ready_laserback = 0;
	ready = 0;
	wii_pressed = 0;


	// Setting filter_laser_back to zero
	for (int i = 0; i < 512; i++){                                      
		filter_laser_back[i][0] = 0.0;
		filter_laser_back[i][1] = 0.0;
	}
	// Setting filter_laser_front to zero
	for (int i = 0; i < 720; i++){                                     
		filter_laser_front[i][0] = 0.0;
		filter_laser_front[i][1] = 0.0;
	}

	ROS_INFO("AP: Initialisation of initional_position::computing_initionalPose ready!!");
	
}

void Wii_Callback(const std_msgs::Float32::ConstPtr& msg){

	// the node starts working, if the c-botton of the wii was pressed
	
	if (msg->data >= 0.5){
		wii_pressed = 1;
	}

}

void ScanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	// processing of the sensor data of the laser sensor in the back
	
	if (wii == 0 || (wii == 1 && wii_pressed == 1)) {
		if (message_laser_back < max_Messages_to_filter){

			// Filtering Sensor Data:

			ScanBack_Filter(msg);

			ROS_INFO("AP: ScanBack_Callback: Message %i processed",message_laser_back);

			// Counting, how many messages resived yet
			message_laser_back = message_laser_back + 1;
		}
	}
	else if (wii == 1){
		ROS_INFO("AP: Waiting for Wii to be pressed!!!");
	}

};

void ScanFront_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	// processing of the sensor data of the laser sensor in the Front
	
	if (wii == 0 || (wii == 1 && wii_pressed == 1)) {
		if (message_laser_front < max_Messages_to_filter){

			// Filtering Sensor Data:

			ScanFront_Filter(msg);

			ROS_INFO("AP: ScanFront_Callback: Message %i processed",message_laser_front);

			// Counting, how many messages resived yet
			message_laser_front = message_laser_front + 1;
		}
	}
	else if (wii == 1) {
		ROS_INFO("AP: Waiting for Wii to be pressed!!!");
	}

};

void ScanBack_Filter(const sensor_msgs::LaserScan::ConstPtr& msg){

	// The Filter concept is the following one: take the average of N laser data.

	for (int i = 0; i < msg->ranges.size(); i++){
		if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max){ //Note: values < range_min or > range_max should be discarded
			filter_laser_back[i][0] = filter_laser_back[i][0] + msg->ranges[i];  
			filter_laser_back[i][1] = filter_laser_back[i][1] + 1;
		}
	}


	if (message_laser_back == max_Messages_to_filter-1){ 
		// Adding the filtered values to the cloud cloud_umgebung
		for (int i = 0; i < msg->ranges.size(); i++){
			if (filter_laser_back[i][1] != 0){

				float length = filter_laser_back[i][0] / filter_laser_back[i][1];
				float x = -distance_lasers_half -length * std::cos(i*msg->angle_increment + msg->angle_min);
				float y = -length * std::sin(i*msg->angle_increment + msg->angle_min);

				cloud_umgebung->push_back(pcl::PointXYZ(x, y, 10.0));			
			}

		}

		ready_laserback = 1;
		ROS_INFO("AP: LaserBack: all values processed");		

	}

}


void ScanFront_Filter(const sensor_msgs::LaserScan::ConstPtr& msg){

	// The Filter concept is the following one: take the average of N laser data.

	for (int i = 0; i < msg->ranges.size(); i++){
		if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max){ //Note: values < range_min or > range_max should be discarded
			filter_laser_front[i][0] = filter_laser_front[i][0] + msg->ranges[i];  
			filter_laser_front[i][1] = filter_laser_front[i][1] + 1;
		}
	}


	if (message_laser_front == max_Messages_to_filter-1){ 
		// Adding the filtered values to the cloud cloud_umgebung
		for (int i = 0; i < msg->ranges.size(); i++){
			if (filter_laser_front[i][1] != 0){

				float length = filter_laser_front[i][0] / filter_laser_front[i][1];
				float x = distance_lasers_half + length * std::cos(i*msg->angle_increment + msg->angle_min);
				float y = length * std::sin(i*msg->angle_increment + msg->angle_min);

				cloud_umgebung->push_back(pcl::PointXYZ(x, y, 0.0));			
			}

		}
	
		ready_laserfront = 1;
		ROS_INFO("AP: LaserFront: all values processed");
	
	}

}

void send_initional_Pose(int corner){

	// send the initional Pose to the Navigation Stack and activate the autonomous driving and the simple_navigation_goals nodes

	ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
	ros::Publisher enable_pub = n.advertise<std_msgs::Float32>("enable_driving",1000);
	ros::Publisher corner_pub = n.advertise<std_msgs::Float32>("corner_init",1000);


	double x, y, z, or_x, or_y, or_z, or_w;

	// Load to the corner corresponding text file to get the corresponding initional pose:

	ROS_INFO("corner: %d", corner);

	  std::stringstream file_name_str;
;

	file_name_str << "/home/tas_group_15/catkin_ws/src/tas_car/initialisation_position/corners/corner_" << corner << ".txt";
     // file_name_str << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/corners/corner_" << corner << ".txt";
	  //file_name_str << "/home/andreas/catkin_ws/src/initialisation_position/corners/corner_" << corner << ".txt";
	  std::string file_name_cstr(file_name_str.str());

	  std::ifstream file;                          // neuen Lese-Stream erzeugen
	  file.open(file_name_cstr.c_str(), std::ios_base::in); // Datei_1.txt öffnen

	  file >> x ; //>>y >> z >> or_x >> or_y >> or_z >> or_w;
	  file >> y;
	  file >> z;
	  file >> or_x;
	  file >> or_y;
	  file >> or_z;
	  file >> or_w;//*/
	  file.close();


	  std::cout << x << " " << y << " " << or_z << " " << or_w << std::endl;

    // Creating of the message for sending the initional pose

	// Erstellen des Headers
		std_msgs::Header header;

		header.seq = 1;
        header.frame_id = "map";
		//header.stamp.sec = 1.0;
		//header.stamp.nsec = 0.0;
		header.stamp = ros::Time::now();

		// Initialisierung PoseWithCovariance

		geometry_msgs::PoseWithCovariance pose_withCovariance;

		geometry_msgs::Pose pose;																
																								
		pose.position.x = x;																
		pose.position.y = y;															
		pose.position.z = z;															
																							
		pose.orientation.x = or_x;															
		pose.orientation.y = or_y;															
		pose.orientation.z = or_z;														
		pose.orientation.w = or_w;	

		pose_withCovariance.pose = pose;

		// Erstellen der Covariance-Matrix, mit der Abweichung der Roboterposition
		for (int i = 0; i++; i<36){
			pose_withCovariance.covariance[i] = 0.0;
		}
		pose_withCovariance.covariance[0] = 0.25;
        pose_withCovariance.covariance[7] = 0.25;
        pose_withCovariance.covariance[35] = 0.068539;
        //pose_withCovariance.covariance[35] = 0.25;

		// Initialisierung PoseWithCovarianceStamped:

		geometry_msgs::PoseWithCovarianceStamped msg;

		msg.header = header;
		msg.pose = pose_withCovariance;

		std_msgs::Float32 msg2;
		std_msgs::Float32 msg3;
		msg2.data = 1.0;
		msg3.data = corner;

		// send the message in order to set the initional pose and to activate simple Navigation Goals
		for (int j = 0; j < 10; j++){
			chatter_pub.publish(msg);
			enable_pub.publish(msg2);
			corner_pub.publish(msg3);
		        ROS_INFO("AP: Initial Pose was sent to ACRM!!");
			ros::Duration(0.25).sleep();
		}


}


void start_localization(){

	  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_umgebung (new pcl::PointCloud<pcl::PointXYZ>);

	 /* if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/andreas/catkin_ws/src/initialisation_position/samples/corner_1.pcl", *cloud_umgebung) == -1)
	//* load the file
	  {
		ROS_ERROR ("Couldn't read file test_pcd.pcd \n");
		return ;
	  }
	  std::cout << "Loaded " << cloud_umgebung->width * cloud_umgebung->height  << " data points from test_pcd.pcd with the following fields: " << std::endl;*/


  // ---------------------------------------------------
  // ----      Saving cloud_umgebung with corresponding corners (Only for test runs)                          -----
  // ----------------------------------------------


	std::stringstream file_name_save;
	//file_name_save << "/home/andreas/catkin_ws/src/initialisation_position/samples/test_" << number << ".pcl";
	file_name_save << "/home/tas_group_15/catkin_ws/src/tas_car/initialisation_position/samples/test_" << number << ".pcl";
	//file_name_save << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/test_" << number << ".pcl";

	if (pcl::io::savePCDFile<pcl::PointXYZ> (file_name_save.str(), *cloud_umgebung) == -1) //* load the file
	{
		ROS_ERROR("AP: Couldn't read file test_%i.pcd \n",number);
		return;
	}


	// Saving a text-file, in which the information, which corner it is, is saved

	std::stringstream file_name_str_save;
	//file_name_str_save << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/testcorner_" << number << ".txt";
	file_name_str_save << "/home/tas_group_15/catkin_ws/src/tas_car/initialisation_position/samples/testcorner_" << number << ".txt";
	//file_name_str_save << "/home/andreas/catkin_ws/src/initialisation_position/samples/testcorner_" << number << ".txt";
	std::string file_name_cstr_save(file_name_str_save.str());

	std::ofstream file_save;                          // neuen Lese-Stream erzeugen
	file_save.open(file_name_cstr_save.c_str(), std::ios_base::out); // Datei_1.txt öffnen

	file_save << corner;
	file_save.close();
  
  // ---------------------------------------------------
  // ----     Search for the best corner using ICP                           -----
  // ----------------------------------------------

    float FitnessScore_minimum = 9999999999.0;
    int FitnessScore_corner = 0;

    for (int iter = 0; iter <= 10; iter++){

		 // ---------------------------------------------------
  		// ----      Load cloud_local                            -----
  		// ----------------------------------------------


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZ>);

        std::stringstream file_name;
       // file_name << "/home/andreas/catkin_ws/src/initialisation_position/samples/corner_" << iter << ".pcl";
        //file_name << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << iter << ".pcl";
	file_name << "/home/tas_group_15/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << iter << ".pcl";

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name.str(), *cloud_local) == -1) //* load the file
        {
            ROS_ERROR("AP: Couldn't read file corner_%i.pcd \n",iter);
            return;
        }



      // ---------------------------------------------------
      // ----      ICP                                -----
      // ---------------------------------------------------

          pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
          // Set the input source and target
          icp.setInputCloud (cloud_local);
          icp.setInputTarget (cloud_umgebung);  // cloud_tartget
          // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
          icp.setMaxCorrespondenceDistance (0.5);
          icp.setMaximumIterations (100);
          // Set the transformation epsilon (criterion 2)
          icp.setTransformationEpsilon (1e-8);
          // Set the euclidean distance difference epsilon (criterion 3)
          //icp.setEuclideanFitnessEpsilon (1);
          // Perform the alignment
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered (new pcl::PointCloud<pcl::PointXYZ>);
          icp.align (*cloud_source_registered);
          // Obtain the transformation that aligned cloud_source to cloud_source_registered
          Eigen::Matrix4f transformation = icp.getFinalTransformation ();

		  // ----------------------------------------------------------------------------------
          // Search for the minium fitness score and the corresponding corner
		  // ---------------------------------------------------------------------------------

          if (FitnessScore_minimum > icp.getFitnessScore()){

              FitnessScore_minimum = icp.getFitnessScore();
              // Load the file, in which the corresponding corner is saved;

              std::stringstream file_name_str;
	      file_name_str << "/home/tas_group_15/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << iter << ".txt";
              //file_name_str << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << iter << ".txt";
              //file_name_str << "/home/andreas/catkin_ws/src/initialisation_position/samples/corner_" << iter << ".txt";
              std::string file_name_cstr(file_name_str.str());

              std::ifstream file;                          // neuen Lese-Stream erzeugen
              file.open(file_name_cstr.c_str(), std::ios_base::in); // Datei_1.txt öffnen

              file >> FitnessScore_corner;
              file.close();
          }

          // Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
          std::cout << "cloud " << iter << ": " << icp.getFitnessScore() << std::endl;

    }

    std::cout << FitnessScore_minimum << "    " << FitnessScore_corner << std::endl;

	// Sending initional Pose to the car

	send_initional_Pose(FitnessScore_corner);

   
	ready = 1;
	
	ROS_INFO("AP: Car ist lokalisiert!!!");

}

// Main:

int main(int argc, char** argv)
{
	ros::init(argc, argv, "computing_initionalPose");
	ros::NodeHandle n;
	
	init();
	//start_localization();

	sensor_msgs::LaserScan scan;

	ros::Subscriber sub_back = n.subscribe<sensor_msgs::LaserScan>("scan_back", 1, ScanBack_Callback);
	ros::Subscriber sub_front = n.subscribe<sensor_msgs::LaserScan>("scan", 1, ScanFront_Callback);
	ros::Subscriber sub_wii = n.subscribe<std_msgs::Float32>("wii_info", 1, Wii_Callback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

	//ros::Subscriber sub = n.subscribe<PointCloud>("points2", 1, callback);
	//ros::Subscriber sub2 = n.subscribe<PointCloud>("points1", 1, callback);
	
	ros::Rate loop_rate(4);
 	while (n.ok()) {

        /*if (argc != 3)
		{
            ROS_ERROR("usage: number_of_point_cloud corner");
			return 1;
        }*/

		// if both messages recived n-times:

		if (ready_laserfront == 1 && ready_laserback == 1 && ready == 0){
			ROS_INFO("AP: start localization!!");
			start_localization();
		}
		else if (ready == 1){
			//ros::spin();
		}

        number = 50; //atoll(argv[1]);
        corner = 1; //atoll(argv[2]);

    	ros::spinOnce();
 		loop_rate.sleep ();
  	}

//ros::spin();

}
