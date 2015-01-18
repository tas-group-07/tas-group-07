// Andreas Pfeuffer

// @ par1: number of current measurement
//	@ par2: current corner
//	This node creates a point cloud of a predefined corner and stores the 
//	corresponding point cloud in the file samples. It is activated by pressing 
//	the C-button of the WII	

#include <ros/ros.h>
#include <iostream>
#include <fstream>  // zum Einlesen von Files
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cstdlib> // für Eingabe


// Include Messages

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>



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
	for (int i = 0; i < 512; i++){                                      // <-- hier auch ändern!!!
		filter_laser_back[i][0] = 0.0;
		filter_laser_back[i][1] = 0.0;
	}
	// Setting filter_laser_front to zero
	for (int i = 0; i < 720; i++){                                     // <-- hier auch ändern!!!
		filter_laser_front[i][0] = 0.0;
		filter_laser_front[i][1] = 0.0;
	}

	ROS_INFO("AP: Initialisation of initional_position::computing_initionalPose ready!!");
	
}

void Wii_Callback(const std_msgs::Float32::ConstPtr& msg){

	// this node starts working, if the C-booton of the wii was pressed.
	
	if (msg->data >= 0.5){
		wii_pressed = 1;
	}

}

void ScanBack_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
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


void start_localization(){



	std::stringstream file_name;
	//file_name << "/home/andreas/catkin_ws/src/initialisation_position/samples/corner_" << number << ".pcl";
	file_name << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << number << ".pcl";

	if (pcl::io::savePCDFile<pcl::PointXYZ> (file_name.str(), *cloud_umgebung) == -1) //* load the file
	{
		ROS_ERROR("AP: Couldn't read file corner_%i.pcd \n",number);
		return;
	}


	// Saving a text-file, in which the information, which corner it is, is saved

	std::stringstream file_name_str;
	file_name_str << "/home/tas_group_07/catkin_ws/src/tas_car/initialisation_position/samples/corner_" << number << ".txt";
	//file_name_str << "/home/andreas/catkin_ws/src/initialisation_position/samples/corner_" << number << ".txt";
	std::string file_name_cstr(file_name_str.str());

	std::ofstream file;                          // neuen Lese-Stream erzeugen
	file.open(file_name_cstr.c_str(), std::ios_base::out); // Datei_1.txt öffnen

	file << corner;
	file.close();

	ready = 1;
	
	ROS_INFO("AP: Data are stored successfully!!!");

}

// Main:

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_CornerPointCloud");
	ros::NodeHandle n;

	if (argc != 3)
	{
		ROS_INFO("usage: number_of_point_cloud corner");
		return 1;
	}
	
	
	init();
	//start_localization();

	sensor_msgs::LaserScan scan;

	ros::Subscriber sub_back = n.subscribe<sensor_msgs::LaserScan>("scan_back", 1, ScanBack_Callback);
	ros::Subscriber sub_front = n.subscribe<sensor_msgs::LaserScan>("scan", 1, ScanFront_Callback);
	ros::Subscriber sub_wii = n.subscribe<std_msgs::Float32>("wii_info", 1, Wii_Callback);

	//ros::Subscriber sub = n.subscribe<PointCloud>("points2", 1, callback);
	//ros::Subscriber sub2 = n.subscribe<PointCloud>("points1", 1, callback);
	
	ros::Rate loop_rate(4);
 	while (n.ok()) {

		// if both messages recived n-times:

		if (ready_laserfront == 1 && ready_laserback == 1 && ready == 0){
			ROS_INFO("AP: start localization!!");
			start_localization();
		}
		else if (ready == 1){
			//ros::spin();
		}

		number = atoll(argv[1]);
		corner = atoll(argv[2]);

    	ros::spinOnce();
 		loop_rate.sleep ();
  	}

//ros::spin();

}
