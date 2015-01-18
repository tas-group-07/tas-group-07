// Andreas Pfeuffer

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//PointCloud cloud1;
PointCloud::Ptr cloud1 (new PointCloud);
int it = 0;

void init(){
	cloud1->header.frame_id = "frame1";
	cloud1->width = 0;
	cloud1->height = 0;
}



void CloudOperations_CopyCloud(const PointCloud::ConstPtr& cloud1, PointCloud::Ptr& cloud2){
	// cloud2 = cloud1

	cloud2->clear();

	cloud2->header.frame_id = cloud1->header.frame_id;
	cloud2->width = cloud1->width;
	cloud2->height = cloud1->height;

	//for (int i = 0; i < cloud1->width; i++){
	//	cloud2->points.push_back(cloud1->points[i]);
	//}

	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud1->points)
    	cloud2->points.push_back(pt);

	ROS_INFO("Alles kopiert %d", it);
	it = it +1;
}

void printCloud(){
	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud1->points)
 	   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  CloudOperations_CopyCloud(msg, cloud1);
  printCloud();

  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "displaying_laser_data");
	ros::NodeHandle nh;
	
	init();

	ros::Subscriber sub = nh.subscribe<PointCloud>("scan", 1, callback);
	ros::Subscriber sub2 = nh.subscribe<PointCloud>("scan", 1, callback);
	
	ros::Rate loop_rate(4);
 	while (nh.ok()) {
    	ros::spinOnce();
 		loop_rate.sleep ();
  	}

//ros::spin();

}

