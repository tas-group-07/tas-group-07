#include "control_fp.h"

control_fp::control_fp()
{
  control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);
  start_parking_task_ = nh_.advertise<std_msgs::Float32>("parking_done", 1000);
  front_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &control_fp::front_laserCallback,this);
  back_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_back", 1000, &control_fp::back_laserCallback,this);
  robot_position_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000, &control_fp::robot_positionCallback,this);
  wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control_fp::wiiCommunicationCallback,this);
}

void control_fp::front_laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
/* evaluate readings from the front laser scanner*/
{  
  int j = 0;                                                  // variable to count valid sensor data
  int l = 0;                                                  // variable to count valid sensor data (for data smoothing) 
  mean_val_front = 0;                                         // variable to calculate the sum of the first five values (only valid values are taken to account)
  double mean_val_dist = 0;                                   // variable to calculate the sum of four sensor values (for data smoothing)
  val_front[0] = val_front[1];                                // store the penultimate mean value  
  val_front[1] = val_front[2];                                // store the last mean value
  
  /* Calculate the mean value of the first five sensor values on the left side of the car: For the front laser scanner index 719 holds the most left 
  sensor reading. This mean value represents the distance between the car and the wall or other parking objects.*/
  
  for (int i = 0; i < 5; i++)
  {
    if (msg->ranges[719-i] < msg->range_max)                  // if the value of the current angle is Inf, it is excluded from the mean value
    {
      mean_val_front = mean_val_front + msg->ranges[719-i];   // otherwise the value is added to the sum 
      j = j + 1;                                              // and the valid data counter is increased
    }
  }
  
  if (j > 0)                                                  // if at least one of this first five sensor readings is valid, 
  {
    val_front[2] = mean_val_front / j;                        // the mean value for the distance is calculated
  }                                                           // otherwise the last mean value is reused 
  
  
  
  /* When the robot detects the second edge of the first box, the signal find_min changes to 1. Now the robot calculates the distance between the two
  boxes in order to check if the space between them is big enough to park.*/
  
  if (find_min == 1)
  {
    /* For control purposes the actual sensor data from the front laser scanner is written to a .txt file. It is known that the parking space is on the
    left side of the car, so it is sufficient to examine the data from the left side. */
    
    fstream f;
    f.open("/home/tas_group_15/catkin_ws/src/tas_car/find_parking_space/test_control.txt", ios::out);
    f << "Dieser Text geht in die Datei" << endl;
    for (int i = 720; i > 360; i--)
    {
      f <<  msg->ranges[i] << endl;
    }
    f.close();
    ROS_INFO("Data saved");
    
    /* Then the scan data from the left side of the car is smoothed for better analysis. 
    Here the mean value of four data readings is calculated and stored in a the variable scan_data. The angle resolution in scan_data is 1 degree. */
    
    for (int i = 720; i > 360; i = i -4) 
    {
      l = 0;                                                // variable to count valid sensor data
      mean_val_dist = 0;                                    // variable to calculate the sum of four sensor readings
      for (int j = 0; j < 4; j++)
      {
        if (msg->ranges[i-j] < msg->range_max)              // if the value of the current angle is Inf, it is excluded from the mean value
        {
          mean_val_dist = mean_val_dist + msg->ranges[i-j]; // otherwise the value is added to the sum 
          l = l + 1;                                        // and the valid data counter is increased                             
        }
      }
      //ROS_INFO("mean_val_dist: %f",mean_val_dist);
      if (l > 0)                                            // if at least one of the current four sensor readings is valid,
      {
        scan_data[180-i/4] = mean_val_dist/l;               // the mean value for the distance is calculated
      }
    }
    
    /* Calculate a local minimum on the left side of the car. This local minimum represents the angle between the side of the first box to the first 
    front edge of the second box. With the knowledge about this angle and the cars distance to the first box, the distance between the two boxes can 
    be calculated. 
    As there are perturbations for the first few degrees the search starts with a minimum angle of 13° and it breaks if the difference between two 
    neighbouring sensor readings is larger than 20 cm. */
    for (int k = 12; k < 86; k++)
    {
        ROS_INFO("k = %d", k);
      if (scan_data[k+1] - scan_data[k] > 0.2)
      {
        ROS_INFO("break");
        break;
      }
      /* Here a local minimum is defined as a point which has a lower value than its three left and three right neighbours. */
      if (scan_data[k-3] > scan_data[k] && scan_data[k-2] > scan_data[k] && scan_data[k-1] > scan_data[k] && scan_data[k+1] > scan_data[k] && scan_data[k+2] > scan_data[k] && scan_data[k+3] > scan_data[k])
      {
        min = k + 1;                                        // set the angle of the local minimum (angles are expected between 1° and 90°)
        ROS_INFO("Local Minimum for %d degree", min);
      }
     }
    find_min = 0;                                           // a local minimum should be found, so we can go on in the main program 
   }

}

void control_fp::back_laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
/* evaluate readings from the back laser scanner*/
{
  int j = 0;                                                // variable to count valid sensor data
  mean_val_back = 0;                                        // variable to calculate the sum of the first five values (only valid values are taken to account)
  val_back[0] = val_back[1];                                // store the penultimate mean value
  val_back[1] = val_back[2];                                // store the last mean value
  
  /* Calculate the mean value of the first five sensor values on the left side of the car: For the back laser scanner index 0 holds the most left 
  sensor reading. This mean value represents the distance between the car and the wall or other parking objects.*/
  
  for (int i = 0; i < 5; i++)
  {
    if (msg->ranges[i] < msg->range_max)
    {
      mean_val_back = mean_val_back + msg->ranges[i];
      j = j + 1;
    }
  }
  
  if (j > 0)
  {
    val_back[2] = mean_val_back / j;
  }
 
}

// function to get the actual robot y position 
void control_fp::robot_positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  rob_pos_y = msg->pose.pose.position.y;
}

// a flag method that tells us if we are controlling the car manually or automatically
void control_fp::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}
