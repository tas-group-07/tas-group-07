# include "control/control_fp.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_parking_space");
  control_fp  fp_control;
  
  ros::Rate loop_rate(30);                                      /* loop at 30 Hz */
  
  int j = 0;                                                    /* state variable of find-parking-space process*/
  double initial_distance;                                      /* start distance to wall/object to check if the car is driving straight ahead */
  double diff_distance;                                         /* difference between the start distance (initial_distance) and current distance to wall/object */
  double diff_front_prev = 0;                                   /* previous difference between penultimate and current mean value from front laser scanner*/
  double diff_front_prev_prev = 0;                              /* penultimate difference between penultimate and current mean value from front laser scanner*/
  double diff_back_prev = 0;                                    /* previous difference between penultimate and current mean value from back laser scanner*/
  double diff_back_prev_prev = 0;                               /* penultimate difference between penultimate and current mean value from back laser scanner*/
  double diff_distance_steering;                                /* difference between initial_distance and current distance to wall/object (used for steering control)*/
  double start_pos_distance_driving;                            /* robot y position when the velocity is set to zero */
  //double act_rel_rob_pos;                                       /* current robot y position relative to the start_pos_distance_driving position */
  double rob_pos_old = 0;                                       /* previous robot y position */
  double diff_rob_pos;                                          /* difference between current and last robot y position */
  int stop_driving = 0;                                         /* flag to set the right velocity command */


  fp_control.start_parking_task.data = 0.0;                     /* signal to control the execution of this node */
  
  while (ros::ok() && fp_control.start_parking_task.data != 2)  /* control the execution time of this node */
  {
  
    if(fp_control.control_Mode.data==0)
    {
      initial_distance = fp_control.val_front[2];               /* in manually mode continuously check the distance to the next object on the left side */
      //ROS_INFO("Manually Control!, Distance: %f", initial_distance);
    }
    else
    {
      if(fp_control.control_Brake.data==1)
      {
        fp_control.control_servo.x=1500;
        fp_control.control_servo.y=1500;
      }
      else
      {
        //ROS_INFO("Automatic Control!");
        diff_distance = initial_distance - fp_control.val_front[2];             // calculate difference between initial_distance and current distance to wall/object
        //act_rel_rob_pos = start_pos_distance_driving - fp_control.rob_pos_y;
        diff_rob_pos = rob_pos_old - fp_control.rob_pos_y;                      // calculate difference in robots y position
        

        double diff_front = fp_control.val_front[0] - fp_control.val_front[2];  // calculate the difference between the penultimate and the current sensor reading (front laser scanner)
        double diff_back = fp_control.val_back[0] - fp_control.val_back[2];     // calculate the difference between the penultimate and the current sensor reading (back laser scanner) 
        
        
        if (diff_front > 0.02 || diff_front < -0.02)                            // if the penultimate and the current sensor reading differ for more than 2 cm
          {
            initial_distance = fp_control.val_front[2];                         // set the current distance to the new initial distance
            fp_control.val_front[1] = fp_control.val_front[2];                  // and adapt the previous distance to the new initial distance (necessary for the next iteration)         
            //fp_control.val_front[0] = fp_control.val_front[2];
            ROS_INFO("Initial Distance changed, diff_distance: %f", diff_front);
            ROS_INFO("Actual Distance: %f", fp_control.val_front[2]);
          }

          
        if (j == 0)                                                              // just for control purpose
        {
            ROS_INFO("diff_distance: %f", diff_distance);
        }
        
        
        /* If the current difference is greater than 20 cm or if it is greater than 10 cm and one of the last two differences is also greater than 10 cm, 
        the first edge of the first box is detected. */
        if (j == 0 && (diff_front > 0.2 || (diff_front > 0.1 && (diff_front_prev > 0.1 || diff_front_prev_prev > 0.1))))
        {
          ROS_INFO("First parking object detected");
          j++;                                                                  // increase the state counter

        }
        
        
        /* If the current difference is lower than -20 cm or if it is lower than -10 cm and one of the last two differences is also lower than -10 cm, 
        the second edge of the first box is detected. */
        if (j == 1 && (diff_front < -0.2 || (diff_front < -0.1 && (diff_front_prev < -0.1 || diff_front_prev_prev < -0.1))))
        {
          ROS_INFO("Beside first parking object");
          fp_control.find_min = 1;                                              // now calculate the distance to the next box by finding a local distance minimum 
          fp_control.min = 0;                                                   // preset the angle of the local minimum to zero
          j++;
        }
        
        
        /* After the calculations for finding the local minimum are done find_min is reset to zero and the distance to a second box can be calculated.*/
        if (j == 2 && fp_control.find_min == 0)
        {
          double space = fp_control.val_front[0] * tan(double(fp_control.min)/180 * PI);  // calculate the distance between the two boxes 
          ROS_INFO("Calculated angle: %d", fp_control.min);
          ROS_INFO("Distance between parking objects: %f",space);

          /* For control purposes the smoothed sensor data from the front laser scanner is written to a .txt file. */
          fstream f;
          f.open("/home/tas_group_15/catkin_ws/src/tas_car/find_parking_space/test.txt", ios::out);
          f << "Dieser Text geht in die Datei" << endl;
          for (int i = 0; i < 90; i++)
          {
             f <<  fp_control.scan_data[i] << endl;
          }
          f.close();

          /* If a second box is found and the space between the boxes is big enough for parking, the car should be placed beside the second box. 
          Unfortunately there was no time left to check this functionality, so the robot is searching for a second box in any case.*/
          if (space > 0.70)
          {
            ROS_INFO ("Second parking object detected");
            j++;
          }
          else
          {
            ROS_INFO ("Only one parking object found");
            j++;
            //j = j + 4;                                          // this case is disabled
            //start_pos_distance_driving = fp_control.rob_pos_y;  // code to drive for a relative distance, not used in the actual program
          }
        }

        
        if (j == 3)                                             // just for control purpose
        {
            ROS_INFO("diff_distance: %f", diff_distance);
        }

        
        /* If the current difference is greater than 20 cm or if it is greater than 10 cm and one of the last two differences is also greater than 10 cm, 
        the first edge of the second box is detected. */
        if (j == 3 && (diff_front > 0.2 || (diff_front > 0.1 && (diff_front_prev > 0.1 || diff_front_prev_prev > 0.1))))
        {
            ROS_INFO("Front Sensor detected second parking object");
            j++;
            stop_driving = 1;                                   // set the velocity to zero (the cars velocity will slow down)
        }

        
        /* Here different stop positions were checked. Even if not commented this code will not be executed in the actual program */
        /*if (j==4)
        {
            ROS_INFO("diff_back:",diff_back);
        }*/
        if (j == 4 && (diff_front < -0.2 || (diff_front < -0.1 && (diff_front_prev < -0.1 || diff_front_prev_prev < -0.1))))
        //if (j == 4 && (diff_back > 0.2 || (diff_back > 0.1 && (diff_back_prev > 0.1 || diff_back_prev_prev > 0.1))))
        {
          ROS_INFO("Beside second parking object");
          j++;
          //stop_driving = 1;
          start_pos_distance_driving = fp_control.rob_pos_y;    // code to drive for a relative distance, not used in the actual program
        }
        
        
        /*
        It was the idea to drive for a relative distance for example if there is no second box detected. Unfortunately there was no time left to fully
        implement and check this code.
        if (act_rel_rob_pos == -0.15 && j == 5)
        {
          stop_driving = 1;
          ROS_INFO("Reached start position for parking procedure");
          j++;
        }
        
        if (act_rel_rob_pos == -1.05 && j == 6)
        {
          stop_driving = 1;
          ROS_INFO("Reached start position for parking procedure");
        }*/

        
        if (j==4)                                      // just for control purpose
        {
            ROS_INFO("waiting_on_stop");
        }
        
        
        /* During the slow down of the car its position is continuously checked. When the difference in the robots y position becomes lower than 1 mm 
        the car is expected to stand.
        The y position should always change while driving, even if the robot is wrongly located in the map. */
        if (j == 4 && diff_rob_pos < 0.001 && diff_rob_pos > -0.001)
        {
           fp_control.start_parking_task.data = 1.0;    // allow publishing the start signal for the parking node
           ROS_INFO("Find parking space finished");
           j++;

        }

        diff_front_prev_prev = diff_front_prev;         // update the penultimate difference from front laser scanner
        diff_front_prev = diff_front;                   // update the previous difference from front laser scanner
        diff_back_prev_prev = diff_back_prev;           // update the penultimate difference from back laser scanner
        diff_back_prev = diff_back;                     // update the previous difference from back laser scanner
        rob_pos_old = fp_control.rob_pos_y;             // update the previous robot y position 
      }
      
// Control servos:

      if (stop_driving == 0)
      {
        fp_control.control_servo.x = 1550;
      }
      else
      {
        fp_control.control_servo.x = 1500;
      }
      
      diff_distance_steering = initial_distance - fp_control.val_front[2];      // car is steering even if the velocity command is set to zero
      fp_control.control_servo.y = 1500 - 500/0.07 * diff_distance_steering;    // calculate the servo command for steering (factor determined experimentally)

// Publish messages:

      if (fp_control.start_parking_task.data == 1.0)
      {
          fp_control.start_parking_task.data = 2.0;                               // set shut down signal for this node
          fp_control.start_parking_task_.publish(fp_control.start_parking_task);  // publish start signal for parking node
      }

      
      fp_control.control_servo_pub_.publish(fp_control.control_servo);            // publish servo commands
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
