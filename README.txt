RUNNING INSTRUCTION FOR AUTONOMOUS DRIVING

execute the following commands on the terminal:
$ roscd tas/launch
$ roslaunch hardware.launch
$ roslaunch move_base.launch
$ roslaunch odom.launch
$ rosrun simple_navigation_goals simple_navigation_goals_node
$ rosrun initilisation_position compute_initionalPose

RUNNING INSTRUCTION FOR AUTONOMOUS PARKING

execute the following commands on the terminal:
$ roscd tas/launch
$ roslaunch hardware.launch
$ roslaunch move_base_fp.launch
$ roslaunch odom.launch

RUNNING INSTRUCTION FOR PARALLEL PARKIN ALONE

execute the following commands on the terminal:
$ roscd tas/launch
$ roslaunch hardware.launch
$ roslaunch move_base_park.launch


--------------------------------------------------------------------
NODE DESCRIPTION
--------------------------------------------------------------------

INITIALISATION_POSITION (Andreas Pfeuffer)

- computing_initionalPose:
		If the C-botton of the WII is pressed, this node starts to determine the 
		initional position of the car. For this, both laser sensors are used. First, 10 messages
		from each laser sensor are used to determine the filtered estimate of the
 		current	environment of the corner. The result is then saved in a point
		cloud. Then, the point cloud is compared with all stored point clouds of 
		the directory samples by means of the ICP Algorithm. It is assumed that
		the car starts from the corner, which point cloud has the minimal Eucleadian
		Error to the actual point cloud. After this, the corresponding corner
		information are read in and the initional pose can be send to the Navigation
		Stack. Furthermore, the corresponding corner is send to the simple_navigation_node
		that this node can set the waypoints such that the car can drive through the
		floor starting in the corresponding corner.

- displaying_laser_data:
		a node, which displays the sensor data of the laser scanner
		
- getting_robot_pose:
		a node, which displays the actual position of the car

- save_CornerPointCloud: 
		@ par1: number of current measurement
		@ par2: current corner
		This node creates a point cloud of a predefined corner and stores the 
		corresponding point cloud in the file samples. It is activated by pressing 
		the C-button of the WII.

-------------------------------------------------------------------------------------------------------

SIMPLE_NAVIGATION_GOALS (Andreas Pfeuffer)

- simple_navigation_goals: 
		First, this node waits, until it gets the starting corner of the node 
		initialisation_position/computing_initionalPose. Then it plans the waypoints
		such that the car can make a round through the building

-------------------------------------------------------------------------------------------------------

VELOCITY_CALCULATION (Fabian Sonntag)

- sensor_sub:
	This is the subscriber on the front laser-scanner. It calls the function 
	LaserScan_Velocity_Calculation when a new laser-scan message is received.

- LaserScan_Velocity_Calculation:
	This function evaluates the front laser-scan message to determine if the velocity of 
	the car can be increased, and if yes, by how much.

	To do so, first, the distance to the closest obstacle in a range of -25deg to 25deg 
	from the middle is determined. This area is further divided into the core-area 
	(-15 to 15deg) and the side area(s) (-25 - -15deg and 15 - 25deg).
	This is done to enable a different valuation of these areas since the core-area should 
	be more sensitive than the side areas.
	Besides, the periphery is checked. This is the area left and right of the car
	from -90 to -25deg and from 25 to 90deg (as seen from the middle).
	The periphery was defined to be 30cm wide at -90/90deg and increasing to 1.4m at -25/25deg.

	This behaviour is coded in the next section. There are 4 possibilities:
	- obstacle is in immediate proximity to the car: car should not proceed 
		--> velocity_value of 1500 (Braking) sent to servo control.
	- obstacle within 4.8m (core-area) or within 4.4m (side-areas), periphery clear:
		--> full-speed (max_vel_value)
	- periphery clear, obstacle 1.8m - 4.8m (core-area) or 1.4m - 4.4m (side-area):
		--> velocity is increased seamlessly depending on actual distance of the obstacle and area
	- periphery not clear or obstacle detected closer than core/side-threshold:
		--> default value 1550

	To avoid abrupt acceleration, a filter on the new computed velocity_value was implemented.
	The calculated velocity_value is sent to the servo control in the tas_autonomous_control_node
	when the car drives forward.

	The code described above was tested with the car "Vettel" and worked as expected, however,
	if the maximum velocity is chosen too high, the car gets more and more problems to locate
	itself, leading to some strange behaviour, like driving into walls or just stopping.
	The car's inability to break properly poses another limitation to the maximum velocity.
	That is why the maximum velocity value is chosen relatively close to the default value.
	The car won't go much faster, but it is still able to locate itself reliably.

-------------------------------------------------------------------------------------------------------


FIND_PARKING_SPACE (Heidi Stamm)

- find_parking_space:
		This node handles the detection of the parking space and places the car beside
		the second box. Therefore the mean value of the five leftmost sensor readings 
		from the front laser scanner are continuously checked. If there is an obvious 
		difference between the mean value of current reading and penultimate reading the
		edge of a box is determined. At the second edge of the first box, the node 
		calculates the distance to the next box. When the first edge of the second box 
		is detected, the node sets the velocity command to zero and waits until the car 
		has stopped. Afterwards it publishes a start signal for the parking node and 
		shuts down.
		As a second task, this node manages driving straight ahead, parallel to the wall. 
		Before getting a drive signal from the WII-control, the node determines the 
		distance to the next object on the robots left side. While driving it permanently 
		compares the current distance on the left side with this initial distance. If 
		there is a slight difference between these two values the car is counter steering. 
		For differences greater than 2 cm the current distance is set to the new initial 
		distance. For this task it is also sufficient to evaluate the five most left 
		sensor readings. 


-------------------------------------------------------------------------------------------------------

PARKING (Tanja Weber)

- control/control.cpp :
  ~ interprets the sensor data from the sensor in the front and the rear
  	discards outlayers, sets up a moving average, searches minimum and maximum distance of the car towards the environment
  ~ defines the state of the parking state model
	checks if the conditions have been fulfilled and changes them concerning the laser based information
  ~ checks whether this program is allowed to start already by subsribing to the find_parking_space

- parking.cpp:
  defines actions for the different parking states (velocity and steering angle)
	alternates between 5 (6) different states to fit in the parking sport
  sends the commands concerning velocity and steering angle
