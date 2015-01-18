Task: parallel parking

RUNNING INSTRUCTION FOR PARALLEL PARKIN ALONE

execute the following commands on the terminal:
$ roscd tas/launch
$ roslaunch move_base_park.launch

RUNNING INSTRUCTION TO START PARALLEL PARKING FROM FIND_PARKING_SPOT
$ roscd tas/launch
$ roslaunch move_base_fp.launch

--------------------------------------------------------------------

CODE DESCRIPTION:

- control/control.h:
  Define the functions and variables used in this project

- control/control.cpp :
  ~ Subsribe to the following publisher: command, odometry,  wii_communication, Laserscanner Front, Laserscanner Back, Found Parking spot
  ~ Interprete the sensor data from the sensor in the front and the rear
  	Discard outlayers, set up a moving average, search minimum and maximum distance of the car towards the environment
  ~ Define the state of the parking state model
	Check if the conditions have been fulfilled and change them concerning the laser based information

- parking.cpp:
  Autonomous/manually control
  Defines action for the different parking states (velocity and steering angle)
  Communicate with …/ Publishes the commands
