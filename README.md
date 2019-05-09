<<<<<<< HEAD
# Jetyak UAV Utilities

***Aiming to provide an interface for the interaction of UAVs and ASVs***
This Documentation is not complete. It does not reflect recent changes nor the entire scope of the project.

## Prerequisites
Here listed a few dependencies that need to be downloaded from the corresponding
repository.

* DJI Onboard SDK
	1. `git clone https://github.com/dji-sdk/Onboard-SDK.git`
	2. `cd Onboard-SDK`
	3. `mkdir build`
	4. `cd build`
	5. `cmake ..`
	6. `make`
	7. `sudo make install`


* DJI Onboard SDK ROS from the catkin_ws/src directory
	1. `git clone https://github.com/dji-sdk/Onboard-SDK-ROS`
	2. `catkin_make -C ..`

* ar\_track\_alvar
	1. `sudo apt install ros-<VERSION>-ar-track-alvar`


## Running the system

### M100
* Terminal 1 on the Manifold
	* source the workspace
	* ```sudo -s```
	* ```roslaunch jetyak_uav_utils visionAndSDKM.launch```
* Terminal 2 on the Manifold or SSH
	* source the workspace
	* ```roslaunch jetyak_uav_utils m100_controller.launch```
		* This will give an error if you do not have an SD mounted and configured in the launch file


### N3
* Terminal 1 on the Manifold
	* source the workspace
	* ```sudo -s```
	* ```roslaunch jetyak_uav_utils visionAndSDKN3.launch```
* Terminal 2 on the Manifold or SSH
	* source the workspace
	* ```roslaunch jetyak_uav_utils n3_controller.launch```
		* This will give an error if you do not have an SD mounted and configured in the launch file

### Modes and changing

The modes are: takeoff, follow, leave, return, land, ride, and hover. They may be changed by calling ```rosservice call /jetyak_uav_utils/setMode "data: '<MODE>'"``` and replacing <MODE> with the mode. This is case insensitive.

#### Brief explanation of each behavior
* takeoff
	* Start the UAVs motors by calling the `prop_enable` or `takeoff` service.
	* Lifts off the platform either by the action taken by the lower level controller in `takeoff` or using tags after `prop_enable` is called.
* follow
	* Follows at a certain pose relative to the tags. This pose may be changed through the `setFollowPosition` service.
* leave
  * No output from behaviors, expects an external behavior to publish to `extCommand`
* return
  * Flys up to a certain altitude above the takeoff altitude.
  * Uses boat GPS to return to the area near the boat.
  * When near the boat, look for tags that provide the same pose difference that GPS does
  * Switch to following mode when a good tag is found and we are near the following pose
* land
  * Move into a pose relative to, just above, the landing platform.
  * When stable and in a good position, land
* ride
  * Ensure motors are off
* hover
  * hold position

## Contributing or Developing
We want continue the development of this project in a modular and robust way. Our primary way of doing this is by creating an interface between our higher level controls given in the behaviors node or external nodes and the UAV we use. In our case, we use dji_pilot to provide this interface for a DJI Matrice M100 and a HexH20 with a Naza-3. We use gimbal_tag to provide an interface to transform coordinate systems. Finally, we use [dji_gimbal_cam](https://github.com/usrl-uofsc/dji_gimbal_cam) to provide an interface to the gimbal controls and camera. These interfaces can be created for any drone.

We ask that you maintain these layers of abstraction when developing your system and contribute your interfaces here as well. If you would like to create more advanced behaviors, feel free to create them using either the behaviors node or as an external behavior (see scripts/nodes/waypoint_following.py).

Please follow good coding practices and submit your code by issuing a pull request.
=======
# jetyak_uav
This package is being developed by the Autonomous Field Robotics Lab as an interface between a GPS-enabled ground vehicle and a multicopter. 
>>>>>>> master
