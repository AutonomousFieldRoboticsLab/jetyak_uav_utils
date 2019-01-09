# Jetyak UAV Utilities

***Aiming to provide an interface for the interaction of UAV and jetyaks***
This Documentation is not complete. It does not reflect recent changes nor the entire scope of the project.
## Basic Command Palette

### Startup

#### M100
* Terminal on the Manifold
	* source the workspace
	* ```sudo -s```
	* ```roslaunch jetyak_uav_utils visionAndSDKM.launch```
* Terminal Anywhere (must maintain connection or use screen)
	* source the workspace
	* ```roslaunch jetyak_uav_utils m100_controller.launch```
		* This will give an error if you do not have an SD mounted and configured in the launch file


#### N3
* Terminal on the Manifold
	* source the workspace
	* ``` sudo -s ```
	* ```roslaunch jetyak_uav_utils visionAndSDKN3.launch```
* Terminal Anywhere (must maintain connection or use screen)
	* source the workspace
	* ```roslaunch jetyak_uav_utils n3_controller.launch```
		* This will give an error if you do not have an SD mounted and configured in the launch file

### Modes and changing

The modes are: takeoff, follow, leave, return, land, ride, and hover. They may be changed by calling ```rosservice call /jetyak_uav_utils/setMode "data: '<MODE>'"``` and replacing <MODE> with the mode. This is case insensitive.

#### Brief explanation of each behavior
takeoff
* Start the UAVs motors by calling the ```prop_enable``` or ```takeoff``` service.
* Lifts off the platform either by the action taken by the lower level controller in ```takeoff``` or using tags after ```prop_enable``` is called.
 
follow
* Follows at a certain pose relative to the tags. This pose may be changed through the ```setFollowPosition``` service.

leave
return
land
ride
hover

## Prerequisites
Here listed a few dependencies that need to be downloaded from the corresponding
repository.

* DJI Onboard SDK
```
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

* DJI Onboard SDK ROS from the catkin_ws/src directory
```
git clone https://github.com/dji-sdk/Onboard-SDK-ROS
catkin_make -C ..
```

* ar\_track\_alvar
```
sudo apt install ros-<VERSION>-ar-track-alvar
```
