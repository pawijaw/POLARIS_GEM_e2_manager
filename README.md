# Robot sensor data management and high level planning manager

This repository contains a high level management package for the
[Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2)
developed in the Center for Autonomy at University of Illinois at Urbana-Champaign.

## Requirements

+ [Ubuntu 22.04 LTS](https://releases.ubuntu.com/focal/)
+ [ROS Noetic Ninjemys](https://wiki.ros.org/noetic)
+ [Gazebo 11](https://classic.gazebosim.org/)
+ [Qt 5](https://doc.qt.io/qt-5/)
+ [Qt 5 Charts](https://doc.qt.io/qt-5/qtcharts-index.html)

## Installation

On Ubuntu 20.04 LTS:

+ [Install ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

+ Install packages required by the *Polaris GEM e2* simulator:
```bash
	$ sudo apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
		ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
		ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator
```

+ Install other required packages:
```bash
  sudo apt install python3-catkin-tools python3-rosdep python3-rosinstall \
    python3-rosinstall-generator python3-wstool build-essential \
    qt5-default libqt5charts5-dev
``` 

+ Clone the repository and its submodules:
```bash
  git clone --recurse-submodules git@github.com:pawijaw/POLARIS_GEM_e2_manager.git
``` 

+ Build the entire workspace:
```bash
  cd POLARIS_GEM_e2_manager
  ./build.sh
``` 

+ Start everything. This will start *Polaris GEM e2* simulator in simulated time mode:
```bash
  ./start.sh
``` 

+ Alternatively, *Polaris GEM e2* simulator can be started in wall clock mode:
```bash
  ./start.sh use_sim_time:="false"
``` 

+ (Optional) Build a docker image:
```bash
  cd POLARIS_GEM_e2_manager
  docker 
``` 

## Repository structure
The repository contains the following ROS packages:
+ **Management Node** manages robot states based on sensor and sensor data status. 
+ **Sensor mock common library** implements functionality common to all sensor mocks.
+ **Battery sensor mock** provides readings of the battery state of charge (SoC).
+ **Temperature sensor mock** monitors temperature of the robot.
+ **GPS accuracy sensor** provides positioning system (GPS) accuracy readings.
  - This mock also provides the robot's planar coordinates (x, y).
        They are retrieved from Gazebo simulator and are only used in
        the indicative manner (are not used in any decision making).
+ **Emergency stop (E-Stop) sensor mock** reads the status (ON/OFF) of emergency stop button.
+ **V2x (Vehicle to everything)** provides indication of wireless (V2x) signal strength.

## Design notes
All the components are implemented as ROS nodes and communicate using the ROS framework. 

Each *sensor mock* contains two ROS interfaces:
1. *Mock sensor data profile interface* is used to configure the mock to produce user-defined data profiles.
   - This function is implemented as [ROS Service](https://wiki.ros.org/Services) and is common to all sensor mocks and implemented in the *Sensor mock common library*.

2. *Sensor Data Interface* is a ROS topic in which sensor data streams are published.
   - Sensor data interfaces are specific to every sensor type. 
     Each sensor mock defines a custom ROS message used to carry sensor-specific data.
     This will enable their customisation in the future with additional sensor-specific data streams.
	 
The *Management Node* comprises three independent functions:
1. *Robot Manager*, which itself contains two sub-functions:
   - *Sensor Data Manager* retrieves and processes relevant sensor data streams from the sensor nodes. 
     It detects error conditions which can be causes by:
      - Loss of communication with sensor nodes.
      - Sensor data falling outside of safe ranges. Those can be defined individually using [ROS Parameters](https://wiki.ros.org/Parameter%20Server)
      - Parameter keys and default values are defined in [RobotManagerDefs.h](https://github.com/pawijaw/POLARIS_GEM_e2_manager/blob/main/src/robot_manager/manager/src/ManagerDefs.h) 
   - *High Level Navigation Planner* communicates with the nodes running the navigation algorithms form the POLARIS_GEM_e2 package. The nodes have been modified to implement the approriate interfaces:
      - [Pure Pursuit]() algorithm.
      - [Stanley]() algorithm.
	 
2. *Mock Manager* configures the *Sensor Mocks* with user-defined data profiles.

3. The *Graphical User Interface* allows user interaction with *RobotManager* and *MockControl*. The
   GUI is separated from the business logic with the *RobotManager* using two
   thread-safe API calls. This enables the GUI to be replaced or headless mode being implemented in the future.
