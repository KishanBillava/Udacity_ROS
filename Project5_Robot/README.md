# Udacity_ROS
Udacity Robotics Software Engineer Nanodegree


### ROS Packages Folders and Files Structure for Home Service Robot Project 
```sh
├── add_markers
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── add_markers.launch
│   │   └── markers_time.launch
│   ├── package.xml
│   └── src
│       ├── add_markers.cpp
│       └── markers_time.cpp
├── CMakeLists.txt
├── map
│   ├── map.pgm
│   ├── map.yaml
│   └── slam_test.world
├── pick_objects
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── pick_objects.launch
│   ├── package.xml
│   └── src
│       └── pick_objects.cpp
├── README.md
├── rvizConfig
│   ├── home_service.rviz
│   ├── nav_test.rviz
│   └── slam_test.rviz
├── scripts
│   ├── add_markers.sh
│   ├── home_service.sh
│   ├── pick_objects.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── slam_gmapping
│   ├── gmapping
│   └── slam_gmapping
├── turtlebot
│   └── turtlebot_teleop
├── turtlebot_interactions
│
└── turtlebot_simulator
    ├── turtlebot_gazebo

```
### Overview  

1. **ROS Launch Files:** Launch files are XML files that specify a collection of nodes and their parameters to be run together. They simplify the process of starting multiple nodes with specific configurations.
2. **Gazebo:** Gazebo is a powerful 3D simulation environment that allows you to model, simulate, and visualize robots and environments. It's used to simulate the robot's interactions with its surroundings.
3. **RViz:** RViz is a visualization tool that provides real-time visualization of robot sensor data and other information. It's used to visualize the robot's sensor data, trajectories, and maps.
4. **ROS Navigation Stack:** The Navigation Stack is a collection of packages that enables a robot to autonomously navigate within an environment. It includes modules for path planning, localization, and obstacle avoidance.
5. **gmapping:** The gmapping package implements the GMapping algorithm, which stands for "Grid-based FastSLAM" for mapping. This package allows the robot to create a map of its environment while localizing itself within that map. It uses sensor data such as laser scans and odometry to build an occupancy grid map.
6. **add_markers:** The add_markers package is a custom package, it's used to simulate the process of picking up and dropping off an object using a virtual marker.
7. **pick_objects:** The pick_objects package is another custom package. The robot is instructed to navigate to a pickup location, simulate picking up an object, move to a drop-off location, and then simulate dropping off the object.
8. **scripts:** Contains shell scripts for running various parts of the project. `add_markers.sh`, `home_service.sh`, `pick_objects.sh`, `test_navigation.sh`, `test_slam.sh`: Shell scripts for executing different functionalities.

### Procedure for home service robot 
The `home_service.sh` contains shell scripts for running various parts of the project to create a complete home service robot scenario. It involves mapping, localization, path planning, and visualization to simulate a robot picking up an object and delivering it to a drop-off location in a home environment.

1. **Project Workspace** : commands in the workspace terminal to set it up for the project

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```

2. **Your Packages and Directories**

* **map:** Inside this directory, you will store your gazebo world file and the map generated from SLAM.
* **scripts:** Inside this directory, you’ll store your shell scripts.
* **rvizConfig:** Inside this directory, you’ll store your customized rviz configuration files.
* **pick_objects:** You will write a node that commands your robot to drive to the pickup and drop off zones.
* **add_markers:** You will write a node that model the object with a marker in rviz.
  
3. **scripts:** Contains shell scripts for running various parts of the project. `add_markers.sh`, `home_service.sh`, `pick_objects.sh`, `test_navigation.sh`, `test_slam.sh`: Shell scripts for executing different functionalities.

4. **Building Editor in Gazebo**
* Copy the world from the build Editor project
* generate a map using `pgm_map_creator` edit the map.yaml file
![image](https://github.com/KishanBillava/Udacity_ROS/assets/84302215/d33121aa-3759-46b2-a9a4-6f9a4dc48480)


5. **SLAM Testing**
* create a `test_slam.sh` shell script that launches that deploys turtlebot_gazebo world using your  world file
* Add launch file for gmapping, navigation to view the map on rvix
* Also Add keyboard_teleop.launch to control keyboard 

![image](https://github.com/KishanBillava/Udacity_ROS/assets/84302215/cac6b7bd-f06a-484b-94f8-bd7ad729812f)


6. Localization and Navigation Testing
* create a `test_navigation.sh` shell script that launches that deploys turtlebot_gazebo world
* amcl_demo to localize the turtlebot
* Also add view_navigation to view the map on rviz
* provide the 2d nav goal and test  functionality

![image](https://github.com/KishanBillava/Udacity_ROS/assets/84302215/4b9e61ec-bc1e-4918-8f37-b1ba4d199816)


7. Navigation Goal Node
* 

8. Virtual Objects


9. Your Home Service Robot


