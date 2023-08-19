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

### Overview 
