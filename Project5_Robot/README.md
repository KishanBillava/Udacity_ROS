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
1. **ROS Launch Files:** Launch files are XML files that specify a collection of nodes and their parameters to be run together. They simplify the process of starting multiple nodes with specific configurations.



