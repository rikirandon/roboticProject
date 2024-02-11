
# IntroRoboticProject

Repository for the final project of the course "Fundamentals of Robotic".

Team members: Carlotta Cazzolli, Riccardo Randon, Martina Panini. 

## Functionality

The project involves the simulation of an ur5 manipulator equipped with a ZED camera for perception. The goal is to identify objects on the table, grab them with the gripper of the manipulator and move them according to their class.

## Prerequisites

Before you begin, ensure you have the following installed:

- Eigen lib

## Setup the Workspace
### Clone the Repository

```bash
git clone https://github.com/rikirandon/roboticProject.git
cd roboticProject
```
### Build the Catkin Workspace
```bash
catkin_make
nano ~/.bashrc
```
Add the path for the setup.bash.
```bash
source $HOME/roboticProject/devel/setup.bash
```
Save, exit.
```bash 
source ~/.bashrc
```
### Setup the world
Into ``roboticProject/world`` folder bring the "blocks.world" file into the ``ros_ws/src/locosim/ros_impedance_controller/worlds`` folder.
In ``ros_ws/src/locosim/robot_control/base_controllers/ur5_generic.py`` change the line 71 with:
```
self.world_name = 'blocks.world'
```
## Directory Structure

### Directories

```bash
├── README.md
└── src  # Source Directory
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    └── package
        ├── CMakeLists.txt
        ├── include
        │   └── package
        │       ├── jointStatePublisher.h
        │       └── ur5Object.h
        ├── package.xml
        ├── src
        │   ├── motion_planner #Code related to motion planning functionality
        │   │   ├── jointStatePublisher.cpp
        │   │   └── ur5Object.cpp
        │   ├── tests  #Code related to testing motion_planning funcionality
        │   │   ├── testJointStatePublisher.cpp
        │   │   ├── test_node.cpp
        │   │   ├── testObject.cpp
        │   │   └── testUr5Movement.cpp
        │   └── vision #Code related to vision functionality
        │       └── vision.py
        └── srv #Service
            └── coordinates.srv
```


