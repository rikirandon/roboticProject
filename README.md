
# IntroRoboticProject

Repository for the final project of the course "Fundament of Robotic 2023/2024".

Team members: Carlotta Cazzolli, Riccardo Randon, Martina Panini. 

## Functionality

The project involves the simulation of an ur5 manipulator equipped with a ZED camera for perception. The goal is to identify objects on the table, grab them with the gripper of the manipulator and move them according to their class.

## Prerequisites

Before you begin, ensure you have Eigen lib installed:
```
sudo apt install libeigen3-dev
```

## Setup the Workspace
### Clone the Repository

```bash
git clone https://github.com/rikirandon/roboticProject.git
cd roboticProject
```
### Setup the world
- In ``roboticProject/world`` folder bring the "blocks.world" file into the ``ros_ws/install/share/ros_impedance_controller/worlds`` folder. 
- Then bring ``roboticProject/model/X1-Y2-Z2`` into ``ros_ws/install/share/ros_impedance_controller/worlds/model``.
- In ``ros_ws/src/locosim/robot_control/base_controllers/ur5_generic.py`` change at line 71 :
```
self.world_name = 'blocks.world'
```
- in ``ros_ws/src/locosim/robot_control/base_controllers/params.py`` at line 32 set True the variable:
```
'gripper_sim': True,
```


### Build the Catkin Workspace
```bash
catkin_make
nano ~/.bashrc
```
Add the path at the and of the file setup.bash.
```bash
source $HOME/roboticProject/devel/setup.bash
```
Then execute.
```bash 
source ~/.bashrc
```

## Run the Code
We need to run 3 different programs in 3 different terminal:
- The ur5_generic in ``ros_ws/src/locosim/robot_control/base_controllers/``
```
python3 ur5_generic.py
```
- The vision node in ``roboticProject/src/package/src/vision/``
```
python3 vision.py
```
- The motion_planner node 
  ```
  rosrun package motion_planner
  ```
## Directory Structure

### Directories

```bash
├── world
│    └── blocks.world
├── model
│   └── X1-Y2-Z2
├── build
├── devel
├── presentation
│   ├── report_robotics.pdf
│   └── Vroboticproject.mp4
├── src
│   └── package
│       ├── include
│       │   └── package
│       │       ├── jointStatePublisher.h
│       │       └── ur5Object.h
│       ├── src
│       │   ├── motion_planning
│       │   │   ├── jointStatePublisher.cpp
│       │   │   ├── motion_planner.cpp
│       │   │   └── ur5Object.cpp
│       │   ├── tests
│       │   │   ├── testJointStatePublisher.cpp
│       │   │   ├── test_node.cpp
│       │   │   ├── testObject.cpp
│       │   │   └── testUr5Movement.cpp
│       │   └── vision
│       │       └── vision.py
│       └── srv
│           └── coordinates.srv
└── README.md

```


