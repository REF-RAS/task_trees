# Task Trees: SDK for Rapid Robot Arm Manipulation Application Development 

Task Trees is a SDK that can shorted the development cycle of behaviour-tree based robot arm manipulation applications. It is an outcome from the work on developing a model architecture for these applications. The model architecture is introduced in the report _Architecting Robot-Arm Applications with Robotic Manipulation Platforms and Behaviour Trees_ ([link](docs/assets/ArchitectingRobotApplicationsV2.pdf)).

## Requirements

The application runs in an environment with the following components:
- Ubuntu 20.04
- Python 3.8 or above
- ROS Noetic with RViz
- Moveit 1
- Pytree 2.2.3
- Jupyros (for execcuting ROS commands from Jupyter notebooks)
- UR10 Configuration and/or Panda Configuration
- The [arm_commander](https://github.com/REF-RAS/arm_commander) package

## Installation (Docker-based)

The arm_commander stems from an architecture that is robot model agnostic and arm manipulation platform agnostic. It hides away the
details and offers a consistent application programming interface. 

This implementation of arm_commander is specific to Moveit Version 1 (and therefore ROS Noetic). An environment installed with Moveit + ROS Noetic is needed. For your convenience, please refer to the docker image from the [docker deployment](https://github.com/REF-RAS/docker_deployment) repository of the REF-RAS group.


## Installation (non-Docker-based)

The non-docker method assumes the starting point of having ROS Noetic installed on Ubuntu 20.04.

### Install Moveit (Non-Docker Method) 

```
sudo apt-get update
sudo apt-get install ros-noetic-moveit -y
```

### Install Arm Commander

Create a catkin workspace.
```
mkdir -p ~/arm_commander_ws/src
cd ~/arm_commander_ws/src
```

Clone this repository to the `src` directory.
```
git clone git@github.com:REF-RAS/arm_commander.git
git clone git@github.com:REF-RAS/task_trees.git
```

### Install a Robot Arm Model

The demo program and the tutorial programs are designed to work with the dimension of a Panda.

```
cd ~/arm_commander_ws/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel 
```

### Build the Packages

```
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```


## Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Feb 2024
