#detect_cans_in_fridge_201202 Package

- - -

## Overview

This is the package for the demo program of bring a can from fridge:

 - Move to the spot of the fridge front
 - Open fridge door
 - Catch can
 - Close fridge
 - Give the can to a person

Following components are used for completion of tasks:

 - [SLAM (gmapping)](http://wiki.ros.org/gmapping)
 - Fridge door recognition based on SIFT
 - Can recognition based on color histogram and euclidean clustering
 - Online action management using [SMACH](http://wiki.ros.org/smach)
 - User Interface for touch screen devices

## Usage

This package can be executed both on simulation and on real robot.

### On Simulation

If you want to execute on real robot, see On real robot.

#### Installation

**NOTE** `hydro` distribution is assumed. Please replace it if you want to use on other distribution.

**NOTE** Assumed that catkin workspace has been installed on your environment. If you don't yet have catkin workspace, please follow [the instruction](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

**NOTE** In simulation mode, the fridge front spot is referenced from `models/room73b2-scene.l` in [euslisp](http://github.com/euslisp/EusLisp) package. So, Please make sure to build [euslisp](http://github.com/euslisp/EusLisp) package.

```bash
source /opt/ros/hydro/setup.bash
cd /path/to/your_catkin_ws
mkdir src
wstool init src
wstool set jsk_demos -t src --git https://github.com/jsk-ros-pkg/jsk_demos
wstool update -t src
rosdep install --from-paths src --ignore-src -r -n -y
catkin build detect_cans_in_fridge_201202
source /path/to/your_catkin_ws/devel/setup.bash
```

#### Launch demo

- kinematics only

1. open terminal, then execute following commands

  ```bash
# Terminal 1
roscore
```

  ```bash
# Terminal 2
roseus
(setq *logging* nil)
(load "package://detect_cans_in_fridge_201202/euslisp/main.l")
(demo)
```

2. Now you will see `pr2 Kinematics Simulator` window, and PR2 robot on it.

<img src="https://gist.githubusercontent.com/h-kamada/55fd2aae53c0e5dc65a8/raw/304b8554dfb94adef85b5f4fc44239c551df6302/move.png" width="300" height="300" />

<img src="https://gist.githubusercontent.com/h-kamada/55fd2aae53c0e5dc65a8/raw/304b8554dfb94adef85b5f4fc44239c551df6302/open.png" width="300" height="300" />  

<img src="https://gist.githubusercontent.com/h-kamada/55fd2aae53c0e5dc65a8/raw/304b8554dfb94adef85b5f4fc44239c551df6302/grasp.png" width="300" height="300" />  

<img src="https://gist.githubusercontent.com/h-kamada/55fd2aae53c0e5dc65a8/raw/304b8554dfb94adef85b5f4fc44239c551df6302/close.png" width="300" height="300" />  

### On Real Robot

This package is assumed to use PR2 robot.
To connect to PR2, set some ENV in shell:

```bash
export ROS_MASTER_URI=http://<PR2 IP>:11311
export ROS_HOSTNAME=<workstation IP>
export ROS_IP=<workstation IP>
```

#### Launch Fridge Demo

Then now you can execute fridge demo.

In PR2 internal pc, you can launch fridge demo with one launch file.

  ```bash
# on PR2
roslaunch detect_cans_in_fridge_201202 startup.launch

```

Then you can start fridge demo from tablet.