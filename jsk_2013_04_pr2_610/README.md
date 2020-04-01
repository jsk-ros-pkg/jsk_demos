jsk_2013_04_pr2_610
====================


## Citation

```
Yuki FURUTA, Yuto INAGAKI, Youhei KAKIUCHI, Kei OKADA, Masayuki INABA:
IRTホームアシスタントロボット による掃除片付け作業シーケンスのPR2による実現,
in The 31th Annual Conference on Robotics Society of Japan, 1I2-02, 2013.
```

## Overview

This is the package of the demo programs including various home-assistant tasks:

 - Picking/Placing tray with dual arm
 - Picking soft clothes with arm
 - Manipulating laundry machine
 - Picking a broom and cleaning room
 - Moving a chair

[![2009 IRT demo](http://img.youtube.com/vi/ToL3egTOahg/0.jpg)](https://www.youtube.com/watch?v=ToL3egTOahg "2009 IRT demo")

This demo is PR2/ROS re-written version of 2009 IRT demo in the video.

Following components are used for completion of tasks:

 - [SLAM (gmapping)](http://wiki.ros.org/gmapping)
 - SIFT based image recognition
 - Furniture leg detection using base laser data
 - [automatic task planning with PDDL(Problem Domain Description Language)](https://github.com/jsk-ros-pkg/jsk_planning/tree/master/task_compiler)
 - Online action management using [SMACH](http://wiki.ros.org/smach)
 - User Interface for touch screen devices

More detailed information is provided in the [citation](#citation)

## Installation

**NOTE** `melodic` distribution is assumed. Please replace it if you want to use on other distribution.

**NOTE** Assumed that catkin workspace has been installed on your environment. If you don't yet have catkin workspace, please follow [this instruction](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

**NOTE** A catkin workspace is created at $HOME/ros/jsk_demo_ws. ( for example, /home/leus/ros/jsk_demo_ws )

**NOTE** Following packages are currently not released in melodic. So this instruction will install them from source.

- [jsk_planning](https://github.com/jsk-ros-pkg/jsk_planning)
- [pr2_gripper_sensor_msgs](https://github.com/pr2/pr2_gripper_sensor)

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/ros/jsk_demo_ws/src -p
cd ~/ros/jsk_demo_ws
catkin init
wstool init src
wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_demos/tree/master/jsk_2013_04_pr2_610/jsk_2013_04_pr2_610.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src -r -n -y
catkin build jsk_2013_04_pr2_610
source ~/ros/jsk_demo_ws/devel/setup.bash
```

## Usage

This package can be executed both on simulation and on real robot.

### Launch demo on simulation

If you want to execute on real robot, see [below](#On-Real-Robot)


#### kinematics only

1. open terminal, then execute following commands

```bash
# Terminal 1
roscore
```

```bash
# Terminal 2
rosrun jsk_2013_04_pr2_610 demo.l "(demo)" # please ignore all error message
```

2. Now you will see `pr2 Kinematics Simulator` window, and PR2 robot on it.

  <img src="https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/5247c78b283030af0ddc66d7c42ae911e5d06bd7/jsk_2013_04_pr2_irt_1.png" width="200" height="200" />
  <img src="https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/9584321f8b5069d056e145752c3ecc8a1026babf/jsk_2013_04_pr2_irt_2.png" width="200" height="200" />
  <img src="https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/2eb3ca13d1b7ac2019da5ca3778fcc28afa3a92f/jsk_2013_04_pr2_irt_3.png" width="200" height="200" />
  <img src="https://gist.githubusercontent.com/k-okada/b3308c08ce31230e8947/raw/c14d6c52d8bf35fd5c244d989beccd35caa6fa8a/jsk_2013_04_pr2_irt_4.png" width="200" height="200" />

#### Kinematics + planning

1. you can launch `demo_<type>.launch` with option `use_sim:=true`

```bash
# Terminal 1
roslaunch jsk_2013_04_pr2_610 demo_tray.launch use_sim:=true
# replace `tray` with `wash` or `all` if you'd like to see other demos.
```

#### Kinematics + planning + planning visualization

1. execute command below

```bash
# Terminal 1
roslaunch jsk_2013_04_pr2_610 planner.launch
```

In this demo, you can see more described information about planning.

  - state transition graph

    ![graph](https://gist.githubusercontent.com/furushchev/ea64ba5949b0f41b7400/raw/abdba86ac3b56ffc0b6204b65408a02ca8a616dd/pddl_graph.png)
  - generated plan (`PDDL Planner Viewer`)

    ![pddl_viewer](https://gist.githubusercontent.com/furushchev/ea64ba5949b0f41b7400/raw/abdba86ac3b56ffc0b6204b65408a02ca8a616dd/pddl_viewer.png)
  - action state transition graph (`SMACH Viewer`)

    ![smach_viewer](https://gist.githubusercontent.com/furushchev/ea64ba5949b0f41b7400/raw/abdba86ac3b56ffc0b6204b65408a02ca8a616dd/smach.png)

### Launch demo on real robot

This package is assumed to use PR2 robot.
To connect to PR2, set some ENV in shell:

```bash
export ROS_MASTER_URI=http://<PR2 IP>:11311
export ROS_HOSTNAME=<workstation IP>
export ROS_IP=<workstation IP>
```

#### Launch All Demo Tasks

Then now you can execute all demo plans.

In PR2 internal pc, you can launch all demos with one launch file.

```bash
# on PR2
roslaunch jsk_2013_04_pr2_610 demo_all.launch
```
