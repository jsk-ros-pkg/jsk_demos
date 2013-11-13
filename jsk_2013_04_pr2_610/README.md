#jsk_2013_04_pr2_610 Package

- - -

## Overview

This is the package of the demo programs which includes various home-assistant tasks:

 - Picking/Placing tray with dual arm
 - Picking soft clothes with arm
 - Manipulating laundry machine
 - Picking a broom and cleaning room
 - Moving a chair
 
All tasks are planned and dynamically executed automatically with PDDL(Problem Domain Description Language) and smach(task/action management system).

## Usage

### Connect to robot

This package is assumed to use PR2 robot.
To connect to PR2, set some ENV in shell:

```bash
   $ export ROS_MASTER_URI=http://<PR2 IP>:11311
   $ export ROS_HOSTNAME=<workstation IP>
   $ export ROS_IP=<workstation IP>
```

### Launch All Demo Tasks

Then now you can execute all demo plans.

In PR2, you must run all detection programs:

```bash
    roslaunch jsk_2013_04_pr2_610 detect_all.launch
```

In workstation PC, you can launch all programs(planner, smach, tasks, detections) in one launch file:

```bash
   $ roslaunch jsk_2013_04_pr2_610 planner.launch
```

