# gazebo_drive_simulator

## Introduction
Provides simple vehicle task simulation environment in gazebo.

## Depend
This program needs drcsim_hydro.
See and install as http://gazebosim.org/tutorials?tut=drcsim_install&cat=drcsim

## Setup
First, build gazebo_drive_simulator package.
```bash
$ catkin build gazebo_drive_simulator
```

You have to make world file for staro in gazebo to use staro in drive simulation environment.
```bash
rosrun hrpsys_gazebo_tutorials convert_drc_world.sh
```

Before launch gazebo, you should setup environmental valiables.
I recommend you to write them in your .bashrc.
```bash
$ source `rospack find hrpsys_gazebo_general`/setup.sh
$ export VRC_CHEATS_ENABLED=1
```

You should change gazebo field from trial task1  to final task1
```bash
$ roscd gazebo_drive_simulator/patch
$ sudo patch -p0 < drc_final_task_1.patch
```


## Execute
### launch simulation
If you want to use atlas with handle controller, execute following command (handle_controller mode):
```bash
$ rosrun gazebo_drive_simulator start_drc_practice_task_1.sh true
```

If you want to use atlas with robot driving controller in euslisp, execute following command (euslisp mode):
```bash
$ rosrun gazebo_drive_simulator start_drc_practice_task_1.sh false
$ roslaunch drc_task_common vehicle.launch ROBOT:=<robotname>
## option
# for OCS
# LAUNCH_PS3JOY (default:false)
# LAUNCH_STEERING (default:false)
# LAUNCH_UI (default:false)
# CONTROLLER_DEV (default:/dev/input/js0)

# for FC
# LAUNCH_EUS (defalut:true)
# OCS_NS (default:/ocs)
```
robotnameche does not matter in euslisp mode.
When you want to use ps3joy locally (not OCS) as robot controller, pass LAUNCH_PS3JOY:="true" and OCS_NS:="/" to vehicle.launch.
When you want to use driving_force_gt locally (not OCS) as robot controller, pass LAUNCH_STEERING:="true" and OCS_NS:="/" to vehicle.launch.

If you want to use staro, execute following command:
```bash
$ roslaunch hrpsys_gazebo_tutorials drc_practice_task_1_staro.launch
$ rosrun gazebo_drive_simulator start_staro_drive_simulator.sh
```
staro have not support euslisp mode yet.
### move handle and pedal of drc_vehicle in gazebo
You can move drc_vehicle in simulator by driving_force_gt pro in handel_controller mode.
Please make sure that handle contorller is connected as /dev/input/js0.
```bash
$ jstest /dev/input/js0
```

You can also move drc_vehicle in simulator by robot-driving-controller in euslisp mode.
In euslisp mode, same user interface as real drc can be used.

If you want to move vehicle without handle_controller and euslisp, you can send following command.
```bash
$ rostopic echo /drc_vehicle_xp900/hand_wheel/cmd std_msgs/Float64 "{data: <rad>}"
$ rostopic echo /drc_vehicle_xp900/gas_pedal/cmd std_msgs/Float64 "{data: <percentage>}"
```
/drc_vehicle_xp900/gas_pedal/cmd requires a value from 0.0 to 1.0.

### use recognition in gazebo
If you want to use recognition in gazebo_drive_simulator, you should execute following command:
```bash
$ rosrun drc_task_common drive_recognition.sh simulation
```

If the point cloud by stereo_image_proc is poor, you can change shadow of obstacles or pattern of ground:
If you want to use recognition in gazebo_drive_simulator, you should execute following command:
```bash
$ roscd gazebo_drive_simulator/patch

# change ground
$ sudo patch -p0 < drc_ground.patch
# remove shadows of obstacle
$ patch -p0 < obstacle_shadows.patch

# return this change
$ sudo patch -p0 -R < drc_ground.patch
# return shadows of obstacle
$ patch -p0 -R < obstacle_shadows.patch 
```
