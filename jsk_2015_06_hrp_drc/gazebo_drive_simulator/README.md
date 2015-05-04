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
$ source `rospack find hrpsys_gazebo_tutorials`/setup.sh
$ export VRC_CHEATS_ENABLED=1
``

## Execute
### start atlas simulation
```bash
$ rosrun gazebo_drive_simulator start_drc_practice_task_1.sh
```

### start staro simulation
`` bash
$ roslaunch hrpsys_gazebo_tutorials drc_practice_task_1_staro.laucnh
$ rosrun gazebo_drive_simulator start_staro_drive_simulator.sh
```

### move handle and pedal of drc_vehicle in gazebo
You can move drc_vehicle in simulator by driving_force_gt pro.
Please make sure that handle contorller is connected as /dev/input/js0.
```bash
jstest /dev/input/js0
```

If you want to move vehicle without handle controller, you can send following command.
```bash
rostopic echo /drc_vehicle_xp900/hand_wheel/cmd std_msgs/Float64 "{data: <rad>}"
rostopic echo /drc_vehicle_xp900/gas_pedal/cmd std_msgs/Float64 "{data: <percentage>}"
```
/drc_vehicle_xp900/gas_pedal/cmd requires a value from 0.0 to 1.0.