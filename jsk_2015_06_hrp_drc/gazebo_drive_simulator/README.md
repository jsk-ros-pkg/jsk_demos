# gazebo_drive_simulator

## Introduction
provides simple vehicle task simulation environment in gazebo.

## Depend
this program needs drcsim_hydro.
see and install as http://gazebosim.org/tutorials?tut=drcsim_install&cat=drcsim

## Setup
```bash
$ catkin build gazebo_drive_simulator
```
### start atlas simulation
```bash
$ rosrun gazebo_drive_simulator start_drc_practice_task_1.sh
```

### start staro simulation
First, you have to make world file for staro in gazebo.
```bash
rosrun hrpsys_gazebo_tutorials convert_drc_world.sh
```

```bash
$ source `rospack find hrpsys_gazebo_tutorials`/setup.sh
$ roslaunch hrpsys_gazebo_tutorials drc_practice_task_1_staro.laucnh
$ rosrun gazebo_drive_simulator start_staro_drive_simulator.sh
```