# elevator_operation

This package provides elevator operation and recognition interface to robot.

## Setup

Basically, dependencies are installed with rosdep.

If you want to use spinal as altitude input, please install `spinal` package from jsk_aerial_robot. https://github.com/jsk-ros-pkg/jsk_aerial_robot/tree/master/aerial_robot_nerve/spinal

## How to run

Connect spinal or M5Stack_ROS ENVIII to PC and start `elevator_state_publisher.launch`

```bash
roslaunch elevator_operation elevator_state_publisher.launch device_type:=<spinal or enviii> device_name:=<port name> robot_type:=<robot_type, default is fetch>
```

And run elevator_operation node

```bash
roslaunch elevator_operation elevator_operation.launch
```

Then call action.

```bash
rostopic pub -1 TODO
```

## How to configure

To configure door frames, please edit [`doors_frame_publisher.launch`](./launch/doors_frame_publisher.launch).
