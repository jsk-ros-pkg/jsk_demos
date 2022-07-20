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
roslaunch elevator_operation elevator_operation.launch input_topic_points:=<point cloud topic> robot_type:=<robot type> launch_switchbot_client:=<launch switchbot client if necessary> switchbot_token_yaml:=<path to switchbot token yaml>
```

Then call action.

```bash
rostopic pub -1 TODO
```

## How to run on Fetch1075

```bash
roslaunch elevator_operation elevator_state_publisher.launch device_type:=fetch1075_enviii robot_type:=fetch
```

```bash
roslaunch elevator_operation elevator_operation.launch input_topic_points:=/head_camera/depth_registered/points robot_type:=fetch
```

## How to configure

To configure door frames, please edit [`doors_frame_publisher.launch`](./launch/doors_frame_publisher.launch).
