# Person Follow Demo

<TODO image>

This demo enables Spot to follow person.

## Prerequities

This demo requires packages below.

- [spot_ros]() with [this patch](https://github.com/clearpathrobotics/spot_ros/pull/25)
- [common_msgs]() with [this patch](https://github.com/ros/common_msgs/pull/171)
- [jsk_recognition]() with [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2579) and [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2581)
- [jsk_spot_startup]() with [this patch](https://github.com/jsk-ros-pkg/jsk_robot/pull/1325)

## How to run

Before running this demo, please launch and prepair a controller.

- `jsk_spot_bringup.launch`
- `object_detection_and_tracking.launch`
- `multi_object_detector.launch`

And then, please run

```bash
roslaunch spot_person_follower demo.launch
```

After this, you can start following behavior by pressing L2 button of the controller.
Spot will follow the nearest person at the time of pressing.
If you want to stop the behavior, please press the L2 button again.
