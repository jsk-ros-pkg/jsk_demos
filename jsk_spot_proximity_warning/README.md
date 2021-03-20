# proximity_warning demo

With this demo, robot speaks warning when people are near the robot.

https://user-images.githubusercontent.com/9410362/111855412-20831e00-8968-11eb-8d02-c3d8670e7171.mp4

## How to run

This demo requires [sound_play](http://wiki.ros.org/sound_play) node with `robotsound` action and a `jsk_recognition_msgs/BoundingBoxArray` topic with person detection (assuming that person label is 0)

Please make sure your robot meets these requirements.

```bash
roslaunch jsk_spot_proximity_warning demo.launch
```

Arguments of demo.launch are set for Spot by default. Please set your configuration when using other robot.
