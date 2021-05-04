# SPOT watchdog demo

## usage

### base process:

```bash
$ roslaunch jsk_spot_startup jsk_spot_bringup.launch # please follow https://github.com/sktometometo/jsk_robot
$ roslaunch jsk_spot_startup object_detection_and_tracking.launch # need to source a python3 based catkin-workspace (please check roslaunch jsk_spot_startup object_detection_and_tracking.launch)
```
### demo process

#### option1: ros smach based on python:

```bash
$ roslaunch jsk_spot_watch_dog watch_dog_smach.launch
```

#### option2: ros smach based on roseus:

```bash
$ roslaunch jsk_spot_watch_dog watch_dog_roseus_smach.launch
```

