# Person Lead Demo

![20210411_spot_highlight](https://user-images.githubusercontent.com/9410362/114382037-d7209a00-9bc6-11eb-9220-4cf1881696ac.gif)

This demo enables Spot to lead person to a specified place.

## Prerequities

This demo requires packages below.

- [spot_ros]() with [this patch](https://github.com/clearpathrobotics/spot_ros/pull/25)
- [common_msgs]() with [this patch](https://github.com/ros/common_msgs/pull/171)
- [jsk_recognition]() with [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2579) and [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2581)
- [jsk_spot_startup]() with [this patch](https://github.com/jsk-ros-pkg/jsk_robot/pull/1325)

## How to run

Before running this demo, please launch and prepair a controller.

- `jsk_spot_bringup.launch` in `jsk_spot_startup` ( for spot_driver and insta360air driver )
- `object_detection_and_tracking.launch` in `jsk_spot_startup` ( for result boudning box of object detection )

And then, please run

```bash
roslaunch spot_person_leader demo.launch
```

This will launch an action server for demo. So you can start the demo by publish a goal to the server.

```bash
rostopic pub
```

## Nodes

### lead_person_demo.py

#### Subscriber:

- `~bbox_array` (message type: jsk_recognition_msgs/BoundingBoxArray)

recognition results which contains a position of a person.

#### Publisher:

- `~cmd_vel` (message type: geometry_msgs/Twist)↲

spot_driver's subscribed topic of cmd_vel

#### Service Client:

- `~list_graph` (service type: spot_msgs/ListGraph)
- `~set_localization_fiducial` (service type: spot_msgs/SetLocalizationFiducial)
- `~upload_graph` (service type: spot_msgs/UploadGraph)

spot_driver's services

#### Action Client:

- `~navigate_to` (action type: spot_msgs/NavigateTo)

spot_driver's action

#### Action Server:

- `~lead_person` (action type: spot_msgs/LeadPerson)

action server of this demo.

#### Parameters:

- `~dist_visible` (float, default: 5.0)

Threshold of distance for whether a person is considered as visible or not

- `~duration_timeout` (float, default: 0.1)

Timeout duration for transform lookingup.

- `~label_person` (int, default: 0)

Label id for a person in bouding box messages.

- `~frame_id_robot` (str, default: 'body')

frame_id for robot.

- `~list_navigate_to` (list of list of str, default: [])

list of graphnav files and its start and end name.
e.g.

```
[['73B2','Elevator','<path to graphnav directory>'],
 ['73B2','81C1',<path to graphnav directory>'],
 ['81C1','Elevator','<path to graphnav directory>']]
```
