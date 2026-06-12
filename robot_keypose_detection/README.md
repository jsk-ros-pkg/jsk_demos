# Robot KeyPose Detection

## Prerequisite

```
pip install cameramodels
pip install scikit-robot
pip install chainer-openpose
```

## build

In your catkin workspace,

```
catkin build robot_keypose_detection
```

## Launch

First, setup external camera. In this tutorial, we use D415/D435 RealSense camera.

```
roslaunch robot_keypose_detection external_camera_tf_publisher.launch input_image:=/camera/color/image_raw input_camera_info:=/camera/color/camera_info
```

## Example

![Example-01](./docs/example-01.gif)

![Example-02](./docs/example-02.gif)
