source ~/coral_ws/devel/setup.bash
roslaunch coral_usb edgetpu_human_pose_estimator.launch INPUT_IMAGE:=/camera/color/image_raw &
rqt_image_view
