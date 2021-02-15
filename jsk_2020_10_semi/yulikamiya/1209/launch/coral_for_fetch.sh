source ~/coral_ws/devel/setup.bash
roslaunch coral_usb edgetpu_human_pose_estimator.launch INPUT_IMAGE:=/head_camera/rgb/image_raw &
rqt_image_view
