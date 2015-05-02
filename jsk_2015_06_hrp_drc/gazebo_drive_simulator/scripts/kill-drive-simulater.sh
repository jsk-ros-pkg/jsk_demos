# kill openrtm
pkill -f model-loader
pkill rtcd
# kill gazebo
killall gzserver
killall gzclient
killall gazebo
# kill hrpsys_ros_bridge
pkill roscore
pkill -f Bridge
pkill rtmlaunch
pkill roslaunch
# kill vision
killall stereo_image_proc
killall state_publisher
killall buffer_server
killall image_view
killall relay
killall scan_to_cloud_filter_chain
killall static_transform_publisher
killall urdf_control_marker
killall nodelet
killall kdtree_obstacle
# kill joystick controller
killall joy_node
# kill ros
killall rosmaster
killall rosout
