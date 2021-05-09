display: PR2 fetches can from fridge
description: PR2 fetches cans from the fridge
platform: pr2
launch: detect_cans_in_fridge_201202/pr2_fridge_demo.launch
interface: detect_cans_in_fridge_201202/pr2_fridge_demo.interface
icon: detect_cans_in_fridge_201202/pr2_fridge_demo.png
plugins:
  - name: kinect_head_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: pr2_fridge_demo_kinect_head.avi
      video_topic_name: /kinect_head/rgb/throttled/image_rect_color
      video_fps: 5.0
  - name: human_pose_estimator_video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: pr2_fridge_demo_kinect_head_human_pose_estimator.avi
      video_topic_name: /edgetpu_human_pose_estimator/output/image
      video_fps: 10.0
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: pr2_fridge_demo_rosbag.bag
      compress: true
      rosbag_topic_names:
        - /rosout
        - /tf
        - /tf_static
        - /joint_states
        - /map
        - /base_odometry/odom
        - /robot_pose_ekf/odom_combined
        - /base_controller/command
        - /navigation/cmd_vel
        - /move_base_node/NavFnROS/plan
        - /move_base_node/DWAPlannerROS/global_plan
        - /move_base_node/DWAPlannerROS/local_plan
        - /move_base_node/local_costmap/costmap
        - /move_base_node/global_costmap/costmap
        - /move_base_node/global_costmap/footprint
        - /safe_teleop_base/local_costmap/costmap
        - /spots_marker_array
        - /particlecloud
        - /base_scan_throttled
        - /tilt_scan_throttled
        - /kinect_head/rgb/throttled/camera_info
        - /kinect_head/depth_registered/throttled/camera_info
        - /kinect_head/rgb/throttled/image_rect_color/compressed
        - /kinect_head/depth_registered/throttled/image_rect/compressedDepth
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/pr2_fridge_demo_kinect_head.avi
        - /tmp/pr2_fridge_demo_kinect_head_human_pose_estimator.avi
        - /tmp/pr2_fridge_demo_rosbag.bag
      upload_file_titles:
        - pr2_fridge_demo_kinect_head.avi
        - pr2_fridge_demo_kinect_head_human_pose_estimator.avi
        - pr2_fridge_demo_rosbag.bag
      upload_parents_path: pr2_fridge_demo
      upload_server_name: /gdrive_server
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /robotsound
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: PR2 fridge demo
      use_timestamp_title: true
    plugin_arg_yaml: /var/lib/robot/pr2_mail_notifier_plugin.yaml
plugin_order:
  start_plugin_order:
    - kinect_head_video_recorder_plugin
    - human_pose_estimator_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - kinect_head_video_recorder_plugin
    - human_pose_estimator_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
