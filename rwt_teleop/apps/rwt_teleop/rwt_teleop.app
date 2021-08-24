description: PR2 teleop via web browser
display: rwt_teleop
icon: rwt_teleop/rwt_teleop.png
interface: rwt_teleop/rwt_teleop.interface
launch: rwt_teleop/rwt_teleop.xml
platform: pr2
plugins:
  - name: kinect_head_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: pick_object_kinect_head.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /kinect_head/rgb/image_rect_color
      video_height: 480
      video_width: 640
      video_framerate: 30
      video_encoding: BGR
  - name: human_pose_estimator_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: pick_object_kinect_head_human_pose_estimator.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /edgetpu_human_pose_estimator/output/image
      video_height: 480
      video_width: 640
      video_framerate: 15
      video_encoding: RGB
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: pick_object_rosbag.bag
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
        - /audio
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/pick_object_kinect_head.avi
        - /tmp/pick_object_kinect_head_human_pose_estimator.avi
        - /tmp/pick_object_rosbag.bag
      upload_file_titles:
        - pick_object_kinect_head.avi
        - pick_object_kinect_head_human_pose_estimator.avi
        - pick_object_rosbag.bag
      upload_parents_path: pr2_fridge_pick_object
      upload_server_name: /gdrive_server
  - name: user_speech_notifier_plugin
    type: app_notifier/user_speech_notifier_plugin
    plugin_args:
      client_name: /robotsound
      warning: true
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /robotsound
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: PR2 fridge pick object demo
      use_timestamp_title: true
    plugin_arg_yaml: /var/lib/robot/pr2_mail_notifier_plugin.yaml
plugin_order:
  start_plugin_order:
    - kinect_head_video_recorder_plugin
    - human_pose_estimator_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - user_speech_notifier_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
  stop_plugin_order:
    - kinect_head_video_recorder_plugin
    - human_pose_estimator_video_recorder_plugin
    - rosbag_recorder_plugin
    - gdrive_uploader_plugin
    - user_speech_notifier_plugin
    - speech_notifier_plugin
    - mail_notifier_plugin
