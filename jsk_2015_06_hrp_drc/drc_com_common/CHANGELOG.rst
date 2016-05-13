^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drc_com_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------

0.0.2 (2015-11-26)
------------------
* [drc_com_common] Add respawn flag to highspeed receivers
* [drc_com_common] Use silverhammer highspeed receiver with internalbuffer in
  order to cleanup silverhammer processes
* [drc_task_common] Omit footstep planner client in fc-executive.l
* [drc_com_common] Use jsk_robot_utils to compress/decompress joint angles
* Contributors: Ryohei Ueda

0.0.1 (2015-06-11)
------------------
* add emergency-pose button to rviz
* add line for send keypoint screen shot
* fix SIMPLE_FOOTSTEP_EXEC's number 71->72
* [drc_task_common, drc_com_common] Add simple footstep exec
* [drc_task_common, drc_com_common]  Add more dynamic reconfigure parameters
* add flag for drill throw
* [drc_task_common] Add detach messages to silverhammer
* [drc_com_common, drc_task_common] Add more basic info for jaxon
* add reset_enc and finish_stair button to rviz
* [drc_com_common] Fix port to receive dynamic reconfigure
* [drc_com_common] Disable reliable highspeed way
* [drc_task_common] Update for terrain task
* add new rviz button
* [drc_task_common] Remove locomotion topics from FC2OCSLarge
* remove bags with experiment with Jaxon Red
* Merge pull request #863 from YoheiKakiuchi/fix_compress
  remove parameter(robot_name), robot name should be defined by environ…
* remove parameter(robot_name), robot name should be defined by environment variable
* [drc_com_common] Add FC2OCSLargeReliable
* [drc_task_common, drc_com_common] Use pesimistic mode for footstep planning
* [drc_task_common, drc_com_common] Perception and planning on ocs side about terrain task
* [drc_task_common] Remove steering_diff_angle_vector from vehicle_ui
* Merge remote-tracking branch 'origin/master' into support-terrain-task-in-fc-ocs
  Conflicts:
  jsk_2015_06_hrp_drc/drc_com_common/msg/OCS2FCSmall.msg
  jsk_2015_06_hrp_drc/drc_task_common/launch/fc/locomotion_planner.launch
* [drc_task_common] Update for terrain task
* [drc_task_common] Call drive/operation services from vehicle_ui, not eus controller
* [drc_com_common, drc_task_common] Support footstep_planner and footstep_controller
* [drc_com_common, drc_task_common] Support projection of footprint
* [drc_com_common] Add tag for terrain task
* add button to reset force offset to rviz
* [drc_task_common] Add egress service to silverhammer
* add impedance and grasp message uint8
* [drc_task_common] Add ui elements for stair task
* [drc_task_common] Add current_steering, crank/handle_pose and
  predicted_path_marker to FC2OCSLarge
* [drc_task_common] Add msgs for set-real service to silverhammer
* [drc_task_common] Implement execute button, which disable joy controller and connection between vehicle-fc/ocs-executive
* [drc_com_common, drc_task_common] Add imu to basic info
* [drc_com_common] Add camera_info to large data
* [drc_task_common] Add neck_p/y_angle to silverhammer
* [drc_task_common] Change rate of executive and streamer
* add hand calib button to rviz gui
* remove stop abc/st button and start impedance soft/hard button to rviz
* [drc_task_common] Add obstcle_length to silverhammer
* [drc_task_common] Add launch-prefix and port settings for vehicle
* [drc_task_common] Integrate launch for vehicle task to main operator_station/field_computer scripts
* Add brake/neck_y/neck_p topics to silverhummer for vehicle
* [drc_task_common] Fix synchronize methods for controller in silverhummer
* [drc_com_common, drc_task_common] Support fisheye lookat
* [drc_com_common] Add topics for opration services
* [drc_task_common, drc_com_common] Cleanup launch files and support
  tmux-based launching
* [drc_task_common] Implement controller-mode services to vehicle silverhummer
* [drc_task_common] add button checker uis
* [drc_task_common] Add steering_diff_angle to vehicle-silverhummer
* [drc_task_common] Add neck_mode to msg
* [drc_task_common] Add topics for SetValue service
* [drc_task_common] Add req/res for new services
* restore lasvegas door
* add door-though-pose button
  fix trans-list of door push motion
* [drc_com_common] Add service request/response to silverhummer msgs for vehicle
* [drc_com_common] Update msgs for topics in driving-controller
* [drc_com_common, drc_task_common] Support forces and temperature in OCS side
* [drc_com_common] Compile vehicle silverhummer msgs
* [drc_com_common] Add msgs and launch for silverhummer system in vehicle task
* remove lasvegas door temporary because drc_com_common msg problem
* fix bug in adding lasvegas door
* change miss rosparam
* support reach-until-touch in teleop system
* [drc_com_common] Add roseus_remote
* [drc_com_common] Add argument to specify port for dynamic_reconfigure
* [drc_com_common] Fix IP address of server of dynamic_reconfigure tunnel
* add door name select button to optional buttons
* add button and functions to select door push/pull direction
* enable to select valve grasp mode (edge or center) from ocs ui
* Change default exposure 0.01 -> 0.1
* Change default exposure in DRCParameters
* send left/right arm information from ocs to fc and apply it to real robot motion.
* [drc_task_common, drc_com_common] add drill poses ui, change codes style a bit simpler
* Merge pull request #496 from mmurooka/modify-stand-point-manually
  [drc_task_common] change robot stand point manually in teleop motion
* enable to change robot stand point manually in teleop motion
* [drc_task_common] Add RobotHeadUI to specify joint angles of head
* add rqt qui button and ocs/fc functions to enable/disalbe head joint overwrite
* [drc_com_common] Add sudo prefix for reconfigure path
* Merge pull request #420 from garaemon/dynamic-reconfigure
  [drc_com_common, drc_task_common] Add rqt_reconfigure between ocs and fc
* [drc_com_common, drc_task_common] Add rqt_reconfigure between ocs and fc
* [drc_com_common] Set bandwidth for fast path
* [drc_task_common] Add state for driving task
* Merge pull request #400 from garaemon/not-compress-joint-angles
  [drc_task_common, drc_com_common] Do not compress joint angles from FC to OCS
* [drc_task_common, drc_com_common] Do not compress joint angles from FC to OCS
* [drc_task_common, drc_com_common] Use pointcloud respected from ground frame
* [drc_task_common]remove some bags
* Merge pull request #393 from garaemon/send-odom-coords
  [drc_task_common, drc_com_common] Relay odom frame from fc to ocs
* [drc_task_common, drc_com_common] Relay odom frame from fc to ocs
* [drc_com_common] Add laser cloud to send from FC to OCS
* cancel-motion button
* merge origin/master and modify conflict.
* change to use fc and ocs
* add enum for sending angle-vector
* [drc_task_common, drc_com_common] Support effort in basic info
* [drc_com_common] Specify bandwidth on fastpath
* [drc_com_common] Increase framerate to send image and pointcloud
* integrate drill pushing button motion to teleop system
* [drc_task_common, drc_com_common] Integrate wall detection for drill task
* [drc_task_common, drc_com_common] Add drill wall recognition
* [drc_com_common] Increase image resolution
* [drc_com_common, drc_task_common] Update minor codes to support robot_status
* [drc_com_common, drc_task_common] Change robot state type from Int32 to
  UInt8 and send robot_state in continuous low-speed path
* [drc_task_common, drc_com_common] Watch robot movement and publish the status
  by watching /fullbody_controller/joint_trajectory_action/status topic.
* Merge remote-tracking branch 'ohara_remote/add_ui_for_drill_put' into icp-param
  Conflicts:
  jsk_2015_06_hrp_drc/drc_com_common/msg/FC2OCSSmall.msg
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/fc-executive.l
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/ocs-executive.l
* add states for push
* add states for push
* [drc_com_common] Write port to be used for highspeed communication
* [drc_com_common] Use ip:=0.0.0.0 for server programs and do not use
  sudo for streamers
* [drc_com_common] Enable event driven mode for lowspeed streamers
* merge origin/master
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_demos into add_drill_interface
* remove bags in programs
* [drc_task_common, drc_com_common] Use timeout to detect failure of detection based on
  timered-state-machine
* add exec interface(not done real robot movement)
* add_recog_drill_for_grasp
* merge origin/master
* insert recog_drill msg
* [drc_task_common, drc_com_common] Remove confirmation after recognizing point to look at
* [drc_task_common, drc_com_common] Add look-around functionality
* [drc_task_common, drc_com_common] Update ocs side to use panorama view
* [drc_com_common, drc_task_common] Add perspective for panorama view
* [drc_com_common] Send panorama image to ocs
* add hose-connect motion function and integrate that motion into teleop system.
* [drc_task_common, drc_com_common, drc_valve_task] Remove catkin.cmake
* [drc_com_common] Update dependency to depend on roseus and jsk_network_tools
* [drc_com_common] Send packages slower not to be dropped
* change packet_interval. remap multisense point cloud.
* integrate hose grasping motion to teleop system
* [drc_com_common] Add script to check process which uses port of low-speed
* [drc_task_common. drc_com_common] Use 1-1023 port for continuous communication
* [drc_task_common, drc_com_common] Send compressed joint angles always as report
* [drc_com_common, drc_task_common] Send compressed joint angles from FC to OCS always
* enable to send valve motion from ocs to fc under communication limitation
* [drc_com_common] Use tunnel in default
* [drc_com_common] Fix type
* [drc_com_common] Do not use compressed image
* [drc_com_common] Use compressed image
* [drc_com_common] Publish smaller image
* [drc_com_common] Enable broad band communication
* [drc_task_common, drc_com_common] Integrate debri detection
* [drc_task_common, drc_com_common] Door handle detection is implemented
* [drc_task_common] Integrate valve detection
* [drc_task_common, drc_com_common] Add narrowband-message-handler to handle
  compact message
* send go-pos command from rviz using ocs-executive.l
* [drc_task_common] Choose Location to go by image with network limitation
* [drc_com_common, drc_task_common] Add image_view2 based user interface. first step of system integration towards DRC final
* [drc_com_common, drc_task_common] Add image_view2 based user interface. first step of system integration towards DRC final
* [drc_com_common] Add special message for narrow band from FC to OCS.
  Now it's only contains joint angles
* [drc_com_common] Add special message for narrow band from FC to OCS.
  Now it's only contains joint angles
* Merge branch 'use-jsk-recognition-msgs' of https://github.com/garaemon/jsk_demos into catkinize
* Merge branch 'use-jsk-recognition-msgs' of https://github.com/garaemon/jsk_demos into catkinize
* add cmake_modules to package.xml
* add cmake_modules to package.xml
* fix typo in drc_com_common : rosbuid -> rosbuild
* fix typo in drc_com_common : rosbuid -> rosbuild
* [drc_com_common] Add script to stream data from FC to OCS using jsk_network_tools
* [drc_com_common] Add script to stream data from FC to OCS using jsk_network_tools
* [drc_com_common] update minimaxwell IP
* [drc_com_common] update minimaxwell IP
* [drc_com_common] Add desktop icon to launch mini maxwell for drc network environment
* [drc_com_common] Add desktop icon to launch mini maxwell for drc network environment
* add hrpys service to pass setting
* add hrpys service to pass setting
* add hrpys service to pass setting
* add hrpys service to pass setting
* fixed installation in catkin.cmake
* fixed installation in catkin.cmake
* Revert "Revert "add drc teleop demo program""
* Revert "Revert "add drc teleop demo program""
* Revert "add drc teleop demo program"
* Revert "add drc teleop demo program"
* add drc teleop demo program
* add drc teleop demo program
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Yu Ohara, Eisoku kuroiwa, Iori Kumagai, Iori Yanokura
