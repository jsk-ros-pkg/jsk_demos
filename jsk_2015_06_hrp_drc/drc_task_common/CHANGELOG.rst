^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drc_task_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------
* [robot_model.launch] use upstream launch file
* [robot-util.l] add jaxon_jvrc in robot-file
* [drc_task_common/euslisp/irex_carry_trass] Set red color for trass interactive marker
* [drc_task_common/euslisp/irex_carry_trass] Test large step stride while trass demo walking.
* [drc_task_common/irex-large-box] Reduce initialization time by k-kimura.
* [drc_task_common/irex_carry_trass] Add viewer title and reduce unnecessary sleep by rkoyama.
* [drc_task_common/euslisp/irex_carry_trass] Add argument to skip recog&gopos.
* [drc_task_common/euslisp/irex-large-box] Add resizing of irt viewer
* [drc_task_common/euslisp/irex-large-box] Update for sift recog by k-kimura and youhei
* [drc_task_common/euslisp/irex-large-box] Fix via coords for reaching and add setting of time parameter for object turnaround detection for box pushing.(by k-kimura)
* [drc_task_common/euslisp/irex_carry_trass] Add y-o-n-p for recognition by rkoyama.
* [drc_task_common/euslisp/irex_carry_trass] Use y-or-n-p instead of read-line
* [drc_task_common/euslisp/irex_carry_trass] Fix gopos value and rename function
* [drc_task_common/euslisp/irex_carry_trass] debug the vibration during carrying the trass
* change push emergency stopper threshold parameter to 100N
* [drc_task_common] Add imprecise scheduler prototype
* [drc_task_common/euslisp/irex_carry_trass]add recognition, and debug the refforce estimation
* modify the timing of recognition
* add apply-primitive-dimensions-to-midi-device
* modify calib-offset-coords parameter
* [drc_task_common/euslisp/irex_carry_trass/carry-trass.l] modify demo to use pfilter
* add jaxon-wooden-box-okkake loop
* receiveing estimated pose from particle filter
* change new sift template KEEP DRY label
* Delete anybots-box programs
* Add maai diff value
* [drc_task_common/euslisp/irex_carry_trass] modify the pose to carry trass
* Add interactive marker
* Change irex demo version for pushing box
* Merge remote-tracking branch 'koyama/add_irex_koyama_release'
* add snozawa san commit
* [drc_task_common/euslisp/irex_carry_trass/carry-trass.l]modify the pose to reach-trass
* add box-push emergency stopper
* modify comment out
* change apc_box name to wooden_box name
* [irex_carry_trass] update trass
* [drc_task_common/euslisp] add files to carry trass.
* add y-or-n-p cancel mode
* modify head neck-p joint-angle
* change head neck-p angle and sift-label-coords
* Merge pull request `#1126 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1126>`_ from mmurooka/valve-motion-slower
  [drc_task_common] make slow valve motion a little
* CMakeLists.txt : clean up install rules
* CMakeLists.txt remove install FILES plugin_description.xml, which is not exists
* change impedance parameters and reach pos y
* change soft arm impedance
* make slow valve motion a little
* modify ik joint angle margin
* delete euslisp files
* add jaxon recognizing push box for IREX
* integrate jaxon sift recognition and push box motion
* modify wooden box dimension z and add push main function
* change to wood box version
* devide anybots box and wood box
* add jaxon_red reaching and push motion
* push program to reach
* delete sift_sample
* change templates name
* move templates files
* delete sample-label.jpg and sift.launch
* update launch and images directory
* change apc model
* Change to other box version
* Add hrp2jsknt version reach motion
* Add irex box sift recognition and reaching motion
* [drc_task_common/euslisp/primitive-marker-util.l]debug the problem of tf by ueda-san
* [drc_task_common] Add door handle detector with super conservative way (it takes 17 secs to detect door)
* Contributors: Eisoku Kuroiwa, Kei Okada, Kohei Kimura, Masaki Murooka, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi

0.0.2 (2015-11-26)
------------------
* reserve params for 2nd time
* fix jaxon ik param for valve motion. set waist joint weight zero.
* change wall motion for renewed limitation
* remove lookat
* remove unneeded lookat
* Ignore auto generated files
* [jsk_2015_06_hrp_drc/drc_task_common/euslisp]remove unnecessary files in door-open
* [door-open]add door-open files
* add okiagari tab for ocs
* remove typo
* stop-impedance when using spreader
* fix param for jvrc
* change rviz interactive marker coordinates
* remove stop-impedance
* finish parameter tuning and add emergency-mode
* disable handle estimate and enable move with selected-point
* write to csv file
* fix param for jvrc
* add jvrc barcode reading
* add option not use resize for local usage
* modify hold-spreader function for speed-up
* add cheating speedup mode for jvrc
* add cylinder finding node
* [jsk_2015_06_hrp_drc/drc_task_common/euslisp/test-drc-terrain-walk.l] Add hrp2jsknt block climbup demo
* change ik for use box
* change stride parameter
* modify throw away function for using rarm camera
* [drc_task_common] add always-ik-mode
* remove designation of hand when grasping
* modify opening gripper for speed-up
* add throw away and look QR code function
* modify move-head-rot and add this function to select-func
* fix typo in jvrc-grasp-spreader
* fix typo
* add multisense launch
* add remove-offset and move-head-rot function
* change insert position and make middle pose for reaching
* enable to use larm for use spreader and to select which to reach (grip or body)
* use pr2_navigation_self_filter
* change grip place and modify adjust function
* add comment for initial function
* add set-ref-force because of spreader weight
* modify real to t in select function
* modify some function in local or world moving and add impedance
* modify jvrc-grasp-spreader
* add initialize of zero vector
* modify *jaxon* to *jaxon_red*
* modify reading number and add use spreader function (move from demo/terasawa)
* [drc_task_common] add ik option for peep into cylinder
* [drc_task_common] one click cylinder detection
* add jvrc-grasp-spreader
* not use group for remapping
* add get-up motion for jvrc
* locomotion_without_silver_for_jvrc
* jvrc ocs and fc without silverhammer
* change base frame and others
* revert only rviz file
* remove program because not used
* [drc_task_common] Use footstep-controller-old.l for prompt compatiblity
* add remapped ui for drc tasks
* change offset value for b_con
* add rqt ui for dummy b_Control_device
* add use-leg option to param-door
* add option for not use ocs remap
* change to recycle code
* remove unneeded tabs
* cp drc -> jvrc
* changed valve motin fast
* reverse angle
* uniform default arm for any robot
* change angle for pre reach
* change drill recog
* [drc_task_common] Remove fisheye_sphere from fc_misc.launch
* [drc_task_common] Remove state viewer, it is drawn on rviz
* [drc_task_common] Rewrite locomotion.launch and ocs_locomotion.launch with
  standalone_complexed_nodelet
* [drc_task_common] Rewrite drill_recognition_for_put.launch with standalone_complexed_nodelet
* [drc_task_common] Rewrite drill_recognition_for_wall.launch with santalone_complexed_nodelet
* [drc_task_common] Rewrite drill_recognition_for_button.launch with standalone_complexed_nodelet
* [drc_task_common] Rewrite drill_recognition.launch with standalone_complexed_nodelet
* [drc_task_common] Rewrite door_unvisible_handle_recognition.launch with standalone_complexed_nodelet
* [drc_task_common] Fix cmake not to compile drc_teleop_interface.cpp
* [drc_task_common] Do not use fisheye nodelet manager in order not to
  use bond connection
* [drc_task_common] Use jsk_rviz_plugins::RobotCommandInterfaceAction instead of drc_teleop_interface
* [drc_task_common] Omit footstep planner client in fc-executive.l
* [drc_task_common] Resolve collision of dynamic_tf_publisher in ocs
* fix walking-pose. check whether the robot has :head-neck-p before calling
* [drc_task_common] Do not echo silverhammer in field_computer.sh
* [drc_task_common] Fix shell script syntax error in operator_station.sh
* add recognition in hand
* slower drill grasp params
* [drc_task_common] Rewrite stereo_preprocess.launch and
  laser_preorocess.launch with jsk_topic_tools/standalone_complexed_nodelet
* change valve recog to use jsk nodelet
* [drc_task_common] Use current instead of effort for JAXON
* [drc_task_common] Remove tilt_laser_listener from laser_preprocess
* [drc_task_common] Set longer ~max_queue_size to extract nearest cluster
  for valve detection
* [drc_task_common] Do not run multisense_remote for hrp2
* add use_reach-until-touch mode
* add keyshot for drill
* add keyshot timing more
* add use_reach-until-touch mode
* fix reset motion
* add comment
* enalbe to select reach-until-touch mode
* add function to judge grasped or not
* add to use reach_until_touch
* [drc_task_common] Update launch files for locomotion planning
* [drc_task_common] Do not use jaxon_red_ros_bridge
* add keyshot for other tasks
* change params to detect standing drill
* add_parentheses
* [drc_task_common] Remove dependency to ar_pose
* Contributors: Hiroto Mizohana, Kentaro Wada, Kohei Kimura, Masaki Murooka, Ryo KOYAMA, Ryohei Ueda, Yu Ohara, Yuta Kojio, Eisoku Kuroiwa, Ryo Terasawa

0.0.1 (2015-06-11)
------------------
* fix drive recognition checks
* [drc_task_common] Add toe-kick method script in vehicle task for emergency. This PR do not affect to original script
* rename service
* para tune for jaxon red
* change drill watch pose
* add check drive recognition
* do not close hand when keep grasp
* remove unworked func
* add nodelet
* rechange drill button coords
* change laser preprocess
* add respawn in drive recognition launch
* add run stop rtcd for hrp2
* merge origin/master
* fix door last
* setq offset arm for drill
* change codes for rarm
* revise drill wall motion
* change drill wall motion for hrp2
* offset movement in drill-grasping
* add run-stop scripts
* change param of silver hammer hz threshold
* change impedance params
* add exception
* update interpolation time shorter in valve and door motion
* [drc_task_common] Fix wrong robot_description in OCS
* [drc_task_common] Increase max_z of stair_marker
* [drc_task_common] add git check to check_sanity_fc
* change_keyshot_timing
* add roscore tab to ocs tmux
* [drc_task_common] Update robot_model in OCS
* add Master check and silver check to OCS
* lower the waist for JAXON at stairs
* first ik with rotation axis
* fix hrp2jsknt door coords
* modified check node names and topic names
* add menu to select door reach deg
* add blacklist check to sanity check
* add git check sanity fc
* delete unnecessary nodes
* add check_saniy_ocs
* merge origin/master
* implement emergency button
* add send-go-pos-command script
* change impedance parameter of hrp2 after valve
* add emergency pose function : supporting jaxon and jaxon_red
* [drc_task_common] Add debug output for lowspeed communication
* [drc_task_common] Fix filter boundingbox
* def go-pos in ocs-ri
* [drc_task_common] Use lowest recognition result for drill
* fix typo
* change resolition
* [drc_task_common] update Silver Hammer Check
* [drc_task_common] Publish execute_flag in vehicle-fc-executive
* revise motion with experiment result
* Add drc final stair
* add emergency motion sample
* Use version argument for stair instead of :test-field
* revice ui
* [drc_task_common] Fix resume caller bag in vehicle-fc-executive
* add emergency-pose button to rviz
* add buttton
* resize keyshot
* default is sagami door (it is final door)
* add final test function
* use sagami door as final door. edit test-drc-door-task.l
* changed stand coords for hrp2jsknts final door
* disable continuous check in door
* add comment line for difficult door param
* use sagami door as final door
* change default flag
* [drc_task_common] Fix typo to work catkin_download_test_file
* add door knob picture to manipulation memo
* add drc final door model and instruction
* change joy topic name
* slightly change motion spee
* [drc_task_common] Patch for old geneus in vehicle task executive
* [drc_task_common] check rosmaster close_wait num in check sanity
* revise joy
* add line for send keypoint screen shot
* [drc_task_common, drc_com_common] Add simple footstep exec
* can change drill arm
* [drc_task_common, drc_com_common]  Add more dynamic reconfigure parameters
* change speed
* add flag for drill throw
* [drc_task_common] More update about check sanity
* add new motion(throw drill)
* change layout for drill
* change stand coords
* send drill put motion
* fix door-through-pose
* [drc_task_common] Modify accel paramters for jaxon
* fix msg instantiation args type
* Add stair check for hrp2jsknts
* [drc_task_common] Comment out draw-objects function
* [drc_task_common] Use detach_step value in stepGage
* [drc_task_common] Publish drive/controller/step only when step-accel is successfully executed
* [drc_task_common] Update detach edit and color when set_detach_step called
* [drc_task_common] Do not update accel-origin when step command failed
* [drc_task_common] Return result in step pedal function
* [drc_task_common] Return command in step function
* [drc_task_common] Add detach messages to silverhammer
* [drc_task_common] Add detach_step button to vehicle_ui
* [drc_task_common] Implement detach to hrp2jsknts
* [drc_task_common] Implement set-detach-step callback
* [drc_task_common] Commit detatch-accel function
* [drc_task_common] LAUNCH_RVIZ option is no longer needed in vehicle_fc
* [drc_task_common] Set threshould lower in pedaling callback
* [drc_task_common] Change background color of obstacle length according to distance
* [drc_task_common] Change background color of changing controller mode service when service is executing
* [drc_task_common] Check multisense remote
* finish processing when finishing task1
* [drc_task_common] Fix small bags
* remove magic number of msg length in ocs-robot-interface.l
* [drc_com_common, drc_task_common] Add more basic info for jaxon
* add reset_enc and finish_stair button to rviz
* fix jaxon and jaxonred valve motion
* change drill wall speed! need test
* change rviz showing
* slightly change drill grasp coords
* slightly expand drill button range
* fix jaxon impedance
* [drc_task_common] Add xyz-filter for drill recognition
* [drc_task_common] Add throttle parameter to checkerboard detector in vehicle
* [drc_task_common] Add jsk_recognition_msgs and jsk_interactive_marker to vehicle executive
* [drc_task_common] Fix typo: load
* [drc_task_common] Fix typo
* [drc_task_common] Remove laser preprocess assmbler
* fix the drill and drill put recognition to use nodelet
* [drc_task_common] Add handle_pose offset -25 to :z, which is compensation of distance between steering-center and marker board
* [drc_task_common] Fix typo
* add comment when drill grasp failed with condition
* remove bug for dril;
* remove bug if drill rotate failed with force-sensor noise
* 10 times sensoring
* [drc_task_common] Fix link name of HRP2
* [drc_task_common] Update valve request timing
* [drc_task_common] Fix OCS model visualization
* use set-foot-steps-with-base-height
* change speed for drill ;otate
* [drc_task_common] Consider car handle angle only when handle-crank is used
* change params for motions
* dynamic change ref force
* add ref force for drill wall
* change to use :arms
* add codes to maintain first leg angle in solving ik
* [drc_task_common] Consider handle-angle in solving approach-handle ik
* add new rviz button
* change to use larm semi-fixed in drill task
* [drc_task_common] Run multisense_remote if needed
* [drc_task_common] Check silverhammer highspeed input topics
* fix parameter to load urdf model marker setting
* remove bags with experiment with Jaxon Red
* fix the position to include robot_description launch
* add quadratic function in table
* rechanged fast motion
* [drc_task_common] Add jaxon_red driving scripts which inherits from ones of jaxon
* [drc_task_common] Comment out unnecessary debug messages
* [drc_task_common] Fix state check bug in correct
* [drc_task_common] Magical progn to avoid SEGV
* set ik-optional-weight-vector for valve motion
* change input topic because it is not working
* change init pose, anglevector-sending time with real environment
* [drc_task_common, drc_com_common] Use pesimistic mode for footstep planning
* modify coorinates transformation of predicted path marker
* support urata robot in publishing tempareture in basic-info
* update jaxon ik-server parameter to reduce base link roll moving
* [drc_task_common, drc_com_common] Perception and planning on ocs side about terrain task
* [drc_task_common] Fix obstacle_length type: int->float
* [drc_task_common] Set default real flag as nil in vehicle_fc.launch to prevent unintended movement in real robot if eus was respown
* [drc_task_common] Change execute/real button from toggle to menu like servoOn/Off
* initial commit of ocs-robot-intercae.l, support :state :angle-vector :force-vector :start-st :start-auto-balancer :start-impedance :stop-impedance :start-grasp :stop-grasp functions
* update jaxon impedance param
* [drc_task_common] More update for terrain task
* remove solve ik and isolate current-pos for u4
* [drc_task_common] Remove steering_diff_angle_vector from vehicle_ui
* [drc_task_common] Change colors when set_min/max_step service was called
* [drc_task_common] Prevent move joints before initialize by correct-handle-pose
* [drc_task_common] Check communication program too
* [drc_task_common] Remove unused scripts
* [drc_task_common] change ros::rate of vehicle-ocs-executive: 10 -> 5
* [drc_task_common] Add sanity script for fc
* [drc_task_common] Fix for communication limitation
* [drc_task_common] Modify default min/max move-mm for hrp2jsknts
* [drc_task_common] Modify default max anklle-p angle from 6 to 15
* change timestampe from ros::time 0 to ros::time-now
* [drc_task_common] set :stop mode before grasp handle, approach handle and overwrite handle angle because they needs synchronize joy controller
* [drc_task_common] Call setControllerMode only when changing mode in vehicle_ui and call drive/operation/synchronize only in setControllerMode
* Update for hrp2jsknt terrain sample
* change angle for drill watch
* [drc_task_common] Generalize setControllerMode function
* fix typo start-grasp command
* [drc_task_common] Update for terrain task
* add icon for reset-force-sensor
* fix typo
* [drc_task_common] Stop operation when overwrite command is called
* [drc_task_common] Call drive/operation services from vehicle_ui, not eus controller
* add funcs for stop right
* [drc_tack_common]change default nums of rotation
* change angle of drill watch pose for jaxon
* [drc_com_common, drc_task_common] Support footstep_planner and footstep_controller
* [drc_com_common, drc_task_common] Support projection of footprint
* add new button for current^pos and ik
* [drc_task_common] Move lleg 10mm to :z of hrp2jsknts to reduce lleg load
* change car center base coords
* change path visualizer parameter
* [drc_task_common] Remove argument like USE_HRP2JSK, use ROBOT envirnoment variable
* change impedance params(need test)
* add button to reset force offset to rviz
* change ros::roseus timing
* add new pose for detect button-pushed
* [drc_task_common] Modify force sensor topic name from *sensor to off_*sensor
* change hrp2 grasp
* change params for recog, pose of hrp2
* [drc_task_common] Do not launch vehicle rviz, integrating into one rviz
* [drc_task_common] Support jaxonred in stair task
* [drc_task_common] Change egress_button color according to execution
* [drc_task_common] Forcely stop and sync controllers in go-to-egress
* add nakashima-stairs test program
* change impedance params for wall
* [drc_task_common] fix neck_y_angle visualization in vehicle_ui
* Update terrain stair sample and readme
* Add function to make testfield stair
* [drc_task_common] Force to disable orientation in stair task
* remove bug around drill button
* [drc_task_common] Add egress service to silverhammer
* teleop program support jaxonred
* add tf car_center publisher for ocs
* delte print debug
* [drc_task_common] Change button background color until service is executing
* change pre angles
* revise miss cords
* add forgoten change
* impl callback of grasp and impedance function
* [drc_task_common] Fix typo of current_steering
  Do not display checkerboard detector view
* [drc_task_common] Update topic name of rviz
* [drc_task_common] Remap topics which is sent to ocs in global launch namespace of vehicle_fc
* [drc_task_common] Add ui elements for stair task
* change marker origin to end-coords(JAXON)
* add spin-once when reflecting fullbody-ik result to robot marker
* add translation when inserting hand mesh marker
* [drc_task_common] Change topic name for ocs in rviz config file
* use end-coords tf for robot marker of stand position
* add translation of end-effector link
* [drc_task_common] Update recognition parameter for door handle detector
* moved end coords of hand marker
* [drc_task_common] Update drill wall recognition
* [drc_task_common] Disable display option of car_center_tf_publisher too.
* [drc_task_common] Set display parameter of handle_pose_detector to 0
* [drc_task_common] Add current_steering, crank/handle_pose and
  predicted_path_marker to FC2OCSLarge
* [drc_task_common] Advertise foggoten topic /ocs/drive/controller/real
* [drc_task_common] Get lock when toggle button is changed
* [drc_task_common] Enable latch option to controller topics
* refactor impedance settig function
* update valve recog tolerance parameter
* publish drill rotate motion on rviz(revices)
* [drc_task_common] Add msgs for set-real service to silverhammer
* [drc_task_common] Add SEND_REAL_ROBOT button to vehicle_ui
* [drc_task_common] Add set-real option and real topic to driving-controller
* new node for showing result
* [drc_task_common] Display force/moment norm instead of force of max dirction
* Added test-field stair model.
* [drc_task_common] Change force sensor display mode from max direction
  force to norm
* [drc_task_common] Make step_gage label larger
* [drc_task_common] Add neck-p/neck-y-angle visualization label to vehicle_ui
* [drc_task_common] Faster recognition of footstep
* enable t-marker moved by pub-point
* [drc_task_common] Modify impedance parameter of hrp2jsknts for handling
* enable to  move any marker
* rename topic name
* reduce result^showing time
* more fast drill motion
* change base_tf from car_center to BODY
* [drc_task_common] Add min/max limitation to :estimate-current-handle-angle
* [drc_task_common] Fix :estimate-current-handle-angle, consider grasp offset
* [drc_task_common] Fix grasp/turn-handle-once offset parameters for hrp2jsknts
* [drc_task_common] Reflect offset to turn-handle-once function and set default offset-wrt to :local of hand in :grasp/:turn-handle-once
* add test codes
* [drc_task_common] Add filter_bbox_position.py
* change jaxon drill params
* [drc_task_common] Implement execute button, which disable joy controller and connection between vehicle-fc/ocs-executive
* [drc_task_common] Fix tmux script not to generate '1' file
* [drc_com_common, drc_task_common] Add imu to basic info
* do not open hand first in jaxon door motion.
* change stand coords to avoid wall
* fix door recognition, plane recog
* add overwrite stand coords
* enable to select stand coords
* add initialization
* [drc_task_common] Add neck status to prevent moving neck before initialize
* [drc_task_common] Update camera topic for ocs
* enable not used coords
* update soft impedance parameter for jaxon
* [drc_task_common] Transmit off_ sensors to ocs
* [drc_task_common] Disable rviz for vehicle in fc
* [drc_task_common] Add neck_p/y_angle to silverhammer
* [drc_task_common] Change rate of executive and streamer
* revise params for button
* add cancel-motion icon
* update door-through-pose to avoid touching right hand to door
* remove unused button : debri, hose, look-around
* add hand pose button
* add push motion
* not show eus ik result on irt viewer
* fix hand marker dead lock by canceling menu
* revised reach-until-touch for local coordinates sys
* [drc_task_common] Fix forgotten argument
* [drc_task_common] Remove nodes for fc in vehicle_ocs
* [drc_task_common] Separate vehicle launch files into vehicle_fc/ocs and remap tf, joint_states, robot_description
* replace to use require instead of load in task motion eus program
* [drc_task_common] Add ocs namespace to model files
  [drc_task_common] Update rviz drc teleop button
* [drc_task_common] Remove force sensor throttle (throttled in vehicle_ui drawing) and remap vision topics in vehicle_ui for silverhammer
* add hand calib button to rviz gui
* change stand coords for grasp
* add wall interactive marker
* enable to apply potentio-vector to rviz robot model
* remove stop abc/st button and start impedance soft/hard button to rviz
* Update parameters for Testfield terrain and update readme
* change input cloud to resize_1_4
* [drc_task_common] Add sleep when launching nodes
* change remap in c++
* [drc_task_common] Move polaris model from hrpsys_gazebo_atlas
* minor update of manipulation memo
* add door-through-pose2 to go through door fast
* [drc_task_common] Add obstcle_length to silverhammer
* [drc_task_common] Add USE_VEHICLE_LAUNCH option to vehicle fc/ocs main launch
* [drc_task_common] Launch car_center_launch and drive_recogntion.sh in vehicle.launch
* [drc_task_common] Display obstacle_length/indicator to vehicle_ui
* [drc_task_common] Add patch to speed up roslaunch
* [drc_task_common] Add window of launch file for vehicle task to ocs/fc shell scirpt
* [drc_task_common] Add ROBOT argument to ocs/fc main for vehicle task
* add imp for support arm
* input angle is deg, so add deg2rad
* [drc_task_common] Fix vehicle.launch path
* branch fail when modify ns
* fix namespace in python script
* fix typo
* [drc_task_common] Set default arguments as default, not value
* [drc_task_common] Integrate launch for vehicle task to main operator_station/field_computer scripts
* [drc_task_common] Update goal_handle_angle just after grasp to prevent unintended movement
* change button pushed recog method
* [drc_task_common] Wait until sync service is finished, but wait 0.5sec in silverhummer because service immediately return in it
* change msg type from Float64 to Float32
* [drc_task_common] Call synchronize service in main function because service call in serivce callback causes deadlock in executive
* change parameter of static tf and passthrough height for obstacle removing
* revice codes around drill button
* Add brake/neck_y/neck_p topics to silverhummer for vehicle
* update vegas stairs parameters
* [drc_task_common] Fix synchronize methods for controller in silverhummer
* [drc_task_common] Separate node which should be launched in ocs or fc. It would probably be in separeted files in future
* [drc_task_common] Add rviz config file for vehicle temporarily, which should be merged into whole system
* [drc_task_common] Add ~sensor_frame to multi_plane_extraction of drill_recognition.launch
* change ref force and add lookat in drill motion
* update drill motion
* [drc_task_common] Add ~sensor_frame to multi_plane_extraction of drill_recognition.launch
* [drc_task_common] Fix grasp offset of hrp2jsknts
* [drc_task_common] Update HRP2 initial pose
* [drc_task_common] Calib blue crank
* [drc_task_common] Update parameter for terrain task
* comment out with revise codes
* change for usefullness
* remove multi-defined func
* [drc_task_common] Re-estimate handle angle when overwrite
* modify CMake
* [drc_task_common] Do not grasp when recognitoin for correct is not succeeded
* [drc_task_common] Modify state check process in handle and accel
* almost finish arrangement of drive recognition launch
* remove comment
* [drc_task_common] Move hrp2jsknts initial position -100 to y axis
* modify coords transformation
* delete unnecessary files
* [drc_task_common] Fix parameter for drill recognition
* [drc_task_common] Support jaxon in tmux-based launching
* [drc_task_common] Fix accel approach angle of hrp2jsknts
* [drc_com_common, drc_task_common] Support fisheye lookat
* change save_data scripts to call rossetlocal
* change drill default grasp coords
* [drc_task_common] Fix angle-vectors of hrp2jsknts legs/rarm in real vehicle
* change codes around drill marker control
* add remap
* [drc_task_common] Modify approach-fist offset for hrp2jsknts
* remove service bug
* [drc_task_common] Modify hrp2jsknts initial poes based on s-noda egress
* add option for joy
* [drc_task_common] Add initialize/synchronize service for operation to executive
* add joy for teleop
* add lasvegas valve test to test full function
* [drc_task_common] Use timerEvent to prevent stop force sensor values
* [drc_task_common] Remove initialize from main function because initial pose can send from ui
* [drc_task_common] Estimate handle angle only when handling
* [drc_task_common] Operate hand in initialize
* [drc_task_common] Remain forcely sync option but default disabled
* [drc_task_common] Add comment
* [drc_task_common] Remove unnecessary :sync-controller
* [drc_task_common] Modify neck joint to 0 in drive-init-pose in jaxon
* [drc_task_common] Synchronize command when state and mode changed to prevent unintended movement
* change condition for button pushed
* re-enable hand-reset pose for hrp2
* change pose to reduce load
* [drc_task_common] Add sanity script to check network
* remove bugs
* Add DRCTestfieldTerrain
* Update README for Terrain demos
* change coords around drill stand coords
* fix dot-rviz to modify the state image position
* [drc_task_common] call :release whether handle is :running or not
* [drc_task_common] Set all control-mode :stop when initialize finished
* [drc_task_common] Fix tiny bug and confirm unvisible handle detector works
* changed ocs number for lasvegas environment
* [drc_task_common] Modify riding parameters for jaxon again
* [drc_task_common] Add floor-offset and fist-offset option to initilaize function
* [drc_task_common] Fixing tmux based launching
* [drc_task_common] Fixing tmux based launching
* Update location of terrain blocks considering size of bounding box
* remove a bug
* Fix size of ground plane
* Add optional ground for test field terrain
* change drill watch pose for jaxon
* [drc_task_common] Fix outsided init pose of jaxon by s-noda and adjustment still goes on
* [drc_task_common] Set :look-at-handle nil as default in correct-handle-pose function
* arrange launch files
* Add test field drc terrain
* [drc_task_common, drc_com_common] Cleanup launch files and support
  tmux-based launching
* enable avs methods in drc to except cancel
* add drill-auto-gops
* fix typo
* add comments if some no mean command selected
* Add hrp2jsknts terrain function
* [drc_task_common] Move initiali position of jaxon 100mm outside
* [drc_task_common] Modify approach-handle: add rotation redundancy
* Merge pull request #730 from terasawa/obstacle-indicator
  add obstacle indicator to assist drivers
* Merge remote-tracking branch 'origin/master' into do-not-send-joint-angle-before-initialize-called
* change pre-set modes
* add & in command rviz
* [drc_task_common] Do not initialize in :init process of controller, only set real silently
* add obstacle indicator to assist drivers
* [drc_task_common] Implement controller-mode services to vehicle silverhummer
* [drc_task_common] add button checker uis
* Merge pull request #729 from mmurooka/fix-jaxon-drill-motion
  fix jaxon valve motion : reaching direction and stand coords
* [drc_task_common] Add steering_diff_angle to vehicle-silverhummer
* fix jaxon valve motion : reaching direction and stand coords
* change showing text on rviz
* revised grasp coords with real sensor data
* [drc_task_common] Add neck-mode functions
* revise drill stand coords(temporary)
* Merge pull request #726 from mmurooka/fix-valve-motion-20150518
  [drc_task_common] valve door motion modification 20150518
* [drc_task_common] Implement SetValue service to vehilce task silverhummer
* Merge branch 'drill20150517' of https://github.com/YuOhara/jsk_demos into drill20150517
* change drill watching pose
* remove bags
* fix the error in the case that search-rotatable-range is called before get-valve-motion is called
* change ui for ocs
* remove bugs
* [drc_task_common] Use empty-service-client/server and add additional empty-services
* [drc_task_common] Add client/server for empty-service
* [drc_task_common] Add look-at-handle option to correct-handle-pose
* [drc_task_common] Replace send *ri* :angle-vector to :model2real in controller
* [drc_task_common] Add model2real method to robot-driving-motion for controller
* [drc_task_common] Modify :real option of motion in each robot-driving-controller
* [drc_task_common] Get whole initialization process together and send angle-vector once
* [drc_task_common] Add :use-real-robot key to real option to choose whether sync with *ri* or not
* [drc_task_common] Fix typos
* add min of rotate num(1)
* [drc_task_common] Add look-at-handle option to correct-handle-pose
* [drc_task_common] Replace send *ri* :angle-vector to :model2real in controller
* [drc_task_common] Add model2real method to robot-driving-motion for controller
* add condition to use pre-pose
* [drc_task_common] Door handle detector for unvisible handle
* [drc_task_common] Modify :real option of motion in each robot-driving-controller
* [drc_task_common] Get whole initialization process together and send angle-vector once
* [drc_task_common] Add :use-real-robot key to real option to choose whether sync with *ri* or not
* [drc_task_common] Synchronize with joy after overwrite hanlde angle
* [drc_task_common] Add name fields to motor_states in ocs side
* [drc_task_common] Optimize nodelet in valve detection
* [drc_task_common] Add neck_mode visualization to vehicle_ui
* [drc_task_common] Add neck_mode and callbacks because neck callbacks seems to be collision with correct-hanlde-pose
* [drc_task_common] Synchronize joy_vehicle status when initialize and grasp
* [drc_task_common] Remove specification of interface file in locomotion_planner.launch
* [drc_task_common] Use :full-interruptible for footstep_controller
* Merge remote-tracking branch 'origin/master' into drill20150517
* Merge remote-tracking branch 'origin/master' into drill20150516
* add todo comemnt
* change pre angles
* change pose a bit
* option to change rotate num
* [drc_task_common] Fix default position of hrp2jsknt after real polaris adjustment in lasvegas
* add option for auto rotate drill
* Merge pull request #717 from mmurooka/fix-valve-impedance
  [drc_task_common] change jaxon impedance damping gain larger
* change jaxon impedance damping gain larger
* Merge pull request #716 from mmurooka/fix-for-forcibly-overwrite-stand-coords
  [drc_task_common] Fix for forcibly overwriting stand coords
* add modification for HRP2 launch files
* tune parameters
* fix bug in force overwrite standcoords for door and valve
* fix jaxon teleop launch network
* suppress shoulder-p and promote waist-y and
* add checkerboad detector for car_center
* chage grasp params for support arm
* [drc_task_common] I think it is beter that neck command is real joint angle
* add jaxon drill orotate test corde
* add sample motions
* add visualize steering angle launch
* add rostopic pub for rosbag
* [drc_task_common] Add neck-p callback to eus controller
* call set-default-impedance-param before starting impedance with rviz button
* fix impedance applying arm
* [drc_task_common] Add set_current_step_as_min button to vehicle_ui
* fix bug
* merge origin/master
* add stop num option
* add stop num option
* [drc_task_common] use euclidean clustering to compute bounding box to
  detect drill in hand
* add stop num option
* searching drill button motions
* [drc_task_common] Pedals should not be touched at first
* change imp timing
* [drc_task_common] Fix wait-sec typo
* [drc_task_common] Modify reach-until-touch param for jaxon
* [drc_task_common] Remove unnecessary compensation in reach-until-touch
* refactor drill souce code again
* overwrite stand-coords forcibly in first motion of valve and door
* [drc_task_common/vehicle_ui] Fix flicker of vehicle_ui by rounding stearing diff angle
* add srv
* add drill button state recog launch
* [drc_task_common]add srvs
* add fft node
* [drc_task_common] Move hrp2jsknts sitting position -50mm in y axis to center
* [drc_task_common] Fix correct-handle-pose bag
* [drc_task_common] Modify initial value of min/max_step of hrp2jsk
* [drc_task_common] Modify min/max edit value in min/max_step of vehicle_ui
* refactor set-drill-environment
* fix drill code minor bag
* [drc_task_common] Add resume-handle-pose-button to vehicle_ui
* [drc_task_common] Display message in initialize
* [drc_task_common] Add set_current_step_as_max button to vehicle_ui
* [drc_task_common] Only view max_force and direction in force_sensor
* [drc_task_common] Fix hrp2jsk impendace, M = 0
* remove slight bug around drill rotation
* delete trailing while space
* add dynamic reconfigure
* [drc_task_common] Use laser pointcloud for detecting wall to cut with drill
* [drc_task_common] Fix indent
* [drc_task_common] Resume approach-pedal, which was eleted wrongly
* [drc_task_common] Disable M in impedance to prevent unintended move according to foot movement
* [drc_task_common] Fix forgetting allow-other-keys in calc-error-of-grasp-arm
* [drc_task_common] Add display-result option to calc-error-of-grasp-arm
* add demo program for las-vegas-indoor-stairs
* [drc_task_common] Add visualization of angle-vector-difference to vehicle_ui
* [drc_task_common] calculate angle-vector difference in main loop
* [drc_task_common] Add calc-error-of-grasp-arm method to test angle-vector difference in steering
* Merge pull request #697 from mmurooka/move-stand-coords-func-util
  [drc_task_common] move check-stand-coords function to robot-util.l
* update jaxon stair parameters
* move check-stand-coords function to robot-util.l and use them in each task
* replace tab with space in drill program
* add door side wall and check collision in test function
* add option to test collision in eus motion
* add door posture memo
* Merge branch 'drill20150515' of github.com:YuOhara/jsk_demos into drill20150515
* change drill button stop num
* [drc_task_common] Fix drive-init-pose for HRP2JSKNTS in real polaris in lasvegas
* add chest offset parameters to waking-pose
* Merge pull request #683 from garaemon/machine-tag-to-run-code-only-localhost
  [drc_task_common] Support USE_LOCALHSOT argument to run code on localhost
* delete comment-out
* publish car_center from posestamped marker
* rename door memo to manipulation task memo. add valve memmo
* [drc_task_common] Set color to large force in vehicle_ui
* [drc_task_common] remove_bug, change params with visual feedback
* [drc_task_common] Add force sensor values of arm to vehicle_ui
* [drc_task_common] Add approach interface to vehicle_ui
* delete ik-optional-weight-vector in set-default-impedance-param
* close hand in jaxon door motion
* use narrow-width-pose for jaxon door through
* modified final pose of jaxon door motion to avoid collision with door
* send first posture of door motion slowly
* changed impedance parameter of jaxon door
* changed stand coords of jaxon door
* [drc_task_common] Update force sensor value less frequently in vehicle_ui
* change jaxon drill wall stand coords
* [drc_task_common] Add LAUNCH_HANDLE_DETECTOR option to vehicle.launch
* [drc_task_common] Add threading lock to drawing functions in VehicleUIWdiget
* [drc_task_common] Implement overwrite handle method and add interface for that to vehicle_ui
* Merge remote-tracking branch 'origin/master' into drill20150515
* revise drill wall motion
* [drc_task_common] Support USE_LOCALHSOT argument to run code on localhost,
  especially about laser preprocess
* [drc_task_common] Use laser pointcloud to detect valve
* [drc_task_common] correct-handle-pose do not have tm in argument
* [drc_task_common] Resume head after correct
* [drc_task_common] Add max-dist argument to some functions which includes reach-until-touch
* [drc_task_common] Speed up some actions in vehicle task
* [drc_task_common] Modify impedance parameter for steering and speed up
* make door-motion fast : use angle-vector sequence and change time from 3000 -> 2000
* make valve-motion fast : time 1500 -> 1000
* fix trans-list of door push motion
* restore lasvegas door
* move arm upper in releasing motion
* add door-though-pose button
  fix trans-list of door push motion
* do not close hand in door-grasp shape
* set door-through-pose after opening door
* add function to set default impedance param and call that before each task setting
* [drc_task_common] Modify parameter range in setText for min/max_step
* [drc_task_common] Separate correct/resume/regrasp process
* change marker height to zero
* add door parameter memo
* fix handle l/r of mirror door
* [drc_task_common] release more distance in :execute-handle-pose-compensation of jaxon
* [drc_task_common] Add release-offset and shoulder-y-angle option to :execute-handle-pose-compensation
* [drc_task_common] Remove accel-origin in initialize
* [drc_task_common] Remove accel-origin in initialize
* [drc_task_common] Update accel-origin in approach-accel
* fix bool of step on flag
* update drill motion slightly
* [drc_task_common] Update accel-origin in jaxon
* [drc_task_common] Modify appraoch-accel pose in jaxon
* Merge branch 'integrate-drill-grasp-recog' into drill20150515
* [drc_task_common] Integrate drill recognition
* [drc_task_common] Modify jaxon init pose for less crotch-roll movement
* [drc_task_common] Add steering position evaluation script
* [drc_task_common] Add collsion avoidance and reach-until-touch to approach-frame
* [drc_task_common] Modify drive-init-pose for jaxon in normal polaris
* [drc_task_common]Do not downloada models on travis
* change params for junte motion
* [drc_task_common] Update drill recognition
* remove bag, change grasp pre pose
* [drc_task_common] Add main silverhummer launch file for vehicle task
* [drc_task_common] Add callback functions for empty service to executives
* tune for junte motion
* [drc_task_common] Optimistic recognition mode for drill recognition
* [drc_task_common] Remove unused topcis
* [drc_com_common] Implement parser for topics in driving-controller
* [drc_task_common] Apply OCS_NS to vehicle_ui in vehicle.launch
* [drc_task_common] Preserve min/max_step and only update min/max_step textbox when min/max_step value is updated
* [drc_task_common] Avoid zero division in vehicle_ui
* [drc_task_common] Do not use global namespace in vehicle_ui
* add lasvegas outdoor model and sample motion function
* tuned params for drc-drill
* arrange drive recognition script for dividing fc function
* [drc_com_common, drc_task_common] Support forces and temperature in OCS side
* add hrp2jsknts launch files
* [drc_task_common] Add prototpype scripts for eus executive in silverhummer, which only pass handle_cmd and accel_cmd
* add tf car_center launch
* merge origin/master
* remove bugs around jaxonmotion
* merge origin/master
* [drc_task_common] Update recognition parameters for las vegas door
* modify pull distance in jaxon valve motion
* [drc_task_common] Use throttle to force sensor values to avoid SEGV in vehicle_ui
* remove lasvegas door temporary because drc_com_common msg problem
* [drc_task_common] Add grasp-point to car frame in polaris model
* add horizontal-rotate motions in drill motion
* [drc_task_common] Visualize current handle/accel state in vehicle_ui
* add drill-primitive-set-coords funcst
* change grasp coords, remove codes
* [drc_task_common]enable to change arm with drill task specific
* add print to usage of gen_hosts.py
* [drc_task_common] Respown vehicle_ui in vehicle.launch
* change not to use support-drill-arm
* change door color to become visible in while background window
* enable to force overwrite door arm side
* [drc_task_common] Add mode toggle interface to vehicle ui
* fix typo
* change default rqt_ui
* readd drill layyout
* merge origin/master
* replace tab with space
* change model dir
* fix parenthesis in ocs-exective.l
* Merge pull request #642 from mmurooka/add-recog-mode-button
  [drc_task_common] Add recog auto/semi-auto mode buttons
* Merge pull request #643 from mmurooka/stand-coords-overwrite-option
  [drc_task_common] enable to select force / auto / on overwrite for stand-coords
* [drc_task_common] Update flags to controller state and add controller mode for operation/recognition switch
* fix bug in adding lasvegas door
* [drc_task_common] Do not use impedance in support legs for jaxon in vehicle task
* [drc_task_common] Fix initial pose for jaxon in real polaris xp900
* [drc_task_common] Fix open/close-hand method for jaxon
* [drc_task_common]remove bugs around drill rotate motion
* [drc_task_common] Add reach-until-touch-thre to set threshould for reach-until-touch
* enable to select force / auto / on overwrite for stand-coords
* add missing config file
* add button and icon for recognition radio buttons
* [drc_task_common]add some extra funcs
* add lasvegas door model and sample
* Merge pull request #637 from mmurooka/modify-col-pair-temporary
  [drc_task_common] ignore head and chest collision pair in door task
* [drc_task_common] Remove unused slot
* [drc_task_common] Add go-to-egress button to vehicle_ui
* [drc_task_common] Add egress callback to controller. All flags are disabled in go-to-egress.
* [drc_task_common] Add function to go to egress pose to motion and impelement for jaxon
* [drc_task_common] Separate obsoluted drive-init-pose
* ignore head and chest collision because model miss
* [drc_task_common] Preserve old initial-pose as egress-pose
* [drc_task_common] Modify rotation-axis from t to :x in approach-fist for jaxon
* [drc_task_common] Change drive-init-pose process for jaxon
* Merge pull request #629 from terasawa/add-fisheye-image-view
  add fishey image_view
* Merge pull request #634 from orikuma/fix-grasp-points-of-support-methods
  Fix grasp points of support methods
* fix hand shape for door special pose of jaxon
* Merge pull request #631 from orikuma/add-reach-button-to-vehicle-ui
  Add reach button to vehicle ui
* add lasvegas environment sample
* [drc_task_common] Use default offset of approach methods in controller
* [drc_task_common] Use seat-left grasp point and fix transformation for offset, not using locate but using translate
* [drc_task_common] Add seat-left grasp point to support body
* ignore collision between chest_link2 and head_link1 temporary
* add optional drill funcs
* [drc_task_common] Do not use reach-until-touch in kinematics simulation mode
* [drc_task_common] Add reach buttom to vehicle_ui and service call for reach method to controller
* [drc_task_common] Increase stop iteration in turn-handle-once because sometimes ik failed in stop 50
* add fishey image_view
* [drc_task_common] Fix approach-fist offset
* add drill rotate button
* add drill rotate button
* change miss rosparam
* changed motion when drill-recog-skip selected
* changed ik nums
* fix jaxon description launch
* add drill_rotate_motion
* add jaxon watch-drill pose
* add marker name
* fix miss cfg params
* changed launch to use nodelet
* more stoic hand-box
* Merge pull request #621 from YuOhara/add_drill_recog_for_grasp
  0Add drill recog for grasp
* Merge pull request #620 from YuOhara/add_jaxon_and_hrp2jsknts_motions
  Add jaxon and hrp2jsknts motions
* [drc_task_common] Add step-on-flag for recognition
* add drill grasp recognition launch
* add /drive/recognition in topic name
* insert set-focus-marker-func
* Merge remote-tracking branch 'ohara_remote/add_primitives_util' into add_jaxon_and_hrp2jsknts_motions
* add set-primitive marker func
* change marker funcs to manipulate 2 markers
* add new cb for drill motion connect
* Merge remote-tracking branch 'origin/master' into add_jaxon_and_hrp2jsknts_motions
* add jaxon and hrp2jsknts motions
* add drill recognition for drill grasp
* omit unnecessary function and remove comment
* Merge pull request #614 from mmurooka/fix-hand-mesh-marker
  [drc_task_common] fix hand mesh marker for other robot
* [drc_task_common] Tune impedance parameter for legs
* delete unnecessary file
* rename input to passthrough/output
* apply drill_button_recognition in drc_system
* merge origin/master
* [drc_task_common]change launch to use new method
* [drc_task_common] add option to not calc cylynder (for drill in hand)
* add options to use buttom of b_box
* fix hand mesh marker bug
* change file name and remove function of mochikae
* [drc_task_common] add drill detection option
* Merge pull request #609 from YuOhara/fix_typo_change_params
  [drc_task_common] fix_typo, change params
* Merge pull request #608 from YuOhara/drill_interpolate_angle_vector
  Drill interpolate angle vector
* [drc_task_common/drill_detect]changed to use cylinder
* changed stand coords for door motion. use setq instead of defvar for other robot redefinition
* [drc_task_common] fix_typo, change params
* Merge pull request #606 from mmurooka/support-reach-until-touch
  [drc_task_common] support reach-until-touch in teleop system
* support reach-until-touch in teleop system
* Use grasp-pose instead of close-pose for hrp3hand
* [drc_task_common] Add approach-fist method, support robot body making rarm land on seat
* [drc_task_common] Fix impedance parameter for leg softly
* [drc_task_common] Fix set-ref-force key name again * 2
* [drc_task_common] Fix set-ref-force key name again
* [drc_task_common] add check-grasp-coords coords
* [drc_task_common] remove unneeded line
* Merge remote-tracking branch 'origin/master' into drill_interpolate_angle_vector
* [drc_task_common/drill-wall] add interpolate angle-vector in wall-motion
* [drc_task_common/drill]change angle for watch drill
* [drc_task_common] Add egress-pose temporarily
* [drc_task_common] Modify initial pose of hrp2 for rarm support
* [drc_task_common] Use jsk_pcl/NormalEstimationOMP in locmotion.launch to
  solve timestamp problem
* [drc_task_common] Modify impedance parameters for support
* [drc_task_common] Add rear-support-frame-attachment and seat grasp point
* [drc_task_common] Fix key argument name: start-ref-force -> set-ref-force
* [drc_task_common] Fix open-hand limb in approach-frame and add args option to approach-frame/grasp-frame
* [drc_task_common] Flip normal direction of laser pointcloud to head frame
* [drc_task_common] add launch to detect drill in hand
* [drc_task_common]add cfg initialization
* [drc_task_common] Do not overwrite step-brake, but brake-cmd and send :accel-cmd 0.0 in it
* [drc_task_common] Fix accel methods for relative command
* [drc_task_common] Change accel command from absolute to relative from accel-origin
* [drc_task_common] Add :coords-system and :debug arguments to reach-until-touch and compensate overshoot after reach-until-touch
* [drc_task_common] Pass args from controller to motion in approach accel
* [drc_task_common] Fix approach-accel position using reach-until-touch for hrp2jsknt
* [drc_task_common] Make impedance harder in support
* [drc_task_common] Make slower reach-until-touch and use impedance first in approach-floor
* [drc_task_common] Do not use limb-controller in kinematics simulation
* Update hrp2jsk terrain walk
* [drc_task_common] Use laser pointcloud to detect door handle
* add door name select button to optional buttons
* [drc_task_common] Reflect rename of joy_vehicle.launch
* add option to use model z pos. change hrp2 imp param. fix posture to use arm avoid pose
* add button and functions to select door push/pull direction
* enable to select whether to overwrite stand-coords or not when reflesh motion
* Merge pull request #584 from mmurooka/door-ocs-fc-function
  [drc_task_common] update fc and ocs functions for door
* Merge remote-tracking branch 'refs/remotes/origin/master' into jaxon-footstep-planner
* [drc_task_common] Support parmaeters for jaxon by USE_JAXON argument
* Merge remote-tracking branch 'origin/master' into add_drill_wall_marker
* add simple marker forr drill wall
* fix door reaching motion and grasp timing
* test valve motion with hrp2jsknts and staro
* update fc and ocs functions for door
* apply hand marker ui to robot node
* visualize hand marker
* [drc_task_common] Support ~verbose parameter to supress info messages
* fix overdone if=false
* [drc_task_common] Use dynamic_reconfigure parameters for StandingDrillDetector
* Fix handle controller namespace settings
* Pass OCS_NS and CONTROLLER_DEV to ps3joy launch
* [drc_task_common] Add standing drill detector
* add conditions for add ref force
* [drc_task_common]rename topic name(sed -i -e 's#/multisense/resize_1_1/points#/multisense/organized_image_points2_color#g' *)
* add test door function without robot-interface
* Merge pull request #568 from YuOhara/add_drill_arm_change_option
  Add drill arm change option
* add test function which use robot-interface
* add valve test program which do not use robot-interface
* [drc_task_common] Fix reach-until-touch direction, reflect result of reach-until-touch to model and add tools for approach-floor to use imu.
* change to use mid-point in drill wall coords
* Do not use index finger in handling
* enable to change stand coords manualy
* Move hrp2 100mm to y direction and fix accel/floor leg position using crotch-y
* Modify detouch-accel-pedal distance from 100 to 50
* enable to switch arm with drill motion
* Use reach-until-touch in approach-accel/brake and return ik result in these functions
* merge origin/master
* add options for change drill-arm
* Return approach-result in approach-pedal function
* Override approach-pedal, not approach-accel/brake in each robot
* Fix look-around method and add look-around interface to contorller
* [drc_task_common]fix typo in drill grasp motion
* add missing ui file
* [drc_task_common]move launch files(related to drill)
* Merge pull request #552 from garaemon/separate-launch-for-each-robot
  [drc_task_common] Separate launch files to load URDF on OCS side according to ROBOT environmental variable
* enable to select valve grasp mode (edge or center) from ocs ui
* change drill picture
* Add launch for ps3joy to vehicle.launch
* add drill rotate motion
* [drc_task_common] Separate launch files to load URDF on OCS side
  according to ROBOT environmental variable
* modify drill_sift.launch
* set relative pose to 0
* drill_sift.launch
* [drc_task_common] Ignore tf timestamp when removing ground pointcloud in ocs
* [drc_task_common] Remove outlier of laser pointcloud by
  RadiusOutlierRemoval for locomotion planning
* [drc_task_common] Use dowmsapmpled pointcloud in v
* Fix handle_operation_interface path and add LAUNCH_EUS option
* Modify vehicle.launch to launch whole node for vehicle task
* Add main functions for each robot
* Rename vehicle-main to robot-vehicle-main
* Move handle_pose detection nodes from vehicle.launch to separeted launch file (handle_pose_detector.launch)
* Add main funciton for vehicle task in euslisp
* Remove unnecessary count
* [drc_task_common] Visualize non-ground points on ocs rviz
* [drc_task_common] Add ground visualization in ocs side
* add optional button panel to ocs UI
* [drc_task_common] Fix locomotion namespace
* change impedence params
* change mirror-angle method
* drill motion with left hand
* add handle and stand point for hrp2 valve motion with center grasp
* add skip-recog iocn
* fasten playing motion on rviz
* Change min radius of valve recognition
  set min_radius of valve recognition 0.05
* [drc_task_common]add escape point in drill button if one ik failed
* Merge remote-tracking branch 'origin/master' into use-projection-to-look-at
* [drc_task_common] Use jsk_perception/project_image_point to compute point to look
  at
* Merge remote-tracking branch 'origin/master' into remove_bags_around_rviz_plugins
* Merge pull request #523 from garaemon/add-passthrough-for-drill
  [drc_task_common] Add jsk_topic_tools/Passthgough to drill detection to
* send left/right arm information from ocs to fc and apply it to real robot motion.
* remove bugs around rviz plugins
* add mirror angle-vector function
* support left/right arm manipulation for valve motion
* remove bags around rviz plugins
* Merge remote-tracking branch 'origin/master' into run-laser-preprocess-in-v
* add left right arm button to ocs ui
* [drc_task_common] Run laser-preprocess processes in vmachine
* Merge pull request #517 from YuOhara/drill_pose_ui
  [drc_task_common, drc_com_common] add drill poses ui, change codes style...
* [drc_task_common] Add jsk_topic_tools/Passthgough to drill detection to
  reduce CPU load and remove voxel grid downsampling in stereo_preprocess.launch
  to supress warning message
* Use turn-velocity in handle-callback of controller
* Add turn-handle-velocity which turn hanlde in target omega with angle-vector method
* Remove max-angle/max-angle-diff limitation in turn
* add drc task icon for ocs ui
* add comments for genarating-drill-motion
* Add argument key to publish steering-trajectory
* change README for new drc_program
* [drc_task_common, drc_com_common] add drill poses ui, change codes style a bit simpler
* [drc_task_common] Add machine tags
* Fix body->robot transformation bag
* Add yes argument to correct
* Call pre-sitting pose only once
* Fix jaxon pose with driving-simulator-envionment
* Stop balancer rtcs before initialize
* change incremental motions for drill button
* fix typo in generate-hose-motion.l
* fix robot_description for JAXON OCS
* remove not needed back-srash
* remove bags
* Publish /drive/contoller/step in accel-cmd
* Use default accel_cmd in hrp2jsknt
* open/close hand should be in moition, not controller
* added finger button motion as one option
* Return when torus-finder failed to estimate in execute-steering-by-torus-finder
* test code for drill button with hrp3-hand-finger
* fix  door motion for real robot
* Separate output topic to torus_finder
* Publish current steering-coords in initialize and when updated
* Fix memory leak in torus-finder-callback
* Use record-handling-end-coords and publish-steering-trajectory in trus_finder
* Add steering-trajectory visualization tools
* Add publish-body-relative-steering-coords to visualize steering-coords
* add sample motion of jaxon door
* Call support-by-leg in initialize without ref-force
* Modify arguments of approach-floor for send*
* drill button with more wide finger
* Merge branch 'jaxon_junte_drill_button' of https://github.com/YuOhara/jsk_demos into jaxon_junte_drill_button
* add feedback of real hrp2 experiment
* add feedback of real experiment
* Add error message for tf
* Revert compensated coords when approach/grasp faield
* jaxon impedence
* Fix error handlig of execute funtion for handle_pose recogniotion
* Run handle_pose detection from vehicle_ui with correct button
* Run handle_pose recognition callback only when handle-pose-estimation-flag is t
* Merge pull request #499 from orikuma/fix-pedal-command-name
  Fix function which is used to convert pedal command to pedal motion
* Return result of approach-handle in motion result in controller
* modify door functions for general robot use
* add forgotten modification for generate hose motion
* Call sync-contorller when initialize called from vehicle_ui
* Set default accel-flag nil and modify to t when approach
* Resume original pose when second ik in approach-handle failed
* memo for jaxon-button motion
* Pass options to approach/grasp/release-handle in controller methods and modify release-handle default rotation-axis to :z
* revise jaxon button push coords
* Do not use sync-controller in release-handle in simulaiton-mode
* Use look-at-target to search handle_pose
* Fix function which is used to convert pedal command to pedal motion
* Merge pull request #496 from mmurooka/modify-stand-point-manually
  [drc_task_common] change robot stand point manually in teleop motion
* Update readme for terrain walk
* Update terrain samples to reduce duplicate functions and add real robot test codes
* remove hrp2 inverval pose
* avoid error when robot_marker_root is not published
* Add release-recognize-regrasp motion prototype
* Enable sync-controller in release-handle
* enable to change robot stand point manually in teleop motion
* HRP2JSK do not have openhrp3hand
* Separate torus_filter and handle_pose result in member valiable
* Release accel when accel-flag disabled
* Return ik result in grasp/release functions
* Add release-handle method to motion and modify default rotation-axis from t to :z in grasp-handle
* Call subscribe after publish because subscriber calls publisher in itself
* Merge pull request #493 from YuOhara/add_ref_force
  Add ref force
* Merge pull request #474 from garaemon/robot-head-ui
  [drc_task_common] Add RobotHeadUI to specify joint angles of head directly
* [drc_task_common] Add RobotHeadUI to specify joint angles of head
* Merge pull request #492 from mmurooka/add-jaxon-takenoko
  [drc_task_common] add jaxon takenoko motion sample
* Merge pull request #494 from mmurooka/fix-continuous-motion-in-valve-motion
  [drc_task_common] fix valve motion to generate continuous motion
* fix valve motion to generate continuous motion
* Merge remote-tracking branch 'origin/master' into add_ref_force
* add-ref-force
* Fix brake_cmd behavior like new handle_cmd
* Fix published step value in hrp2jsknt: relative move-mm -> absolute move-mm
* Fix accel_cmd behavior like new handle_cmd and publish /drive/controller/pedal_state for recognition
* Publish all operation command from handle controller and trim handle_cmd in driving-controller to reflect newest command
* add jaxon takenoko motion sample
* fix bug : add setq in generate-valve-motion.l
* Add comment
* Return remain-angle in turn like turn-handle
* change weight for drill grasp ik
* add jaxon 1m lateral walk parameters
* check continuousness of joint angle in rotating valve
* Add comment
* Integrate checkerboard handle_pose detector and driving-controller
* Use hoffarbib interpolation instead of linear
* modify valve motion with real jaxon experiment
* Modify topic name in vehicle_ui for controller namespace
* Return real command in accel-cmd for hrp2jsknt-driving-controller
* Add /drive/controller/step, min_step, max_step for vehicle_ui and enable latch
* Merge remote-tracking branch 'ohara_remote/add_collision_check' into remove_bags_in_wall_motion
* [drc_task_common] drill add missed robot-pose
* Modify default turn-handle method: once->sequence
* Skip target-angle when interpolating by angle-vector-sequence in turn-handle
* add r(l)arm in c-check list
* add collision check for drill wall
* Remove unnecessary sleep in initialize
* Modify rate of driveing_force_gt 1 -> 100
* Add turn-hanlde-once function, which call angle-vector once for target angle instead of angle-vector-sequence without thinking of steering path
* Add wait-interpolation after send angle-vector-sequence because angle-vector flashback occurs when angle-vector-sequence is overwritten
* Slow down first angle-vector in turn-handle sequence to prevent oscillation at first time
* add jaxon valve sample. enable to grasp valve center. rotate ccw direction.
* forget to use deg2rad
* Remove copy-object in robot-driving-motion.l
* Remove :update-handle-angle-coords-table method which is no more needed
* Remove debug print
* Fix steering-center-at-zero-deg coordinates in handle
* Fix memory leak bag in estimate-current-handle-angle
* Estimate -current handle-angle based on coordinates, not coords table
* Disable diff-max supreession
* Add function to display debug message
* fix root joint min parameter for jaxon
* Use floor-footrest instead of floor for hrp2
* Add target-handle key to apporach-floor
* Add floor-footrest handle for hrp2 footrest
* change drill grasp coords
* Override turn-handle for jaxon because stop is bigger than default
* [drc_task_common]jaxon standcoords for drill wall
* Override approach-accel/brake-pedal method for jaxon
* merge branch
* jaxon drill wall motions
* Avoid collision with handle and arms because steering-center is in handle-link
* Fix only x and y axis by rotation-axis in approach-floor
* Add stop argument to turn-handle
* Do not move arms in drive-init-pose-crank
* [drc_task_common]add comment
* Rename steering-ik-seed to steering-arm-ik-seed and add use-ik-seed option to turn-hanlde
* Add update-ik-seed function to use same ik-seed in turn-handle
* [drc_task_common] remove bags around drill put, fix drill-grasp-move-target
* [drc_Task_common]change drill motion params[grasp, put, button]
* [drc_task_common] Use circle dot patterns instead of ar marker as handle marker
* [drc_task_common]change drill grasp coords for jaxon
* [drc_task_common]remove bags, add attachment
* [drc_task_common]add jaxon drill motion
* Fix step-accel-command method name in jaxon-driving-controller
* [drc_task_common] Add vehicle.launch and detect handle pose by ar marker
* [drc_task_common] Add script to convert ar_pose/ARMarker to geometry_msgs/PoseStamped
* Fix min/maxEditCallback: update_value should be called to set values and setText should set returned value from controller
* Fix min/max Down/Up button callback: setText should make string from next_value.set_value, not next_value
* Modify service/topic names for driving controller naming conventions
* Add service callback and fix topic names for vehicle_ui
* Implement initialize/grasp/release callback. collect needs to some changes.
* Add initialize funtion for driving controller
* Add detatch-accel-pedal method for emergency avoidance to accel
* Modify arguments for new controller and motion methods
* Separate interface functions to controller, remove unused methods and add support-leg methods
* Add floor grasp-point to vehicle and simulator
* [drc_task_common] add air-graspup for drill
* Fix typo: ImageWidget->ROSImageWidget for multisense_widget
* Add document for jaxon stair climb simulation
* Add jaxon stair kinematics simulation
* Set color for models
* Set color for models
* [drc_task_common]change_orig_of_interactive_marker
* [drc_task_common]add_grasp_pose
* Merge remote-tracking branch 'ohara_remote/change_params_for_drill_button' into change_rotation_axis_for_Drill_grasp
* change rotation axis for drill grasp
* add dependency to python-urlgrabber in README
* do not load hrp2 model as default
* [drc_task_common] change params for drill button
* add fullbody options
* Merge pull request #454 from YuOhara/add_joy_move-end
  Add joy move end
* add joy funcs
* [drc_task_common] Add vehicle UI
* [drc_task_common] update_params for drill grasp
* [drc_task_common]change grasp reaching params
* add function for the motion to add force
* [drc_task_common]drill push botton many times
* Merge pull request #453 from orikuma/jaxon-driving-pose-examination
  Add jaxon driving poses for egress
* Disable brake pedal
* Add pre-left-sitting-pose to jaxon motion
* Add left-sitting pose which is mid pose of sitting and egressing and add prepare-egress for noda-egress
* fix knob position and motion for new door sagamihara knob handle position
* fix loading robot_descrioption in operator_station_main.launch
* Merge pull request #426 from mmurooka/enalbe-head-overwrite
  [drc_task_common] add functions to enable/disalbe head joint ovewrite
* Add drive-init-pose-touch-fist-to-seat pose
* add robot environment instruction
* add options for drill manip without reverse hands
* Update for sagami terrain block
* Update hrp2 model path in README
* change palams for drill buton
* Merge remote-tracking branch 'mmurooka/enalbe-head-overwrite' into murooka-20150411
* fix punch motion and reach motion for sagami door
* modify jaxon valve parameter such as end-effector transformation and ik parametr
* Fix joint name: :elbow-y -> :shoulder-y
* [drc_task_common] Set default parameters for torus_finder in steering_estimation
* add rqt qui button and ocs/fc functions to enable/disalbe head joint overwrite
* Add drive-init-pose and ride position for right-sitting/front-sitting position of jaxon
* Add rot-offset to rotate approach coords around original grasp-point to sit on the right of car
* Add warn message when turn-handle deg is limited by handle min/max
* Publish estimated/target handle angle
* add move-end with joy
* Modify base handle-angle of handle angle estimation from model-angle to old-estimated-angle and move estimation functions to controller
* Update handle-angle estimation and add function to overwrite handle angle when overturn occures
* delte old program for visualizing predicted car path
* add new program for visualizing predicted car path
* fix launch file for jaxon
* Update handle-angle estimation and add function to overwrite handle angle when overturn occures
* Merge pull request #418 from furushchev/use-method-instead-slot
  [drc_task_common] use :active-state method instead of slot 'active-state'
* fix color of string in rviz_status
* reduce robot dependent source from euslisp and launch files
* [drc_task_common] Update steering estimation
* change tf name : hrp2_marker_root -> robot_marker_root
* Merge pull request #437 from mmurooka/jaxon-valve
  [drc_task_common] support jaxon in valve-motion
* [drc_task_common] Modify transformation of base for steering estimation: steering relative -> body relative
* [drc_task_common] Call set-impedance-for-support when approach to ground
* add test code for 4 motion
* Fix approach-handle offset parameter for hrp2jsknt
* Fix brake bug: disable brake because hrp2jsknt use lleg as supprot
* Fix pose of hrp2jsknt for new vehicle seat
* Merge pull request #435 from orikuma/jaxon-driving-pose-examination
  Jaxon driving pose examination
* support jaxon in valve-motion
* Fix approach-handle offset parameter for hrp2jsknt
* modify param and motion for sagami door
* Fix brake bug: disable brake because hrp2jsknt use lleg as supprot
* Fix pose of hrp2jsknt for new vehicle seat
* Remove approach function which is no longer needed
* Add half-sitting pose of jaxon and fix parameters for handling and accel in it
* Fix handle angle and position in polaris model
* Add move-arm option to crank initialize function
* [drc_task_common] Fix typo in stereo_preprocess.launch
* [drc_task_common] Add script to convert ar_pose/ARMarker to geometry_msgs/PoseStamped
* [drc_task_common] Remove ros::roseus from state-machine.l
* add save_with_normal
* Add projection of grasp-point to steering plane because end-coords are assumed to be same as grasp-point but torus is estimated as steering plane
* fix-get-potentio-vector-from-ocs
* integrate sagami door motion with teleop system
* add interval poses
* Add methods to apply estimated steering coords to vehicle model
* Add accessor for drive-sim-handle etc
* [drc_task_common] Support more primitive types for ocs/fc dynamic_reconfigure
* Merge pull request #420 from garaemon/dynamic-reconfigure
  [drc_com_common, drc_task_common] Add rqt_reconfigure between ocs and fc
* [drc_com_common, drc_task_common] Add rqt_reconfigure between ocs and fc
* add push motion for drill grasp
* Modify coordination of end-effector trajectory from world to BODY relative
* add missing move-target option
* change drill grasp move target
* [drc_task_common] Add scripts for steering_estimation with torus_finder
* add pre-grasp motion(grasp up
* change drill arm grasp coords
* extend door program for sagami environment
* change grasp with drill type condition
* remove bags (around finish conditions)
* [drc_task_common] use :active-state method instead of slot 'active-state' directly
* revise get-reach-drill pose
* change drill prepose
* remove bags(undefined variable)
* initial pose for drill button
* [drc_task_common] change_calib_param
* revise codes slightly
* change drill motion(impedance, pre_pose)
* Add generalized grasp-frame motion to robot-driving-motion which was in jaxon-driving-motion
* add skip recog func(almost for drill button)
* add look-at-target in ik-request
* Supress handle andgle estimation output
* Fix handle angle of polaris-xp900
* Add OCS_NS to define namespace for ocs
* Fix accel parameters for hrp2jsknt with new testbed seat
* Add some changes for new seat (testbed version) of drc vehicle.
  - move drive-init-pose-support-by-leg to robot-driving-motion
  - default stop-impedance to nil in approach-handle
  - waist-p 0 -> 10 in drive-init-pose
* modify launch and add steering_angle_marker for drive recognition
* [drc_task_common] Update laser preprocessing parameter
* Merge remote-tracking branch 'refs/remotes/origin/master' into drive
* Merge pull request #408 from YuOhara/comment_out_drill_type
  Comment out drill type
* add keywords
* remove bags(undefined variable)
* [drc_task_common] Fix small bugs for vehicle task
* Merge pull request #404 from garaemon/add-drive-state
  [drc_task_common] Add state for driving task
* Merge remote-tracking branch 'refs/remotes/orikuma/modify-operation-cmd-namespace' into drive
* [drc_task_common] Add state for driving task
* Modify namespace for operation cmd topic: staro_drive -> drive
* Fix ros package path from drive_recognition to drc_task_common
* comment out drill pose
* Add build rules for drive_recognition programs to CMakeLists.txt
* Add msg file for recognition programs in vehicle task
* Add script files for recognition programs in vehicle task
* Add launch files for recognition programs in vehicle task
* Add config files for recognition programs in vehicle task
* Add cpp sources for vehicle task recognition programs
* changed motion for new drill
* Merge pull request #400 from garaemon/not-compress-joint-angles
  [drc_task_common, drc_com_common] Do not compress joint angles from FC to OCS
* [drc_task_common] Hot fix to use hrp2016 latest model
* [drc_task_common] Add .rviz file for locomotion development
* Merge remote-tracking branch 'origin/master' into change_takenoko_drill
* change for new takenoko drill
* [drc_task_common, drc_com_common] Do not compress joint angles from FC to OCS
* Fix staro-interface path to hrpsys_ros_bridge_tutorials
* Add force compensation scripts for vehicle task
* Add scripts for handle_controller_interface in vehicle task
* [drc_task_common, drc_com_common] Use pointcloud respected from ground frame
* Modify path of euslisp script for vehicle task
* Merge pull request #397 from mmurooka/arrange-rviz-text
  [drc_task_common] Arrange rviz text
* Add eus scripts for drc vehicle task
* update rviz setting to arrange text
* arrange rviz text color and size
* [drc_task_common]remove some bags
* Merge pull request #393 from garaemon/send-odom-coords
  [drc_task_common, drc_com_common] Relay odom frame from fc to ocs
* [drc_task_common, drc_com_common] Relay odom frame from fc to ocs
* [drc_task_common] Update parameters for locomotion planning
* Merge remote-tracking branch 'origin/master' into add_cancel_motion_button
* cancel-motion button
* [drc_task_common] Add laser_preprocess.launch
* add jaxon to init function
* change to use new drill
* add_new_takenoko_drill_model
* Modify position of images
* Add new images for README
* Update base-height calculation sample for jaxon
* Update base-height calculation
* add codes for svm desicion
* Add document about test-drc-terrain-walk
* add function to wait interpolation in ocs
* add Uint8Request.srv
* do not run eus-command-server.l in fc nor ocs.
* merge origin/master and modify conflict.
* change to use fc and ocs
* enable to use Rviz angle-vector GUI with communication limitation environment
* remove unused button callback in b_control_client node
* move drill specific function in request-ik-from-marker.l to request-ik-from-marker-for-drill.l. enable to run request-ik-from-marker.l and walk-to-object.l in ocs.
* Use rleg coords instead of ee
* Add pathcalc function
* Use :angle-vector-sequence
* Update rtmsample and function names
* [drc_task_common] Update locomotion parameters and add cwd option to
  coompile footstep_planner.l correctly
* remove non used icons
* remove unused menus
* remove duplicate method :reach-until-touch
* [drc_task_common, drc_com_common] Support effort in basic info
* move deprecated launch files to another directory.
* remove launch and config files for operator sub machine
* do not generate *ri* in ocs
* add takenoko motion test-codes
* Update samplelaunch and auto-root-height function
* Add functions to check leg reachability and base trajectory
* Add ground surface for stair and terrain
* [drc_task_common] Fix hostname for fc/ocs gateway
* [drc_task_common] Fix remapping of tf and joint_states and robot_description
* [drc_task_common] Fix for smach msgs
* Update hrp2 stair sample
* Add hrp2jsk terrain walk simulation
* [drc_task_common] Do not write hostnames which are not allowed to use
* add node to calc fft of wrench
* [drc_task_common] Add launch file for locomoion planning
* add hand pose for avoid hand-collision
* Add hrp2jsk sample
* Update stair model and walking poses and add stair testing codes
* Assoc link to robot-model and fix color
* Add add-groud-p argument for terrain and stair models
* Add roll offset for walking pose
* add max-dist for reach-until-touch
* meerge origin/master
* update reach until-touch to get displacement of the limb
* [drc_task_common] Use oriented bounding box in each_link mode of robot-boundingbox.l
* [drc_task_common] Support ~analysis_level to generate bounding box of robots
* Merge remote-tracking branch 'origin/master' into add_drill_symbol_coords
* add p-control for reach-until-touch
* [drc_task_common]reach_until_touch with given initialforce
* merge remote tracking
* Add staro version terrain walk simulation
* visualize drill coords list
* add drill marker publisher
* apply drill-wall-motion to fc-ocs interface
* update color-map to be able to select grasp or connect motion
* add drill wall motion
* [drc_task_common] add coords(grasp, put) to drill model
* Add hrp2jsknt and jaxon terrain walk simulation sample
* modified comments and added exception warnings about project-coords-on-to-plane
* introduce reach-until-touch for grasping drill
* Update terrain methods and add terrain hrpsys simulation sample
* revise params for push button with middle finger
* Define terrain link as bodyset-link
* add rviz button for hook pose after 5sec
* add auto focus to subgraph mode
* [drc_task_common] Support padding parameter for robot-boundingbox.l
* Add argument to configure block dimensions and add getting face method
* [drc_task_common] Support ~links to specify links to compute bounding
  box and update locomotion.launch
* show 6-dof control default
* add gun-drill mode for genarate motion
* download model with Make Command
* add gun_drill downloader
* implement state machine subgraph
* replace gen -> gen-drc-testbed-debris
* intial commit of debris.l, gen random position and attitude model
* add color map for hose connect
* fix drc terrain order
* add car marker test code
* show_handle_with_marker
* change handle tf more static
* remove dynamic tf remapping
* [drc_task_common] Add script to generate /etc/hosts for drc
* add in launch
* add feature that supports smach viewer for visualization
* add calc drive tf
* Add drc testbed models
* [drc_task_common] Add debug mode for valve detection
* [drc_task_common] Update parameter for localization and add multisense
  standalone mode
* fix typo
* add drill put motion
* fix typo
* fi recoog codes
* fix codes for auto-gopos
* change for stable drill recognition
* [drc_task_common] Add x-y filter for locomotion planning
* remove bag in drill_recog
* [drc_task_common] Enable normal flag of handle detector
* [drc_task_common] Add hint parameter for handle detection
* [drc_task_common] Add handle detection for driving task
* [drc_task_common] Add stereo plane detection and snapit
* Add slope walking tests
* add walk codes for drill grasp
* [drc_task_common] Use nodelet manager to reduce communication amount of /tf
* Merge pull request #298 from mmurooka/drill-button-motion
  [drc_task_common] integrate drill pushing button motion to teleop system
* [drc_task_common] Remove upper pointclouds for locomotion planning
* [drc_task_common] Add normal estimation, filtering by normal and imu and estimate planer region
  for locomotion planning
* merge origin/master, debug missing function
* [drc_task_common] Use filtered laser pointcloud to localize robot
* Merge branch 'drill-button-motion' into add_markers_for_drill
* integrate drill pushing button motion to teleop system
* add markers-util for drill
* [drc_task_common] Add simple code to publish bounding box of robot
* change node name
* revise drill pos with clicked point
* Merge pull request #295 from garaemon/drill-wall-recognition
  [drc_task_common, drc_com_common] Integrate wall detection for drill task
* [drc_task_common, drc_com_common] Integrate wall detection for drill task
* [drc_task_common] Add locomotion.launch
* [drc_task_common] Convert coords set to float vector
* add drill button marker publisher
* [drc_task_common, drc_com_common] Add drill wall recognition
* [drc_task_common] Change text color on rviz according to communication status
* [drc_task_common] Respawn basic info in fc side
* [drc_task_common] Add .gitignore
* [drc_task_common] Show ocs exeucutive message on rviz
* [drc_task_common] Visualize status on rviz using OverlayText
* [drc_com_common, drc_task_common] Update minor codes to support robot_status
* [drc_com_common, drc_task_common] Change robot state type from Int32 to
  UInt8 and send robot_state in continuous low-speed path
* [drc_task_common] Update rqt perspective to show status
* [drc_task_common, drc_com_common] Watch robot movement and publish the status
  by watching /fullbody_controller/joint_trajectory_action/status topic.
* use drill urdf marker
* [drc_task_common] Download pcd models in compiling
* Merge remote-tracking branch 'ohara_remote/add_ui_for_drill_put' into icp-param
  Conflicts:
  jsk_2015_06_hrp_drc/drc_com_common/msg/FC2OCSSmall.msg
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/fc-executive.l
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/ocs-executive.l
* [drc_task_common/package.xml] remove roslint
* Merge remote-tracking branch 'origin/master' into icp-param
  Conflicts:
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/generate-drill-motion.l
  jsk_2015_06_hrp_drc/drc_task_common/package.xml
* [drc_task_common] Add roslint to avoid bug of jsk_travis
* add states for push
* [drc_task_common] Visualize state which has same context (same subgraph)
* [drc_task_common] Update drill recognition around ICP
* add states for push
* add deps to build and run drc_programs
* remove_constant_params_for_drill
* save fuji local diff temporary
* change to use icp for drill
* changed_some_params_reletate_to_drill
* add launch for detect drill_put place
* [drc_task_common] Allow state transition from
  :recognizing-look-at-point-panorama to :recognizing-look-at-point
* change service name of drill-grasp button. forget to add change.
* refactor ocs-executive.l
* change drill_sift interface to mach drill_recognition
* [drc_com_common] Use ip:=0.0.0.0 for server programs and do not use
  sudo for streamers
* add drill finder with sift
* modify parameter of hose-connect motion
* change template_cloud
* merge origin/master
* remove bags
* change stand point in the first part of hose task motion
* add drill_motion
* add generate-door-motion.l
* integrate door motion to teleop system
* add drill grasp motion generator
* Merge pull request #256 from mmurooka/change-stand-point-in-valve-motion
  [drc_task_common] Change stand point in valve motion
* comment in go-pos commnad to real robot
* enable to change stand point in valve task
* enable to change stand point in valve task
* do not launch trackball head node as default because trackball is difficult to use in communication limited environment
* merge origin/master
* merge origin/master
* Merge pull request #253 from mmurooka/enable-to-move-to-initial-from-selecting-region
  [drc_task_common] Enable to move to initial from selecting region
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_demos into add_drill_interface
* remove bags in programs
* Merge pull request #249 from garaemon/add-state-machine-for-fc
  [drc_task_common, drc_com_common] Add state machine for fc to implement timeout for recognition
* enable to transit to initial from selecting-region state
* Merge pull request #248 from garaemon/text-label-rqt
  [drc_task_common] Add StringLabel to show status rather intead of draw on image
* [drc_task_common] Disable panorama debug view on fc side
* modify motrion generation function for searching stand point
* [drc_task_command] Look around environment more aggressively
* [drc_task_common, drc_com_common] Use timeout to detect failure of detection based on
  timered-state-machine
* Merge remote-tracking branch 'origin/master' into add_drill_interface
* add exec interface(not done real robot movement)
* Merge branch 'text-label-rqt' into add-state-machine-for-fc
* [drc_task_common] Add StringLabel to show status rather intead of draw on image
* fix visualization of debri motion
* change gopos icon.
* modify hose releasing motion
* add_recog_drill_for_grasp
* [drc_task_common] Add statemachine for ocs
* [drc_task_common]add dep for drc_com_common
* merge origin/master
* send current angle-vector to rviz robot model when go-pos is commanded.
* [drc_task_common] Fix order of panorama images
* Add push/pull verification for door open
* [drc_task_common] Add timered-state-machine class to add timelimit to
  state machine
* Update walking pose and fix function name
* [drc_task_common] Add dependency to footstep planners
* Add terrain walk functions
* Add walkingp for door open funcs
* change construct of grasp-code
* add_state
* add_layout_button
* add_drill_find_launch_with_icp
* add additional modification of task motion
* Merge pull request #231 from garaemon/look-around
  [drc_com_common, drc_task_common] Look around and capture several image to build panorama view
* use common function in generating motion functions.
* Merge remote-tracking branch 'origin/master' into look-at-without-confirm
  Conflicts:
  jsk_2015_06_hrp_drc/drc_task_common/euslisp/state-machine.l
* Merge pull request #228 from garaemon/panorama-perspective
  [drc_task_common] Add Panorama perspective
* [drc_task_common, drc_com_common] Remove confirmation after recognizing point to look at
* [drc_task_common, drc_com_common] Add look-around functionality
* [drc_task_common, drc_com_common] Update ocs side to use panorama view
* Fix door open + walk functoin and comment out old sample
* Add global pose variables and update function docs
* [drc_com_common, drc_task_common] Add perspective for panorama view
* add obstacle-avoid-motion.l
* [drc_com_common] Send panorama image to ocs
* fix hose-connect motion for real robot experiment
* [drc_task_common] Compute centroid of panorama view
* [drc_task_common] Add ros::sleep in main loop
* add hose-connect motion function and integrate that motion into teleop system.
* [drc_task_common] Add panorama view by using IntermitentImageAnnotator
* Update door open testing codes ;; push + pull without door closer
* add search-stand-position-for-debri.l to get color-map for debri-task
* [drc_task_common, drc_com_common, drc_valve_task] Remove catkin.cmake
* [drc_task_common] Check if the next state is possible to move to in
  state machine
* Add test code for door open
* [drc_task_common] Update parameters for debri detection
* do not use robot-interface in ocs program
* [drc_task_common] Add debug print for continuous communication of tf transformations
* [drc_task_common] Update OCS settings for separated network
* integrate hose grasping motion to teleop system
* fix look-at and debri motion
* [drc_task_common] Remap tf and joint_states for ocs settings
* [drc_task_common. drc_com_common] Use 1-1023 port for continuous communication
* integrate debri motion to teleop system
* [drc_task_common] Update more document about setting
* [drc_task_common] Update document about sudo permissions
* fix look-at. look at valve in valve motion.
* changed topic flows to move topic once
* move robot model when go-pos pose is recognized
* enable to change and use transformable marker in teleop system
* [drc_task_common] Update document about sudo
* [drc_task_common, drc_com_common] Send compressed joint angles always as report
* [drc_com_common, drc_task_common] Send compressed joint angles from FC to OCS always
* generate primitive transformable model to rviz when valve is recognized
* add icon to refresh playing motion
* enable to send valve motion from ocs to fc under communication limitation
* Merge pull request #189 from garaemon/valve-motion
  [drc_task_common] Integrate valve motion
* Merge remote-tracking branch 'garaemon/do-not-compress-image' into valve-motion
* [drc_task_common] Integrate valve motion
* [drc_task_common, drc_com_common] Update launch file for separated machines with network limitation
* [drc_com_common] Use tunnel in default
* move reachability map source code to drc_task_common
* add error explanation to README
* remove drc_task_common/RobotCommandInterface from rviz setting
* [drc_task_common] Depends on spacenav_node
* [drc_task_common, drc_com_common] Integrate debri detection
* remove_bags_in_request_ik
* [drc_task_common, drc_com_common] Door handle detection is implemented
* [drc_task_common] Add DEBUG_VIEW argument to centroid_of_pointcloud_in_rect.launch
* [drc_task_common] Integrate valve detection
* [drc_task_common] Use one launch file for go-pos and look-at recognition
* [drc_task_common, drc_com_common] Add narrowband-message-handler to handle
  compact message
* [drc_task_common] Implement "look-at". Select region in image and look at pos.
* [drc_task_common] Disable UI based on image_view when ocs in :initial state
* [drc_task_common] Use ratio based size/location to visualize text on image_view2
* [drc_task_common] Visualize current state of ocs state machine
* Remove dependency to hrpsys_ros_bridge
* add drc_task_common/srv/GoPosCommand.srv
* add color map of reachbility
* send go-pos command from rviz using ocs-executive.l
* change not use tf_listener
* change size of photos, add Some more text in README
* [drc_task_common] Add images and look-at button to ui
* [drc_task_common] Choose Location to go by image with network limitation
* change input in launch
* add deps for request_ik_from_marker
* add dep for jsk_ik_server
* add launch for hrp2jsknt
* change codes for jsk_ros_pkg
* add hrp2jsknt robot option
* [drc_task_common] Implement go-pos functionality satisfying limited
  communication
* [drc_task_common] Add state machine class based on state-machine of roseus_smach
* add marker pics
* add readme for drc_task_common
* [drc_com_common, drc_task_common] Add image_view2 based user interface. first step of system integration towards DRC final
* search rotatable position for valve
* add hrp3hand grasp and desicion function
* update to use jsk_recognition_msgs
* add code to support yaml both 0.5.0, 0.3.0
* Use jsk_recognition_msgs in drc packages
* add recognition parameter for valve with multisense
* add launch file for staro
* removed bags
* merge master branch
* add param to clarify the program
* add push with force-sensor
* add arguments for multisense setting
* enable to switch target robot from launch file argument
* remove argument to set trackball device file
* add codes to push many times
* not use :potentio-vector methods
* modify codes around move
* add particle filter_based revise model pose
* sumirize codes for grasping parameter
* allow slip in moveing
* add_t_marker_info_publisher
* remove some ros_info codes used for debug
* summarize code in functions(not changed default functions)
* removed some removable codes, removed error
* add manipulation data server in operator station
* add push function with grasping
* changed to grasp ik-arm only
* add drill_grip function
* change to change ik stop nums
* add dynamic reachability
* rename misspell names
* renamed executable map to inverse-reachability-map
* clean code around inverse-reachabirity
* clean program slightly(removed same codes by defining function, changed stop parameter for ik)
* added config for manipulation_data_server
* added dual-arm ik z_free
* added dual-arm-interface
* added midiate grasp pose
* added functions to reset model
* added manually_pose_set mode
* added :z_free ik solution
* changed some parameters to grasp correctly
* changed reaching time for not loosing balance
* added axial-restraint ik
* fixed to do reaching to object
* added axial-restraint interface for ik
* add dependency on jsk_teleop_joy
* added reverse_hands cb
* renamed some funcs and variable
* changed cmake
* added manipulation_data visualize node
* added executabl_marker
* added wait_for_Transform function
* added codes to change coords when arm is different
* changed to use dynamic_tf_publisher
* devided tf_publisher into the different node
* added dependencies in drc_task_common
* fixed bugs with pcl_points initialization
* added color_histogram matcher in launch
* add hrpys service to pass setting
* add launch script for ocs and fc
* added more funcs to solve ik from pose
* fixed installation in catkin.cmake
* added dependency
* Revert "Revert "add drc teleop demo program""
* Revert "add drc teleop demo program"
* removed test_codes for debug
* renamed topic names, removed left and right name
* added callback to solve ik with pose_msg
* added manipulation_data_processor in launch file
* added assoc_function
* add drc teleop demo program
* Contributors: Eisoku Kuroiwa, Yuki Furuta, JSK Lab Member, JAXON, JSK, Kamada Hitoshi, Kei Okada, Kentaro Wada, Kohei Kimura, Masaki Murooka, Ryo Terasawa, Ryohei Ueda, Satoshi Iwaishi, Shunichi Nozawa, Yu Ohara, Yusuke Oshiro, Yuto Inagaki, Chi Wun Au, Iori Kumagai, Iori Yanokura, Kouhei Kimura, Satoshi Otsubo, SHintaro Noda, Yoshimaru Tanaka
