^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package detect_cans_in_fridge_201202
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------
* [detect_cans_in_fridge_201202/euslisp/pddl-action.l] fix missing options
* [detect_cans_in_fridge_201202/CMakeLists.txt] add roseus to find_package
* Merge pull request `#1152 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1152>`_ from furushchev/fridge-fix-interactive
  [detect_cans_in_fridge_201202/euslisp/main.l] bugfix: prevent calling (enable-behavior-server) many times
* [detect_cans_in_fridge_201202/launch/gazebo_startup.launch] add launch file for gazebo
* [detect_cans_in_fridge_201202/detect_cans] add plane extraction for fridge inner shelf substraction
* [detect_cans_in_fridge_201202] add debug-view argument for preventing irtviewer launch
* [detect_cans_in_fridge_201202/euslisp/match-hist.l] update parameter for match hist lower cutting threshold
* [detect_cans_in_fridge_201202/euslisp/main.l] bugfix: prevent calling (enable-behavior-server) many times
* [detect_cans_in_fridge_201202] add debug-view argument for preventing irtviewer launch
* [detect_cans_in_fridge_201202] remove old/unused codes; cleanup directory
* [detect_cans] add README
* [detect_cans] async join based parallel state-machine
* Contributors: Yuki Furuta, Kamada Hitoshi, Ryohei Ueda

0.0.2 (2015-11-26)
------------------
* [jsk_demos] exec fridge demo in smach
* [jsk_demos] exec fridge demo in simulation mode
* [detect_cans] set *current-context
* [detect_cans] change the load place
* [detect_cans] fix arg in startup.launch
* [detect_cans_in_fridge201202/euslisp/main.l] add logging option
* [detect_cans] visualize GaussianPointCloud of fridge
* add documentation string
* euslisp/color_histogram_creater.l: use eus-pointcloud instead of 3dpointcloud
* [detect_cans_in_fridge_201202 & jsk_demo_common] change name space openni -> kinect_head
* [jsk_demo_common] divide-fridge-func
* [detect_cans_in_fridge] modify launch remap
* [detect_cans_in_fridge] change from openni_c2 to kinect_head_c2
* [jsk_demos] fix typo
* Contributors: Kamada Hitoshi, Kei Okada, Yohei Kakiuchi, Yuki Furuta

0.0.1 (2015-06-11)
------------------
* [detect_cans.l] load-ros-manifest self package
* [jsk_demos] remove rosmake files
* [detect_cans_in_fridge_201202] ind_package component is not necessary
* remove rosmake settings
* add build dependencies for catkin build; fix typo install
* catkinize jsk_demos
* [detect_cans_in_bridge_201202] remove hydro_detection.launch. now it
  run in default
* fixed perception.launch
* adapt attention-clipper for fridge demo
* Add small launch file to run interactive_behavior on pr2
* add pddl planner mode demo
* add pddl supported launch
* Fix typo :heck -> :neck and add sound-localize.l to launch file
* Add script and desktop icon for pr2 tablet demo using surface
* suppress annoying logging
* changed perception.launch
* perception.launch
* modify launch
* add fridge2
* add fridge-marker
* divided startup.launch
* crop
* changed open-fridge-door function in pr2-action.l
* Load pr2_gripper_sensor_msgs by ros::roseus-add-msg-files
* Fix package name, topic names for hydro
* Use pcl_msgs on hydro instead of pcl
* Run behavior server and use it from detect_cans demo
* change the threshold to apply snapmap
* run main.l on c2
* update detect_can demo
* update program for match histogram
* add new object_models after debugging match-hist.l
* add some comments
* add tea_powder object
* update get-bin function
* add description to match-histogram
* add descriptions to object_models
* fridge demo should use map
* remove loading pr2_semantic/actions.l
* change load -> require
* use-arm keyword to actions for choosing arm to grasp can
* add auto exit if using app-manager
* remove read-from-string
* add debug message
* rename app_execute/object to app_execute/target
* rename type -> atype
* change detection_msgs publish style, objects in one scene should be published at the same time
* remove global variables
* rename param for multitask
* fixed the bug
* update main.l for using various object via android
* update startup.launch for using various object via android
* update detect can package
* update startup.launch to be compatible with app_manager
* add initialize-demo function
* do not use :use-torso for limb :inverse-kinematics method ;; behavior will not change because :use-torso was neglected at the previous revision
* remove comment out codes and update code for initializing robot
* update parameter
* add scripts for speaking english
* fix detection parameter
* update image for detecting fridge
* added rviz config for groovy
* fixed topic name in rviz config
* add keyword for fixing torso-lift and head-pitch
* modify for choosing demo-type from launch
* update japanese speaking
* add dpends to jsk_pr2_startup
* fixed wait-query mode
* fix: speaking correct name
* use jsk_pcl_ros_unreleased -> jsk_pcl_ros
* update index.rst,conf.py by Jenkins
* change default behavior of detect_cans startup.launch
* update index.rst,conf.py by Jenkins
* add API for starting demo from appchooser
* use require for loading files
* refine function names and samples
* fix for jsk_demo_common
* add camera
* fix Display name
* move action/move functions to jsk_demo_common
* remove main-old.l
* update motion for robustness
* added dependency on SnapMapICP
* update index.rst,conf.py by Jenkins
* add short course demp
* refine parameters for openning fridge
* add timeout for look-around-pr2
* add look-around-pr2.l for capturing several pointclouds
* refine parameter
* default topic name cheanged
* unsubscribe if there is no listener
* change machine for processing shift
* fix: speak-jp
* add speak-name
* using new feature for detecting fridge
* update detect_cans.vcg
* update: debugging demo
* update index.rst,conf.py by Jenkins
* add arguments to startup.launch
* add debugging message to detect_cans../main.l
* update index.rst,conf.py by Jenkins
* removed rectangular and added cropbox
* pddl functin for detect_cans_demo added
* change parameter for ideal fridge coords
* add speak words
* change for using check-detection in detection_interface.l
* removed specific topic name
* added sample function for wait query
* add use-arm-navigation flag
* update for using arm_navigation
* add arm_navigation to grasping can
* update test code
* fix diffcds calculation ,pr2 and object has origin coordinates as reference
* update index.rst,conf.py by Jenkins
* check if look-transform works
* add comment
* add euslisp/test-go-to-fridge.l
* update add obstacble
* update index.rst,conf.py by Jenkins
* rename main2.l -> main.l, and rename old main.l to main-old.l
* add object_detection_marker_array
* add Spot Array Marker
* update launchdoc in startup.launch
* add launch/rviz.launch
* update index.rst,conf.py by Jenkins
* add detect_cans.vcg for rviz
* update index.rst,conf.py by Jenkins
* add test_perception.launch
* update index.rst,conf.py by Jenkins
* fix some bugs
* outout launchdoc-generator to build directry to avoid svn confrict
* fix bug and update parameters
* remove loading jskgeo
* extract action functions from demo function
* add detect_cans.launch
* do not compile jskgeo.l
* remove depend to rectangular_solid_filter
* add rosdoc
* removed nodes concering knowrob and openrave
* fixed object model name typo
* update for demo 2012.4.6
* remove dependancy to white_balance_converter
* update for embeded irtpointcloud.l
* changed pre-grasp arm pose
* add detect_cans_in_fridge.vcg
* fixed indent
* update demo script
* update object color histgram
* add main2.l for demo program
* add comments
* add comment
* removed a node in startup launch
* update demo script
* demo package moved from internal repo
* Contributors: Yuki Furuta, JSK applications, Kamada Hitoshi, Kei Okada, Ryohei Ueda, Haseru Chen, Shohei Fujii, Yusuke furuta, Hitoshi Kamada, Kazuto Murase, Manabu Saito, Hioryuki Mikita, Shunichi Nozawa, Youhei Kakiuchi
