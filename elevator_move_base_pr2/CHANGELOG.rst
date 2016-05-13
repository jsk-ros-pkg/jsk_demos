^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package elevator_move_base_pr2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------
* [elevator_move_base_pr2/src/database-interface.l] add logging feature for elevator demo
* Contributors: Yuki Furuta

0.0.2 (2015-11-26)
------------------
* [elevator_move_base_pr2/launch/elevator_move_base_modules.xml] use rectified image on light detect
* [elevator_move_base_pr2/launch/check_elevator_open.xml] rename /openni_c2 -> /kinect_head_c2
* Contributors: Yuki Furuta

0.0.1 (2015-06-11)
------------------
* [elevator_move_base_pr2] use octree_change_detector to check elevator door is open
* [elevator-move-base-pr2] add move-inside state machine
* [elevator-move-base-pr2] remove / from tf frame
* [elevator_move_base_pr2] add missing deps/ build recipes
* [elevator_move_base_pr2/src] remove root '/' from lookup tf
* [jsk_demos] remove rosmake files
* catkinize jsk_demos
* crop
* changed navigation-client.l and elevator-move-base-main.l
* fix pr2eus link name(append _lk)
* add imagesift to elevator_move_base_modules.xml
* update index.rst,conf.py by Jenkins
* use use-tilt-laser-obstacle-cloud in pr2-interface.l (r4204)
* comment out test-modules-callpanel, since send *ri* :state :worldcoords returns nil due to tf error
* fix test color-point-detect without X
* update index.rst,conf.py by Jenkins
* fix test-color-point-detector and add test code
* fix compile error
* fix for groovy
* update index.rst,conf.py by Jenkins
* outout launchdoc-generator to build directry to avoid svn confrict
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* changed the port of mjpeg_server in test, 8080 -> 8910
* momove --ctx 0 option
* update test file arguments
* use video_directive
* use rosbuild_check_for_display
* fix launchdoc
* changed gif file name in sphinx document
* mp4->gif for html
* a node has moved virtual_camera -> jsk_perception
* add bag file and scripts for test-modules-callpanel
* changed to use mjpeg server capture
* add doc generator cmake command
* add unit test by bag for button light detector
* add dummy test for elevator button light detector
* generate pose detection video for make test
* add rviz setting file for test movie
* add test for generate rviz mp4
* fixed orientations of spots
* add navigation function for through the automatic door in eng2-2f
* add test for panel-detection, and fixed debug image of button-color detection
* split launch for elevator_navigation, to test modules
* delete speak-jp method, which is loaded in pr2-interface.l
* clean up src of elevator_move_base
* remove old header file include
* update tilt-laser on/off for move_base (switch-global-planner-observation), push button deeper (10mm -> 30mm)
* add filter-type for pose estimation
* add sample for detection launcher generator
* update change-floor method smart
* topic name should be changable, add time-stamp of pose detection to object
* fied methods in elevator-move-base
* removed navigation-client-addon.l
* added test, split main script into main/methods
* change to use timestamp for getting correct panel pose
* update manifest for electric, remove useless state-machine-node class
* add addon script for elevator-move-base
* commit for current scripts
* not to care about which arm is to be free in tuckarm pose
* add debug image publisher for detector nodes
* change using arm for pushing elevator button when grasping object
* change to use stringstamped in roseus
* fix the pushing method and floor name string
* add message name to constant in msg definition
* move call-empty-service, clear-costmap, initialize-costmap, change-inflation-range to roseus-utils
* add / to result of elevator number detector
* add ** to constant in msg
* update elevator panels number detection for eng2
* fix to use template match method in matchtemplate.py
* add code for eng2-map demo
* add depend package, virtual_camera package
* add script for visualize pr2 elevator demo
* add debug image publisher in match template
* check the first quartile in color distance
* not to confuse 1F and B1F in elevator, and prolong timeouts
* see narrow area for the button color checker
* change the order of (costmap-normal) and (speak-jp)
* move pr2 tuckarm pose function to pr2eus/pr2-interface.l
* add tuckarm pose when start moving
* add verbose message
* fix typo, c&c error
* fix typo in costmap-for-elevator
* unify function name rule for costmap(s)
* change elevator-move-base to use action-client direct
* change pr2eus camera name, tune B1F elevator position, modify floor check function
* fix floor check method, and depend package name
* set camera namespace for ObjectDetection, and fix typos
* change to renamed include launch file package
* remove DB insertion code
* change posedetectiondb class
* move imagesift node under the camera namespace
* change the name of load script
* modify to use 8gokan-map-scene
* change name smach_roseus -> roseus_smach
* rename smach_roseus -> roseus_smach
* SIFT template macher will launch here
* fix DB access method
* fix many typos, and set machine tag for pr2machine
* copy elevator_move_base for pr2 from pr2eus_sample
* Contributors: JSK applications, Kei Okada, Ryohei Ueda, Yuki Furuta, Manabu Saito
