^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_2011_07_pr2_semantic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------

0.0.2 (2015-11-26)
------------------

0.0.1 (2015-06-11)
------------------
* [jsk_demos] remove rosmake files
* catkinize jsk_demos
* Not include nonused pr2eus_openrave
* do not use :use-torso for limb :inverse-kinematics method ;; behavior will not change because :use-torso was neglected at the previous revision
* use require for loading files
* update demo actions
* add path-div
* add time-tick and grasp-check keyword to open-fridge
* add :return-sequence to open-fridge
* add :wait-time keyword to open-fridge
* refine parameters for openning fridge
* fix typo
* refine open-firdge
* using new feature for detecting fridge
* changed topic name of markers
* update for checking traget-name
* update check function, check timestam that the object is detected after check function is called, fix bug to confirm that object detection result is stable
* publish objectdetection results
* moved jsk_pr2_gui to jsk_ipad_gui, messgaes used in src/test_webui.l  will be merged into jsk_gui_msgs
* divided perception nodes to start_perceptin.launch
* update for demo 2012.4.6
* open gripper while angle-vector-sequence of open-fridge
* speak-jp with object name
* check x::*display* is NULL ?
* uddate action definition
* fixed the grasp motion for cups
* not to use kinect pointcloud data in my demo launch
* changed many lines
* changed many lines
* add util for pareto distribution
* delete speak-jp method, which is loaded in pr2-interface.l
* update current success version drobot actions
* update actions.l and launch for demo
* pointcloud_screenpoint changed to pointcloud_screenpoint_nodelet
* commit updates for demo
* commit all for demo
* update action.l in demos/jsk_2011_07 for pick controller
* add controller mode to pick method
* update demo scripts
* update opening fridge script
* update script for bigmac and fridge
* add coords definition to grasp sandwich box from top
* add bigmac sandwich package texture to jsk_2011_07 package
* update the planning domain and output graph
* add planning based demo script
* update demonstration scripts
* fixed typo
* added callback for ipad knowrob viewer
* added cup-mit.jpg
* update documents for 2011/07 demo package
* back to original position when no cup found
* add launch for c1
* changed motion of hand-over
* added avs for hand-over function
* update demo launch, script
* changed hand-over position
* check object pose again if armplanning failed
* look 4sec/cand when object search, fix the making reversed motion
* add debugmessage, change inflation-range from 0.15 to 0.20, lookat hand befor pass the cup
* use include to launch external scripts
* remove machine tags
* updated webui callbacks
* modify cup pose in mini-kitchen, update demo script
* update demo script
* use_asynchronous for reduce computational cost
* commit cup type and scripts
* added new images
* update pr2_semantic demo script
* add object type of cup in pr2_semantic_demo
* update demo programs for pr2_semantic
* add find-objects-by-distance
* add timeout to check object method
* add viewer_window option to disable the OpenCV window
* empty window name to disable window, point_pose_extractor
* include all nodes in one launch file
* user openrave for reaching cup, grippert sensor for hand-over action
* renamede sift template of launch file
* added new image for cups and mugs
* add similar object find method
* added noimage.png
* applied ri move-event using knowrob from iPad
* typo filename jpg -> png
* add mit-mug to jsk_map:scene1
* move eus_json_prolog to json_prolog in tum repository
* change eus prolog interface to extract knowrob:oritentation from prolog answer, add launch load option to json_prolog
* add find-knowrob-objects-with-info method, and current demo program
* fixed typo and renamed old service name
* add action of pick and grasp the cotesys cup
* added str-cb from iPad on test_webui.l
* added sample for webui
* commit current demo elements
* add demo package for pr2 semantic demo
* Contributors: Kei Okada, Ryohei Ueda, Haseru Chen, Manabu Saito, Hiroyuki Mikita, Syunichi Nozawa, Youhei Kakiuchi
