^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_2013_04_pr2_610
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------
* jsk_2013_04_pr2_610/README.md: without space, it breaks kanji-code in emacs?
* jsk_2013_04_pr2_610/README.md: fix section/subsection
* update to use ROBOT env value to select machine file
* Contributors: Kei Okada, Yuto Inagaki

0.0.2 (2015-11-26)
------------------
* README.md: now jsk_2013_04_pr2_610 depends on task_compiler
* Update README.md: source donwload xdot
* Update README.md  solves https://github.com/jsk-ros-pkg/jsk_demos/issues/1117
* add rostest for demo program for pr2_610
* [README] update jsk_2013_04_pr2_610 demo readme
* [jsk_demo_common] fix error in simulation-modep part
* [jsk_demo_common] change from :joint-action-enable to :simulation-modep
* [jsk_demos/jsk_2013_04_pr2_610] add use_sim option to launch files for launching on non PR2 environment
* Contributors: Yuki Furuta, Kamada Hitoshi, Kei Okada

0.0.1 (2015-06-11)
------------------
* [jsk_demos] remove rosmake files
* add catkin build dependencies for 610 demo
* jsk_2013_04_pr2_610 only depends on roseus for build process
* switch catkin.cmake when use catkin
* catkinize jsk_2013_04_pr2_610
* add geometry_msgs to dependencies
* add multi planner option in demo launch
* change chair assoced link name
* replace r_wrist_roll_link to r_wrist_roll_link_lk
* now no need special client to use pddl downward
* support downward planner
* bugfix: invalid funcall in push-chair
* add readme
* fix type
* update location of loading setup sequence
* update action.l
* move pddl-action-fuctions to action.l
* move loading file location
* add functions for smach
* rename pddl action close -> close-door, bacause close overwrite euslisp function
* add plan-demo-smach.l for using smach as a plan executor
* add switch plan or not in launch file
* do not push-back chair
* add smach
* more-open-door laundry
* add description_wash.l
* rename sweep-under-table to sweep-under-table-front
* move raise-mop from place-broom to sweep-room
* disable metric of description.l
* add 't' at each function for pddl
* debug description.k
* debug
* debug for pddl
* add pddl total-cost metric, divide pddl object 'table' -> 'table-front'/'table-side'
* add sweap macro
* change demo to planning
* debug of function renames
* add push-chair function
* add planning demo
* rename functions
* repair setup-for-pddl.l
* add plan.l, plan-graph.l
* revert to commit by furuta-jsk
* assoc *broom* after bringup-broom, go *pr2* forward before placing tray
* fix *table* position
* move demo in *ri* simulation
* speak task before doing
* refactor sweep-under-table-dual-arm
* change torso height when pick up bloom
* fix demo_*.app, syntax of launch/icon section is [package]/[filename]
* look at cloth when pass and change sweep init pose
* add  launch file for planner
* update icon
* fix paths for appchooser
* change sweep motion and position
* remove bag in sweep-under-table-init-slim
* fix typo
* use angle-vector sequence when put cloth
* fix pick-cloth-larm: go back if fail to grasp
* fix a little due to [#1815]
* add forgotten parenthesis
* dont use torso when pick broom
* do not set the rarm pr2-tuckarm-pose-rarm-free
* change the way of set the rarm off of the broom
* use angle-vector-sequence when open laundry
* fix demo2 improved by furuta-jsk/s-fujii
* return nil if pr2 fails to pick cloth by larm
* skip tuckarm pose at the initialize of move-to-chair-larm
* dont wait interpolation after place tray
* change position of putting tray
* fix typo in move-to-chair-larm
* use angle-vector-with-constraint in pick-broom
* use turtlebot_big.jpg under jsk_perception #173
* enable tray detection for pic-tray #173
* enable tray-detection
* look at tray in pr2-pick-tray-pose, #173
* pick cloth with larm and pass to rarm
* move arms after open hands in place-tray
* repeat pick-tray if fail
* fix #213
* change rotation angle and rotation axis when pull chair
* fix yokei-down-height
* dont detect laundry in simulation
* fix previous furuta-jsk commit
* dont check grasp in simulation
* dont detect chair in simulation
* add raise mop fucntion in util.l
* change length of putting forward a tray
* ignore checking tray in simulation
* up torso before place tray
* lower the pos of catch chair and do not go-pos backward on pulling chair
* fix grasp check on pick-tray
* put forward a tray before putting down
* fix launch script for footobject, see #199
* tray-detection is not impremented yet
* revert tray-detection
* add move-arm
* bugfix: fix typo
* bugfix xml 'if' error
* fix malform of xml
* remove old launch; generalize detect_with_image.launch
* debug move-chair
* integrate demo files to 1 file
* rename launch files - to _
* now available for app_chooser
* add code fir app_chooser
* add macro setup-for-pddl.l
* delete unused code; function move-to-* and pick-tray returns t if success or nil if not;
* add depends to pddl_planner
* delete test.l
* add test.l
* add tray image recognition; common image detection launch file
* add detect-with-image.launch
* add test-detect-chair.l
* add detect-foot-object.l
* modify chair grasp problem
* change tray-spot x -= 100
* rename app -> apps
* modify detect-foot
* debug test-particles
* modify test-particles
* make example-filter
* test example for pfilter
* add particleFilter
* add new msg type
* add joy-move.l and detect-foot.l
* set roseus name to jsk_irt_demo
* little change
* commit 2013/6/12 demo version
* update with cost
* add speak
* fix for demo; add app for app_manager
* fix for demo
* break open laundry door
* add table launch
* add detect-tabls s
* modify
* try to open laundry
* change topic name scan_filtered2 -> scan_filtered_foot, all_input_marker_array -> detect_chair_debug_marker
* this is not needed
* modify sweep-under-table and move-chair
* minor changes
* move-chair change to grab side || a little change in sweep-under-table
* change sweep-under-table's last and init func
* make pick-cloth speedy and change some go-pos
* change inflation
* update  sweep-under-table function [#181]
* dissoc before exit function [#177]
* fix for casing grasp [#177]
* add comment to how to test [#177]
* in pick-broom (grasp-broom), we use :rotation-axis t, fixed [#177]
* fix indent for debug
* move pick-brooms-spot, more closer to the wall [#177]
* fix grasp-broom, do not exit from function whith assced object, dessoc before exit and assoc again in next function, check if the robot grasp broom using return value of start-grasp and returns from function
* pr2-reset-pose, before :stop-grasp, since pr2-reset-pose wait-interpolation [#177]
* check if ik is solved, retry 3 times [#177]
* add detect-all.launch that start detect-chair and detect-laundry [#182]
* add comment and ros-info
* little arrange in move-chairs
* debug of assocs
* debug delete extra interpolation
* bugfix: rotating wrist unexpectedly during put-cloth-into
* fixed ticket:[#172], retry unless grasping broom
* fixed ticket[#170]
* debug move-chair
* make chair-detect better
* add msg and repair chair-detection
* add check-chair-marker
* add check-marker function
* add test-publish-marker.l
* debug little change
* remove move-chair-back.l
* infration value change
* topic name repair
* bug fixed position of laundry
* little modify in detect-chair
* change params in detect-chair
* debug in detect-chair
* move-chair-back
* In detect-chair add limitations
* propdel svn:executable from detect-laundry.launch
* add depend to laser_filters_jsk_patch, jsk_perception
* fix move-to-laundry: remove move neck-p
* add detect-chair.launch
* rotation-axis :z -> t in grasp-broom
* implemented pick-broom.l
* In move-chair get rid of do-until-key
* remove detect_laundry.launch
* delete shadow_filter_example.yaml~
* new pick-broom.l with image processing
* merge confict
* new parameters for shadow_filter
* move-chair was repaired
* merge conflicted
* add sweep-under, put, open, close pull, push, push-button [#89]
* add pddl/plan.l
* do not execute detect-chair when loaded
* clean up obsolete files
* move launch files under launch directory, change euslisp file name with _ to -
* detect chair with using objectDetection
* laundry recognition success using narrow_stereo left
* move-to-chair-bacl modify
* add move back functions
* add test code test-detect-laundry.l
* bugfix: publish-laundry-marker.l
* add publish-laundry-marker.l
* test-detect-laundry tf publish
* use :object keyword to pick only tray
* detect_chair can publish ObjectDetection
* calc chair centor pos
* modify marker_laundry_cut.jpg
* add marker_laundry
* add image processing to put-cloth-into-laundry
* add detect_chair.l
* add test-detect-laundry
* tilt chair more smoothly
* bugfix:typo miss in app.launch demo.l
* In sweep-under-table add some inverse-kinematics process
* move-to-laundry modify to more simple
* [#126] bug fix: unnecessary comment out in move-to-laundry and modify pos of *laundry*
* minor bug fix
* in move-to-sweep tuck right. sweep-spot modify
* add draw object in pick-broom
* change move-chair to display IRT viewer
* add move floor spot
* add marker of laundry
* clear-costmap after disable tilt
* change kitchen-table height in place-tray.l, change behavior after pick broom in pick-broom.l
* do not need to set link-list https://sourceforge.net/p/jskeus/tickets/20/
* simultaneously change pose in move-to-table
* commit
* clean up plcae-tray codes [#108]
* add test code
* clean up plcae-tray codes [#108]
* add change-inflation-range and clear-costmap at setup.l
* use https://sourceforge.net/p/jskeus/tickets/12/, https://sourceforge.net/p/jskeus/tickets/22/
* resolved conflicts
* clean up other codes [#108]
* clean up pick-tray codes [#108]
* move objecs before setting up robot-interface [#108]
* move world-to-610 in setup [#108]
* use (setup) function to initialize demo environment
* rename from switch-global-planner-observation to use-tilt-laser-obstacle-cloud, #94
* params repair in move-chair
* param chousei for chair
* sweep-under-table disenable regrab and move chair-spot and chair
* add moving mop function in furuta-sweep
* fixed package name
* at move-chair , we pull chair back
* refactor furuta-sweep.l
* add guard to irtviewer
* add tilt off function switch-global-planar-observation
* modify sweep undertable
* pick-broom success
* modify move-around function in furuta-sweep
* merge conflict
* last update of setup.l for move-chair
* modify sweep under table
* add yukizaki's function to setup.l
* add demo.l for app launcher
* add file for move-chair
* added place tray function
* add move-chair-back
* y-tanaka-pick-tray.l has been added and loaded from setup.l modified
* update iwaishi-pick-croth.l
* gripper only to set rarm in move-chair
* update (iwaishi-pick-cloth)
* change move-around in furuta-sweep
* modify setup.l
* laundry
* ik
* hoge
* add app settings and icon
* add init function in put-cloth
* add go-pos-unsafe after move-to
* add furuta sweep function
* update put-cloth-into-laudry.l
* a bit param change for move-chair
* added sweep-under-table
* add revert-if-fail to put-cloth-into-laundry.l
* update (iwaishi-pick-cloth)
* add (iwaishi-pick-cloth)
* debug move-chair
* add laundry model
* add iwiishi pick croth
* update put-cloth-into-laundry.l
* move-chair bug fix
* move-chair arrange
* bug fix dissoc of move-chair
* change *broom* bind from room73b2 to room610
* add location to open-laundry.l
* pick-broom success on sim
* not mv, but svn mv
* correct filename
* update open-laundry.l
* model no koushin
* move-to
* unload open-laundry.l
* clean up code and filename style
* add inagaki
* add yukizaki
* add move-chair
* update move-to-chair
* do not show room610 in objects
* add (move-to-table)
* add assignee names
* show room610 model in pr2-interface ,need to update jskeus
* fix typo laundary -> laundry
* bug fix
* add utilities
* add close-laundry-door
* complete the part that are in charge of kuroiwa
* add kuroiwa.l
* なんとなく持った。
* add banzai pose
* とりあえずモップを持つはず。
* fix typo bloom -> broom
* add jsk_2013_04_pr2_610
* Contributors: Yuki Furuta, Kei Okada, Ryohei Ueda, Yuto Inagaki, Shohei Fujii, Yusuke Furuta, Satoshi Iwaishi, Eisoku Kuroiwa, Hiroyuki Mikita, Chen Wesley, Yoshimaru Tnaka, Youhei Kakiuchi, Sou Yukizaki
