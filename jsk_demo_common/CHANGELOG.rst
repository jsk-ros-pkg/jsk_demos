^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_demo_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2016-02-11)
------------------
* [jsk_demo_common] fix wrong code
* Merge pull request `#1124 <https://github.com/jsk-ros-pkg/jsk_demos/issues/1124>`_ from h-kamada/add_reset_pose
  pr2-action.l: apply reset-pose before open-fridge-func
* pr2-action.l: apply reset-pose before open-fridge-func
* [detect_cans] async join based parallel state-machine
* Contributors: Kamada Hitoshi, Kei Okada, Yuki Furuta

0.0.2 (2015-11-26)
------------------
* jsk_demo_common: add pr2_gripper_sensor_msgs
* [jsk_demo_common] state_publisher -> robot_state_publisher
* [jsk_demo_common] run depends for model_filter.launch
* [jsk_demo_common] Filtering model launch file
* [jsk_demos] exec fridge demo in smach
* [jsk_demos] exec fridge demo in simulation mode
* [jsk_demo_common] add action logging feature to attention-action; add pr2-attention-action.l, pr2-attention-move.l
* [jsk_demo_common/euslisp/pr2-action.l] fix: use :link to link name
* [detect_cans_in_fridge_201202 & jsk_demo_common] change name space openni -> kinect_head
* [jsk_demo_common] remove rosbuild related files
* [jsk_demo_common] fix error in simulation-modep part
* [jsk_demo_common] change from :joint-action-enable to :simulation-modep
* exec in simulation mode
* [jsk_demo_common] divide-fridge-func
* [jsk_demo_common/euslisp/pr2-action.l] fix hand-over for issue `#381 <https://github.com/jsk-ros-pkg/jsk_demos/issues/381>`_
* remove dependency to jsk_2013_10_pr2_73b2 package
* Contributors: Kamada Hitoshi, Kei Okada, Kentaro Wada, Yuki Furuta

0.0.1 (2015-06-11)
------------------
* [jsk_demo_common] add pr2 actions' action servers
* use pr2_gripper_sensor_msgs instead of dummy for hydro in pr2-action.l
* [jsk_demos] remove rosmake files
* fix typo symbol
* fix: conflict key arg with class in irtgraph
* add loop-check-error-p option
* add README.md
  add brief description and usage of sample codes.
* add preemptive-task-execution and its sample
* add provide attention-observation
* add sample directory
* eql -> eq
* support submachine in loop check
* add option to grasp; pre-grasp, post-grasp move
* repair return-from target funcition in pr2-action.l
* add :outside option when close fridge
* catkinize jsk_demo_common
* changed pr2-move.l
* divided startup.launch
* modified pr2-action.l
* modified open-fridge-traj in pr2-action.l
* modified open-fridge-door function in pr2-action.l
* modified open-fridge-traj in pr2-action.l
* changed closign door motion in pr2-action.l
* changed closing door motion in pr2-action.l
* changed openning door motion in pr2-action.l
* changed open-fridge-door function in pr2-action.l
* added func-time in pr2-action.l
* added func-time in pr2-action.l
* modified open-fridge-door function in pr2-action.l
* (#17) workaround for hydro/euslisp-only environment
* update topics for detecting
* add preemptive-action-server
* update
* add hand over function
* remove loading pr2_semantic/actions.l
* add describing demo
* use-arm keyword to actions for choosing arm to grasp can
* update robot actions
* avoid collision when pr2-reset-pose
* update pick-tray and place-tray safe for *pr2* move
* improve pick-tray and place-tray
* bugfix: dont go-pos-unsafe if tray is 180deg rotated
* bugfix: tray position
* update parameters
* udpate pre grasp pose
* update close firdge motion
* update open firdge motion
* add torso-height keyword to pr2-pick-tray-pose
* add pick-tray to decide position using detection result
* add comments
* rename type -> atype
* [place-tray] option use-base-scan set default value nil
* add provide
* fridge demo after moving *pr2*; bugfix grasp-can; pick/place z-axis ver.
* use shortname for tilt-laser-obstacle-cloud
* add hold-chair
* unsubscribe *base-scan-id*
* add move-with-base-scan
* add pick-tray to pr2-action
* add place-tray to pr2-action
* add pr2-pose.l for pose database for pr2
* add move-to-spot
* update parameter
* update attention observation programs
* add tablet attention node
* add sound attention node
* fix typo
* add loop check
* minor update
* add command with recover
* update parameter
* do not use :use-torso for limb :inverse-kinematics method ;; behavior will not change because :use-torso was neglected at the previous revision
* add functions using smach
* update ros-wait
* replace sleep to :ros-wait for making interruptible
* update attention-observation
* add methods for gripper
* add wait-android-query for interrupt
* add func-before-throw
* override :ros-wait at attention-interface
* check catch barrier before throw
* add methods for interrupting by attention-observation
* add scripts for speaking english
* update japanese speaking
* add keyword for fixing torso-lift and head-pitch
* add put-can-on-turtlebot2
* add put-can-on-turtlebot
* add move-to-initial keyword to pr2-look-around
* fix: parameter for actions
* add pr2-look-around for looking around
* fix for preparing pose
* update demo actions
* add :open-fridge-func keyword to change basic function
* moved go-pos-unsafe from close-fridge to grasp-can
* fixed dependency
* add test/
* add jsk_demo_common/
* Contributors: Yuki Furuta, JSK applications, Kamada Hitoshi, Kei Okada, Ryohei Ueda, Yuto Inagaki, Kazuto Murase, Hiroyuki Mikita, Shunichi Nozawa, Youhei Kakiuchi
