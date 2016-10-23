#!/usr/bin/env roseus
(ros::roseus "planner_demo_main")

(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")

(load "planned-state-machine.l")

(load (ros::get-param "~action"))

(when (fboundp 'task-compiler/after-init-hook)
  (task-compiler/after-init-hook))
(when (fboundp 'task-compiler/before-run-hook)
  (ros::ros-info "call before-run hook")
  (task-compiler/before-run-hook *sm*))

(send *sm* :reset-state)
(exec-smach-with-spin *sm* nil :hz 1.0)

(when (fboundp 'task-compiler/after-run-hook)
  (ros::ros-info "call after-run hook")
  (task-compiler/after-run-hook *sm*))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
