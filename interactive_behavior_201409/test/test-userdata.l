#!/usr/bin/env roseus
;; test-userdata.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defaction move-to (from-place to-place userdata)
  (ros::ros-info "move-to ~A ~A ~A" from-place to-place userdata)
  (set-alist :target-place :kitchen userdata)
  (ros::ros-warn "userdata: ~A" userdata)
  t)

(defaction find-object (obj spot &optional userdata)
  (ros::ros-info "find-object ~A ~A ~A" obj spot userdata)
  (ros::ros-error "target-place: ~A" (cdr (assoc :target-place userdata)))
  t)

(defaction pick (obj arm spot &optional userdata)
  (ros::ros-info "pick ~A ~A ~A ~A" obj arm spot userdata)
  t)

(defaction find-placement (obj arm spot &optional userdata)
  (ros::ros-info "(find-placement ~A ~A ~A)" obj arm spot)
  t)

(defaction place (obj arm spot &optional userdata)
  (ros::ros-info "place ~A ~A ~A ~A" obj arm spot userdata)
  t)

(defaction open-door (arm spot &optional userdata)
  (ros::ros-info "open-door ~A ~A ~A" arm spot userdata)
  t)

(defaction close-door (arm spot &optional userdata)
  (ros::ros-info "close-door ~A ~A ~A" arm spot userdata)
  t)

(defaction deliver (obj arm person spot &optional userdata)
  (ros::ros-info "deliver ~A ~A ~A ~A ~A" obj arm person spot userdata)
  t)

(pr2-init)
(sample-run)
