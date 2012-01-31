#!/usr/bin/env roseus

(load "unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test-button-light")

(load "package://elevator_move_base_pr2/src/elevator-move-base-main.l")

(init-unit-test)

(deftest test-dummy ()
  (unix::sleep 15) ;; sleep for making video
  (assert t))

(run-all-tests)

(exit)
