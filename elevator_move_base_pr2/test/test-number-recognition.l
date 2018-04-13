#!/usr/bin/env roseus

(load "lib/llib/unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test-number-recognition")

(load "package://elevator_move_base_pr2/src/elevator-move-base-main.l")

(init-unit-test)

(deftest test-dummy ()
  (assert t))

(run-all-tests)

(exit)
