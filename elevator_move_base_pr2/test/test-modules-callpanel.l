#!/usr/bin/env roseus

(load "unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test_modules_callpanel")

(init-unit-test)

(deftest test-modules-callpanel ()
  (assert t))

(run-all-tests)

(exit)
