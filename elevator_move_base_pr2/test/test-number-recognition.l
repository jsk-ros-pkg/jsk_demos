#!/usr/bin/env roseus

(load "lib/llib/unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test_number_recognition")

(load "package://elevator_move_base_pr2/src/elevator-move-base-main.l")

(init-unit-test)

(deftest test-check-target-floor ()
  (setq userdata '())
  (set-alist 'target-floor "7F" userdata)
  (assert (check-target-floor userdata)
          "check-target-floor 7F"))

(run-all-tests)

(exit)
