#!/usr/bin/env roseus

(load "lib/llib/unittest.l")
(load "package://elevator_move_base_pr2/src/state-machine-main.l")


(ros::roseus "test_number_recognition")

(init-unit-test)

(deftest test-check-target-floor ()
  (setq userdata '())
  (set-alist 'target-floor "7F" userdata)
  (assert (check-target-floor-arrived userdata)
          "Failed to check if arrived at target-floor (7F)."))

(run-all-tests)

(exit)
