#!/usr/bin/env roseus

(load "lib/llib/unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test-button-light")

(init-unit-test)

(defun button-callback (msg)
  (ros::ros-info "button callback received: ~A" msg)
  (setq *m* msg))

(deftest test-button-light ()
  (setq *m* nil count 0)
  (ros::subscribe "light_button" std_msgs::float32 #'button-callback)
  (ros::rate 1)
  (while (and (ros::ok) (or (null *m*) (>= 1.0 (send *m* :data))))
    (ros::spin-once)
    (ros::sleep))

  (assert *m*
	  "detector node returns result")
  (assert (and *m* (< 1.0 (send *m* :data)))
	  "button light has detected successfully")
  )

(run-all-tests)

(exit)
