#!/usr/bin/env roseus

(load "unittest.l")
(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "test-button-light")

(init-unit-test)

(deftest test-button-light ()
  (setq *m* nil count 0)
  (ros::subscribe "light_button" std_msgs::float32 #'(lambda(m)(setq *m* m)))
  (unix::sleep 1)
  (while (and (< count 100) (not (and *m* (< 1.0 (send *m* :data)))))
    (ros::spin-once)
    (unix::usleep (* 100 1000)))
  ;;(unix::sleep 15) ;; sleep for making video
  (assert *m*
	  "detector node returns result")
  (assert (and *m* (< 1.0 (send *m* :data)))
	  "button light has detected successfully")
  )

(run-all-tests)

(exit)
