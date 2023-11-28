#!/usr/bin/env roseus

(load "lib/llib/unittest.l")
(load "package://elevator_move_base_pr2/src/state-machine-main.l")


(ros::roseus "test_number_recognition")

(init-unit-test)

(deftest test-number-recognition ()
  (let ((msg nil) (topic "light_button"))
    (call-service "/wide_stereo/left/pass_through_image_rect_color/request"
                  (instance std_srvs::EmptyRequest :init))
    (ros::subscribe "light_button" std_msgs::float32 #'(lambda (m) (setq msg m)))
    (ros::rate 10)
    (while (ros::ok)
      (ros::spin-once)
      (when msg (return))
      (ros::sleep))
    (assert msg (format nil "Could not subscribe topic: ~A." topic))))

(run-all-tests)

(exit)
