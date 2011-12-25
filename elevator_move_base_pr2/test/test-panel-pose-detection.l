#!/usr/bin/env roseus

(load "unittest.l")
(ros::roseus-add-msgs "posedetection_msgs")

(ros::roseus "test-panel-pose-detection")
(defvar *tfl* (instance ros::transform-listener :init))
(setq *msg* nil)

(init-unit-test)

(defun object-detection-cb (msg)
  (setq *msg* msg))

(deftest test-call-panel ()
  (ros::subscribe "/narrow_stereo/left/ObjectDetection"
		  posedetection_msgs::ObjectDetection #'object-detection-cb)
  (setq is-ok nil)
  (dotimes (cnt 300) ;; 30sec in real
    (when *msg*
      (let (co tf)
	(setq co1 (ros::tf-pose->coords
		  (send (car (send *msg* :objects)) :pose)))
	(setq co2 (send *tfl* :lookup-transform
		       (send *msg* :header :frame_id)
		       (send (car (send *msg* :objects)) :type)
		       (send *msg* :header :stamp)))
	(when (and co1 co2)
	  (assert (> 50 (distance (send co1 :pos) (send co2 :pos)))
		  "ObjectDetection and TF(in bag) are close")
	  (setq is-ok t)
	  (return)
	  ))
      (setq *msg* nil))
    (ros::spin-once)
    (unix::usleep (* 100 1000)))
  (assert is-ok
	  "test exit successfully")
  )

(run-all-tests)

(exit)
