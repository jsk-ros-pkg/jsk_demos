#!/usr/bin/env roseus

(load "unittest.l")
(ros::roseus-add-msgs "posedetection_msgs")

(ros::roseus "test-panel-pose-detection")
(defvar *tfl* (instance ros::transform-listener :init))
(setq *msg* nil)

(init-unit-test)

(defun object-detection-cb (msg)
  (ros::ros-info "*msg*: ~A" (setq *msg* msg)))

(deftest test-call-panel ()
  (ros::subscribe "/narrow_stereo/left/ObjectDetection"
                  posedetection_msgs::ObjectDetection #'object-detection-cb)
  (setq is-ok nil)
  (setq *distance* -1)
  (ros::rate 1)
  (dotimes (cnt 30) ;; 30sec in real
    (when *msg*
      (let (co tf)
        (setq co1 (ros::tf-pose->coords
                   (send (car (send *msg* :objects)) :pose)))
        (setq co2 (send *tfl* :lookup-transform
                        (send *msg* :header :frame_id)
                        (send (car (send *msg* :objects)) :type)
                        (send *msg* :header :stamp)))
        (when (and co1 co2)
          (ros::ros-info "co1: ~A" co1)
          (ros::ros-info "co2: ~A" co2)
          (ros::ros-info "dist: ~A" (distance (send co1 :pos) (send co2 :pos)))
          (setq *distance* (distance (send co1 :pos) (send co2 :pos)))
          (when (> 50 (distance (send co1 :pos) (send co2 :pos)))
            (return)
          ))
      (setq *msg* nil)))
    (ros::spin-once)
    (ros::sleep))
  (assert *msg* "/narrow_stereo/left/ObjectDetection not received")
  (assert (> *distance* 0)
          (format nil "looking transform not found"))
  (assert (< *distance* 50)
          (format nil
                  "ObjectDetection and TF(in bag) are close: ~A" *distance*))
  (unix::sleep 15) ;; for making video
  )

(run-all-tests)

(exit)
