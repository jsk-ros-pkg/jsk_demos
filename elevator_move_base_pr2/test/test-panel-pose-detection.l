#!/usr/bin/env roseus

(ros::roseus-add-msgs "posedetection_msgs")

(require :unittest "lib/llib/unittest.l")


(defvar *msg* nil)
(init-unit-test)


(defun object-detection-cb (msg)
  (ros::ros-info "*msg*: ~A" (setq *msg* msg)))


(deftest test-call-panel ()
  (let ((tfl (instance ros::transform-listener :init))
        (dist -1)
        (max-dist (ros::get-param "~max_distance" 50))
        co1 co2)
    (ros::subscribe "/narrow_stereo/left/ObjectDetection"
                    posedetection_msgs::ObjectDetection #'object-detection-cb)

    (ros::rate 1)
    (dotimes (cnt 30) ;; 30sec in ros time
      (ros::spin-once)
      (when *msg*
        (setq co1 (ros::tf-pose->coords
                   (send (car (send *msg* :objects)) :pose)))
        (setq co2 (send tfl :lookup-transform
                        (send *msg* :header :frame_id)
                        (send (car (send *msg* :objects)) :type)
                        (ros::time 0)))
        (when (and co1 co2)
          (setq dist (distance (send co1 :pos) (send co2 :pos)))
          (ros::ros-info "co1: ~A" co1)
          (ros::ros-info "co2: ~A" co2)
          (ros::ros-info "dist: ~A" dist)
          (when (> max-dist dist)
            (return)))
        (setq *msg* nil))
      (ros::sleep))

    (assert *msg* "Could not receive /narrow_stereo/left/ObjectDetection")
    (assert (> dist 0)
            (format nil "looking transform not found"))
    (assert (< dist max-dist)
            (format nil "ObjectDetection and TF(in bag) are not close: ~A"
                    dist))
  ))


(ros::roseus "test_panel_pose_detection")

(run-all-tests)
(exit)
