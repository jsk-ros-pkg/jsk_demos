#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_perception")

;; define detection parameters before load detection_interface.l
;;(defvar *detection-topic* "/ObjectDetection")
;;(defvar *detection-topic* "/openni/rgb/ObjectDetection")
(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")

;; load functions  
(load "package://jsk_perception/euslisp/detection_interface.l")

(defun laundry-detection (obj)
  ;; start program
  (ros::roseus "objectdetection_laundry_publisher")

  (setq *target-object* obj)

  (ros::roseus "object_detection_marker_laundry")
  (check-detection :type nil ;; work for any object
                       :speak-name "sentakuki"
                       :target-object *target-object*
                       :timeout 30
                       :diff-position 10
                       :diff-rotation (deg2rad 10)
                       :speak nil))
