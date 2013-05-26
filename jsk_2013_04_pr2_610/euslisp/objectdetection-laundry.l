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
  (ros::rate 10)
  (let ((diff-pos-norm 1000) 
	(diff-rot-norm 1000)
	(prev-coords (make-coords))
	c)
    (unless (and (< diff-pos-norm 10) (< diff-rot-norm 0.2))
      (send *target-object* :move-to (make-coords) :world)
      (setq c (check-detection :type nil ;; work for any object
			       :speak-name "sentakuki"
			       :target-object *target-object*
			       :speak nil))
      (setq diff-pos-norm (norm (send c :difference-position prev-coords)))
      (setq diff-rot-norm (norm (send c :difference-rotation prev-coords)))
      (setq prev-coords c)
      (ros::spin-once)
      (ros::sleep)
      (warn "diff pos: ~A, rot: ~A~%" diff-pos-norm diff-rot-norm)
      ))
  )