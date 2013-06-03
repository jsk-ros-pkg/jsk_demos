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

  (warn "610->base: ~A~%" (send *tfl* :lookup-transform *room610-origin* *base-frame-id* (ros::time 0)))
  (warn "*pr2*: ~A~%" *pr2*)
  (warn "*laundry*: ~A~%" *laundry*)
  (warn "*broom*: ~A~%" *broom*)

  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

  (ros::roseus "object_detection_marker_laundry")
  (check-detection :type nil ;; work for any object
                       :speak-name "sentakuki"
                       :target-object *target-object*
                       :timeout 30
                       :diff-position 10
                       :diff-rotation (deg2rad 10)
                       :speak nil)

#|  (setq temp-c (send *pr2* :copy-worldcoords))
  (send *pr2* :reset-coords)
  (send *pr2* :assoc *target-object*)
  (send *pr2* :transform temp-c :world)
  (send *pr2* :dissoc *target-object*)
|#

  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

  (send *target-object* :transform *pr2*)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

  ;; debug
  (warn "610->base: ~A~%" (send *tfl* :lookup-transform *room610-origin* *base-frame-id* (ros::time 0)))
  (warn "*pr2*: ~A~%" *pr2*)
  (warn "*laundry*: ~A~%" *laundry*)
  (warn "*broom*: ~A~%" *broom*)

)
