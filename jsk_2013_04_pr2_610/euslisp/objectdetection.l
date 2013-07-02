#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_perception")

;; define detection parameters before load detection_interface.l
(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")

;; load functions  
(load "package://jsk_perception/euslisp/detection_interface.l")

(defun laundry-detection (obj)
  (let (ret)
    ;; start program
    (ros::roseus "objectdetection_laundry_publisher")

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    (ros::roseus "object_detection_marker_laundry")
    (setq ret 
          (check-detection :type "laundry_button" ;; work for any object
                           :speak-name "sentakuki"
                           :target-object obj
                           :timeout 30
                           :diff-position 10
                           :diff-rotation (deg2rad 10)
                           :speak nil))

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    (if (boundp '*irtviewer*) (send ret :draw-on :flush t :size 100))
    (if (boundp '*irtviewer*) (send (send *pr2* :copy-worldcoords) :draw-on :flush t :size 2000))
    (send obj :move-to ret :world)
    (send obj :transform *pr2* :world)
    (if (boundp '*irtviewer*) (send (send obj :copy-worldcoords) :draw-on :flush t :size 1500 :color #f(1 0 0)))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    ;; debug
    (ros::ros-info "610->base: ~A" (send *tfl* :lookup-transform *room610-origin* *base-frame-id* (ros::time 0)))
    (ros::ros-info "*pr2*: ~A" *pr2*)
    (ros::ros-info "*laundry*: ~A" *laundry*)
    (ros::ros-info "*broom*: ~A" *broom*)

    (ros::ros-info "found laundry?: ~A" ret)
    
    ret))

(defun tray-detection (obj)
  (let (ret)
    ;; start program
    (ros::roseus "objectdetection_tray_publisher")

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    (ros::roseus "object_detection_marker_tray")
    (setq ret nil)
    (while (not ret)
    (setq ret
          (check-detection :type "tray_center" ;; work for any object
                           :speak-name "とれい"
                           :target-object obj
                           :timeout 30
                           :diff-position 10
                           :diff-rotation (deg2rad 10)
                           :speak t))
    )

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    (if (boundp '*irtviewer*) (send ret :draw-on :flush t :size 100))
    (if (boundp '*irtviewer*) (send (send *pr2* :copy-worldcoords) :draw-on :flush t :size 2000))
    (send obj :move-to ret :world)
    (send obj :transform *pr2* :world)
    (if (boundp '*irtviewer*) (send (send obj :copy-worldcoords) :draw-on :flush t :size 1500 :color #f(1 0 0)))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t))

    ;; debug
    (ros::ros-info "610->base: ~A" (send *tfl* :lookup-transform *room610-origin* *base-frame-id* (ros::time 0)))
    (ros::ros-info "*pr2*: ~A" *pr2*)
    (ros::ros-info "*tray*: ~A" *tray*)
    (ros::ros-info "*broom*: ~A" *broom*)

    (ros::ros-info "found tray?: ~A" ret)
    
    ret))
