#!/usr/bin/env roseus
;; define detection parameters befor load detection_interface.l
(defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")

;; load functions
(load "package://jsk_perception/euslisp/detection_interface.l")

;; stret program
;;(load "package://euslisp/jskeus/eus/models/room610-chair-object.l")

;;set variable to use for judging if grasp success or not
(defparameter *grasp-success-threshold* 10)

;;loop counter threshold for start-detect-chair
(defparameter *detect-chair-loop-threshold* 0)
;;


;;if you call this function, (as like (start-detect-chair))
;;*chair* will be update and move to where it would be
(defun start-detect-table (&key (debug nil))
  (let ((loop t) (loop-counter 0) (chair-coords nil) (ret))
    (ros::roseus "objectdetection_publisher")
    (defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")
    (ros::rate 10)
    (ros::ros-info "detecting_now")
    (do-until-key
      (setq ret (check-detection :type "table-center" ;; work for any object
                                 :speak-name "table object"
                                 :target-object *table*
                                 :speak nil))
      (ros::spin-once)
      (if ret
          (progn
            (ros::ros-info "table- pos info ~A before trans" *table*)
            (send *table* :translate (send *pr2* :worldpos) :world)
            (ros::ros-info "table- pos info ~A after trans" *table*)
            (send *pr2* :head :look-at (send *table* :worldpos))
            (send *ri* :angle-vector (send *pr2* :angle-vector))
            (push (send *table* :worldcoords) chair-coords)
            ))


      (inc loop-counter)
      (ros::ros-info "~d loop counter ----" loop-counter)
      (ros::sleep)
      )
    )
  )


