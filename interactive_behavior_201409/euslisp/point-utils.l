#!/usr/bin/env roseus
;; point-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :point-utils)
(ros::load-ros-manifest "jsk_recognition_msgs")
(defparameter *pointed-bbox-topic* "/pointed_object/boxes")
(defvar *tfl* (instance ros::transform-listener :init))
(defvar *pointed-object* nil "pointed object position")
(defvar *base-frame-id* "base_footprint")

(defun pointed-bbox-cb (msg)
  (unless (send msg :boxes) (return-from pointed-bbox-cb nil))
  (ros::ros-info "found ~A objects" (length (send msg :boxes)))
  (let ((box (car (send msg :boxes)))
        cds)
    (setq cds (if (string= (send box :header :frame_id) *base-frame-id*)
                  (ros::tf-pose->coords (send box :pose))
                (sned *tfl* :transform-pose *base-frame-id*
                      (instance geometry_msgs::PoseStamped :init
                                :header (send box :header)
                                :pose (send box :pose)))))
    (unless cds (return-from pointed-bbox-cb nil))
    (send cds :name *base-frame-id*)
    (setq *pointed-object* cds)))


(defun get-pointed-object ()
  (ros::subscribe *pointed-bbox-topic*
                  jsk_recognition_msgs::BoundingBoxArray
                  #'pointed-bbox-cb)
  (unwind-protect
      (progn
        (setq *pointed-object* nil)
        (ros::rate 1)
        (dotimes (i 10)
          (when *pointed-object*
            (return-from get-pointed-object *pointed-object*))
          (ros::spin-once)
          (ros::sleep)))
    (ros::unsubscribe *pointed-bbox-topic*)))
