#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/hrp2jsk-driving-controller-on-polaris-xp900.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsknt-driving-motion-on-polaris-xp900.l")

(defclass hrp2jsknt-driving-controller-on-polaris-xp900
  :super hrp2jsk-driving-controller-on-polaris-xp900
  )

(defmethod hrp2jsknt-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "hrp2jsknt_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance hrp2jsknt-driving-motion-on-polaris-xp900 :init :real nil
                                       :grasp-arm :rarm :grasp-point :handle-crank)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   )
  )

(defun hrp2jsknt-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "hrp2jsknt" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance hrp2jsknt-driving-controller-on-polaris-xp900 :init :real nil))
