#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/hrp2jsk-driving-controller-on-polaris-xp900.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsknts-driving-motion-on-polaris-xp900.l")

(defclass hrp2jsknts-driving-controller-on-polaris-xp900
  :super hrp2jsk-driving-controller-on-polaris-xp900
  )

(defmethod hrp2jsknts-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "hrp2jsknts_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance hrp2jsknts-driving-motion-on-polaris-xp900 :init :real nil
                                       :grasp-arm :rarm :grasp-point :handle-crank)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   )
  )

(defun hrp2jsknts-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "hrp2jsknts" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance hrp2jsknts-driving-controller-on-polaris-xp900 :init :real nil))

