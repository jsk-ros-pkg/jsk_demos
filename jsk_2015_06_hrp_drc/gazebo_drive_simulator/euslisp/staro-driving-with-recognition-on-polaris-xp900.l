#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/staro-driving-controller-on-polaris-xp900.l")
(load "package://gazebo_drive_simulator/euslisp/staro-driving-with-recognition.l")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-msgs "std_msgs")

;; for demo

(defclass staro-recognition-controller-bridge-on-polaris-xp900
  :super staro-recognition-controller-bridge
  :slots ()
  )

(defmethod staro-recognition-controller-bridge-on-polaris-xp900
  (:init
    (&key (controller (instance staro-driving-controller-on-polaris-xp900 :init
                               :real t :roseus-init nil))
          (rate 10) ((:cmd-rate tmp-cmd-rate) 1.0) (roseus-init t))
    (when roseus-init
      (ros::roseus "staro_recognition_driving_demo_on_polaris_xp900")
      )
    (send-super :init :controller contorller :rate rate
                :cmd-rate tmp-cmd-rate :roseus-init nil)
    )
  )

(defun demo-main
  (&key (real t) (rate 1.0))
  (setq *bridge* (instance staro-recognition-controller-bridge-on-polaris-xp900 :init))
  (send *bridge* :execute rate)
  )
  
  
