#!/usr/bin/env roseus

(ros::roseus "teleop_test")
(ros::roseus-add-msgs "geometry_msgs")
(load "package://drc_task_common/euslisp/robot-util.l")
;(setq *robot-name* (string-upcase (unix::getenv "ROBOT")))
(setq *robot-name* "HRP2JSKNT")
(generate-robot *robot-name* :generate-ri? t)
;(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
;(samplerobot-init)
;(setq *robot* *sr*)
(objects *robot*)
(send *robot* :angle-vector (send *ri* :state :reference-vector))
(setq *linear-factor* 4)
(setq *angular-factor* 0.06)
(defun twist-callback (msg)
  (let ((twist (make-coords :pos (scale *linear-factor* (float-vector (send msg :linear :x) (send msg :linear :y) (send msg :linear :z))) :rpy (list (* *angular-factor* (send msg :angular :z)) (* *angular-factor* (send msg :angular :y)) (* *angular-factor* (send msg :angular :x))))))
   ;(send *robot* :angle-vector (send *ri* :state :reference-vector))
    (send *robot* :larm :inverse-kinematics (send (send *robot* :larm :end-coords :copy-worldcoords) :transform twist))
    (send *ri* :angle-vector (send *robot* :angle-vector) 500)
    )
  (objects *robot*)
  )
    


(ros::subscribe "twist" geometry_msgs::Twist #'twist-callback)
(ros::rate 3)
(while t
  (ros::spin-once)
  (ros::sleep)
)
 