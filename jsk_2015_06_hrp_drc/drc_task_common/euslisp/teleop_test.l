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
(setq *linear-factor* 6)
(setq *angular-factor* 0.06)
(setq *full-body* t)
(defun twist-callback (msg)
  (let ((twist (make-coords :pos (scale *linear-factor* (float-vector (send msg :linear :x) (send msg :linear :y) (send msg :linear :z))) :rpy (list (* *angular-factor* (send msg :angular :z)) (* *angular-factor* (send msg :angular :y)) (* *angular-factor* (send msg :angular :x))))))
   ;(send *robot* :angle-vector (send *ri* :state :reference-vector))
    (let (ik-ret)
      (if *full-body*
	  (let 
	      ((tc (list
		    (send (send *robot* :larm :end-coords :copy-worldcoords) :transform twist)
		    (send *robot* :rarm :end-coords :copy-worldcoords)
		    (send *robot* :rleg :end-coords :copy-worldcoords)
		    (send *robot* :lleg :end-coords :copy-worldcoords))))
	    (with-move-target-link-list
	     (mt ll *robot* (list :larm :rarm :rleg :lleg))
	     (setq ik-ret (fullbody-inverse-kinematics-for-teleop tc mt ll :draw? t :rotation-axis (list t t t t) :translation-axis (list t t t t)))
	    )
	    )
	(setq ik-ret (send *robot* :larm :inverse-kinematics (send (send *robot* :larm :end-coords :copy-worldcoords) :transform twist)))
	)
      (if ik-ret
      	  (send *ri* :angle-vector (send *robot* :angle-vector) 600)
      	)
      )
    (objects *robot*)
    )
  )
    
(defun fullbody-inverse-kinematics-for-teleop
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0))
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis translation-axis
        :rotation-axis rotation-axis
        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
        :stop 100
        :debug-view (if draw? :no-message nil)
        )
  )


(ros::subscribe "twist" geometry_msgs::Twist #'twist-callback)
(ros::rate 2)
(while t
  (ros::spin-once)
  (ros::sleep)
)
 