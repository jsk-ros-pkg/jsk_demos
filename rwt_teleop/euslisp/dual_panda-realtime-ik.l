#!/usr/bin/env roseus
(require :dual_panda-interface "package://franka_eus/euslisp/dual_panda-interface.l")

(ros::roseus "dual_panda_realtime_ik")

(dual_panda-init)

;; (setq larm-ee-mod (make-cascoords :pos #f(0 0 0) :rpy #f(3.14 1.57 0)))
;; (send (send *dual_panda* :larm :end-coords :parent) :assoc larm-ee-mod)
;; send (send *dual_panda* :larm :end-coords) :assoc (make-cascoords)
(setq *robot* *dual_panda*)
(setq larm-tgt (make-coords :pos #f(500  300 1000) :rpy #f(3.14 1.57 0)))
(setq rarm-tgt (make-coords :pos #f(500 -300 1000) :rpy #f(3.14 1.57 0)))
(setq head-tgt (make-coords))

(objects (list *robot* larm-tgt rarm-tgt))

(defclass realtime-ik-class
  :super propertied-object
  :slots ())
(defmethod realtime-ik-class
  (:init ()
	 (ros::subscribe "master_larm_pose" geometry_msgs::PoseStamped #'send self :larm-cb)
	 (ros::subscribe "master_rarm_pose" geometry_msgs::PoseStamped #'send self :rarm-cb)
	 (ros::subscribe "master_head_pose" geometry_msgs::PoseStamped #'send self :head-cb)
	 (defun set-reset-pose (req)
	   (let ((m (send req :response)))
	     (setq larm-tgt (make-coords :pos #f(500  300 1000) :rpy #f(3.14 1.57 0)))
	     (setq rarm-tgt (make-coords :pos #f(500 -300 1000) :rpy #f(3.14 1.57 0)))
	     (setq head-tgt (make-coords))
	     (send *ri* :angle-vector (send *robot* :reset-pose) 5000)
	     (send *ri* :wait-interpolation)
	     m)
	   )
	 (ros::advertise-service "set_reset_pose" std_srvs::Empty #'set-reset-pose)
	 )
  (:larm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f(100 -200  600))
	    (setq ulim #f(600  500 1200))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq larm-tgt (make-coords :pos safe-pos :rot (send raw-coords :rot))))
  (:rarm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f(100 -500  600))
	    (setq ulim #f(600  200 1200))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq rarm-tgt (make-coords :pos safe-pos :rot (send raw-coords :rot))))
  (:head-cb (msg)
	    (setq head-tgt (ros::tf-pose->coords (send msg :pose))))
)


(setq m (instance realtime-ik-class :init))

(defun run ()
  (send *ri* :set-all-joint-pd-gain 30 3)
  (send *robot* :reset-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (ros::rate 5)
  (while (ros::ok)
    (when (send *ri* :check-error)
      (send *ri* :recover-error))
    (send *robot* :inverse-kinematics (list larm-tgt rarm-tgt)
  	  :move-target (list (send *robot* :larm :end-coords) (send *robot* :rarm :end-coords))
  	  :translation-axis (list t t)
  	  :rotation-axis (list t t)
  	  :use-torso nil
  	  :stop 2
  	  :revert-if-fail nil :debug-view nil
  	  )

    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (tgt (list larm-tgt rarm-tgt)) (send tgt :draw-on :flush t))
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(run)
