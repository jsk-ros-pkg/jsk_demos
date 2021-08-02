#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::roseus "pr2_realtime_ik")

(pr2-init)
(setq *robot* *pr2*)
(setq larm-tgt (make-coords :pos #f(500 300 1000)))
(setq rarm-tgt (make-coords :pos #f(500 -300 1000)))
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
	 )
  (:larm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f( 400  -200  500))
	    (setq ulim #f(1500  1500 1500))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq larm-tgt (make-coords :pos safe-pos :rot (send raw-coords :rot))))
  (:rarm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f( 400 -1500  500))
	    (setq ulim #f(1500   200 1500))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq rarm-tgt (make-coords :pos safe-pos :rot (send raw-coords :rot))))
  (:head-cb (msg)
	    (setq head-tgt (ros::tf-pose->coords (send msg :pose))))
)


(setq m (instance realtime-ik-class :init))

(defun run ()
  (send *robot* :reset-pose)
  (send *robot* :head :neck-p :joint-angle 60)
  (send *robot* :torso_lift_joint :joint-angle 200)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (ros::rate 5)
  (while (ros::ok)
    (send *robot* :inverse-kinematics (list larm-tgt rarm-tgt)
	  :move-target (list (send *robot* :larm :end-coords) (send *robot* :rarm :end-coords))
	  :translation-axis (list t t)
	  :rotation-axis (list t t)
	  :use-torso nil
	  :stop 2
	  :revert-if-fail nil :debug-view nil
	  )
    (send *robot* :head :neck-p :joint-angle (rad2deg (elt (car (rpy-angle (send head-tgt :rot))) 1)))
    (send *robot* :head :neck-y :joint-angle (rad2deg (elt (car (rpy-angle (send head-tgt :rot))) 0)))

    (send *ri* :angle-vector (send *robot* :angle-vector) 100)
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (tgt (list larm-tgt rarm-tgt)) (send tgt :draw-on :flush t))
    ;; (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(run)


