#!/usr/bin/env roseus

(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://semi2024/euslisp/semi-kxr.l")
(load "models/arrow-object.l")

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "visualize_hand_pose")

(kxr-init)

(setq *camera-color-optical-frame* (arrow))
(send *camera-color-optical-frame* :newcoords
      (send (send (send (send (send *robot* :HEAD_LINK1_lk) :copy-worldcoords) :translate #f(0 10 50)) :rotate pi/2 :y :local) :rotate -pi/2 :z :local))
(send (send *robot* :HEAD_LINK1_lk) :assoc *camera-color-optical-frame*)
(objects (list *robot* *camera-color-optical-frame*))


(defun callback (msg)
  (let (skeleton bone index-pos i-x i-y i-z distance dis-10 x1)
	  (setq skeleton (car (send msg :skeletons)))
	  (when skeleton
	    (progn
	      (setq bone (car (send skeleton :bones)))
	      (when bone
		      (setq index-pos (send bone :end_point))
		      (setq i-x (send index-pos :x))
		      (setq i-y (send index-pos :y))
		      (setq i-z (send index-pos :z))
		      (when (< 0 i-z)
		        (setq dis-10 (* 10 (sqrt (+ (+ (* i-x i-x) (* i-y i-y)) (* i-z i-z)))))
		        (setq x1 (floor dis-10))
		        (setq distance (* 10 x1))
		        (ros::ros-info (format nil "=============================="))
		        (ros::ros-info (format nil "index finger detected!!"))
		        (ros::ros-info (format nil "(~A, ~A, ~A)" i-x i-y i-z))
		        (ros::ros-info (format nil "distance : ~A" distance))
		        ;; (speak-jp (format nil "~A" distance) :wait t)
		        ;; (speak-jp "センチメートルです" :wait t)
					  (when (< distance 30)
		          (send self :move-rarm (float-vector i-x i-y i-z)))))))))

(defun move-rarm (pos-from-head)
  (setq pos-from-head (float-vector
			                 (* 1000.0 (elt pos-from-head 0))
			                 (* 1000.0 (elt pos-from-head 1))
			                 (* 1000.0 (elt pos-from-head 2))))  ;; meterをmillimeterに変換
  (let (target-coords)
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq target-coords (send (send *camera-color-optical-frame* :copy-worldcoords) :transform (make-coords :pos pos-from-head)))
    (send *irtviewer* :draw-objects)
    (when  ;; ik solved
	      (send *robot* :inverse-kinematics-loop-for-look-at :head
              (send target-coords :worldpos) :move-target *camera-color-optical-frame*)
	    (progn
	      (send *irtviewer* :draw-objects)
	      (ros::ros-warn "ik solved")
	      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
	      )
      )
    )
  )

(ros::subscribe "/camera/hand_skeleton_with_depth/output/pose" jsk_recognition_msgs::HumanSkeletonArray #'callback 1)

(send *ri* :servo-on)

(ros::rate 10)
(do-until-key ()
  (ros::spin-once)
  (ros::sleep)
  (x::window-main-one))
