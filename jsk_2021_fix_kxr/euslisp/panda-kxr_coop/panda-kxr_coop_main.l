#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "repair-robot")

(ros::advertise "/panda_order" std_msgs::String 1)

(defvar *timeout-servo-list*)
(defvar *reparing-servo-num*)
(defvar *panda_order*)


(setq *panda_order* (instance std_msgs::String :init))

(defun make-timeout-servo-list ()
  (setq *timeout-servo-list* '())
  (setq count (- (length (send joint_states :name)) 1))
  (while (> count -1)
    (if (< (elt (send joint_states :effort) count) 0.1)
	(progn
	  (setq *timeout-servo-list* (cons (elt (send joint_states :name) count) *timeout-servo-list*))
	  (setq *reparing-servo-num* count)))
    (setq count (- count 1))))

(dual_panda-init)

(objects (list *robot*))

(defun reset-pose-high()
  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (send *robot* :reset-pose)
  (send *robot* :rarm :move-end-pos #f(-500 0 0))
  (send *robot* :larm :move-end-pos #f(-500 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

;setting
(reset-pose-high)

(setq joint_states (one-shot-subscribe "/puppet_joint_states" sensor_msgs::jointstate))

(make-timeout-servo-list)

(cond ((string-equal (subseq (elt *timeout-servo-list* 0) 0 4) "rarm")
       (progn
	 (reset-pose-high)
	 (send *panda_order* :data "RARM-ELBOW-P")
	 (ros::publish "/panda_order" *panda_order*)
	 (ros::spin-once)

	 (unix:sleep 3)
	 
	 (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -300.5 1400) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(467 -90.157 1305) :rpy (float-vector 0.0 1.501 -3.061))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :larm :move-end-pos #f(10 0 0))
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *ri* :start-grasp :larm)
	 (unix:sleep 1)

	 (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -188.5 1400) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -32.157 1300) :rpy (float-vector 2.341 1.552 -2.369))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :rarm :move-end-pos #f(100 0 0))
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *ri* :start-grasp :rarm)
	 (unix:sleep 2)

	 (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1300) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 60.157 1300) :rpy (float-vector 2.341 1.552 -2.369))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *ri* :stop-grasp :larm)
	 (unix:sleep 1)

	 (send *robot* :larm :move-end-pos #f(-100 0 0))
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1301.6) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(432.17 234.157 1400) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 180.8 1340.32) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *ri* :start-grasp :larm)
	 (unix::sleep 1)

	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 195.8 1310.22) :rpy (float-vector -1.533 0.56 1.63))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 210.8 1280.22) :rpy (float-vector -1.533 0.0 1.63))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	 (send *ri* :wait-interpolation)

	 (send *panda_order* :data "rarm-free")
	 (ros::publish "/panda_order" *panda_order*)
	 (unix:sleep 5)
	 
	 ;;wait for insert-cable
	 (print "cable-inserting")
	 (unix:sleep 10)
	 (send *panda_order* :data "ope-finish")
	 (ros::publish "/panda_order" *panda_order*)
	 (print "ope-finish")

	 (setq f 0)
	 (while (= f 0)
	   (print "waiting for joint_states")
	   (setq joint_states (one-shot-subscribe "/puppet_joint_states" sensor_msgs::jointstate))
	   (if (> (elt (send joint_states :effort) *reparing-servo-num*) 0.1)
	       (progn
		 (setq f 1))))

	 ;;check effort

	 (send *ri* :stop-grasp :larm)
	 (unix:sleep 1)

	 (send *robot* :larm :move-end-pos #f(-100 0 0))
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *panda_order* :data "rarm-hold")
	 (ros::publish "/panda_order" *panda_order*)
	 (unix:sleep 7)

	 (send *panda_order* :data "check-rarm")
	 (ros::publish "/panda_order" *panda_order*)
	 (unix:sleep 13)

	 (send *panda_order* :data "ope-success")
	 (ros::publish "/panda_order" *panda_order*)
	 ))
      
      ((string-equal (subseq (elt *timeout-servo-list* 0) 0 4) "larm")
       (progn
	 (reset-pose-high)
	 (send *panda_order* :data "LARM-ELBOW-P")
	 (ros::publish "/panda_order" *panda_order*)
	 (ros::spin-once)

	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(276 -125.5 1320) :rpy (float-vector -2.97 1.545 0.16))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *robot* :larm :move-end-pos #f(25 0 0))
	 (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	 (send *ri* :wait-interpolation)

	 (send *ri* :start-grasp :larm)
	 (unix:sleep 2)

	 (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(443 114.5 1300) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
	 (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
	 (send *ri* :wait-interpolation))))

#|
;rarm用に切り替えた直後
(setq f 0)
(while (< f 100)
  (send *panda_order* :data "tte")
  (ros::publish "/panda_order" *panda_order*)
  (ros::spin-once)
  (setq f (+ 1 f))
  (print "huga"))

(do-until-key
  (send *panda_order* :data "tte")
  (ros::publish "/panda_order" *panda_order*)
  (ros::spin-once)
  (print "huga"))

;rarmを崖に近づける
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -300.5 1400) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(467 -90.157 1310) :rpy (float-vector 0.0 1.501 -3.061))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :move-end-pos #f(10 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix:sleep 1)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -188.5 1400) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -32.157 1300) :rpy (float-vector 2.341 1.552 -2.369))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix:sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1300) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 60.157 1300) :rpy (float-vector 2.341 1.552 -2.369))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)
(unis:sleep 1)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1301.6) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(432.17 224.157 1400) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)


;rotate KXR's rarm by himself

;grasp
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 170.8 1340.32) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix::sleep 1)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1310.22) :rpy (float-vector -1.533 0.56 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1280.22) :rpy (float-vector -1.533 0.0 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;move KXR's rarm by himlelf
|#
