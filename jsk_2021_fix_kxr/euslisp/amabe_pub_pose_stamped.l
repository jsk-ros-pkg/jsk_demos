#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pose-stamped-publisher")

(dual_panda-init)

(defvar *joints* nil)
#|
(setq buf #f(0.0 15.0 0.0 -135.0 0.0 150.0 45.0 7.0 8.0 0.0 15.0 0.0 -135.0 0.0 150.0 45.0 14.0 15.0))
(setq buf (coerce buf cons))
(setq l (- (length buf) 1))
(setq i l)
(setq ans nil)
(while (<= 0 i)
  (if (and (not (= i 7)) (not (= i 8)) (not (= i 16)) (not (= i 17)))
      (if (not ans)
	  (setq ans (cons (elt buf i) nil))
	  (setq ans (cons (elt buf i) ans))))
  (setq i (- i 1)))
|#

(defun joints-cb (msg)
  (setq *joints* (send msg :position))
  (coerce *joints* cons)
  )

(ros::rate 10)
(ros::subscribe "/dual_panda/joint_states" sensor_msgs::JointState #'joints-cb 1)

(while (ros::ok)
  (let ((pose-stamped-rarm (instance geometry_msgs::PoseStamped :init)))
    ;;(send joint-states (coerce (mapcar #'(lambda (x) (rad2deg x)) (coerce *joints* cons)) float-vector))
    (setq joint-states (coerce (mapcar #'(lambda (x) (rad2deg x)) (coerce *joints* cons)) float-vector))
    (send *robot* :angle-vector joint-states)
    (setq pose-stamped-rarm (ros::coords->tf-pose-stamped (send *robot* :rarm :end-coords) base))
    (ros::publish "/pose_stamped_rarm" pose-stamped-rarm)
    (ros::spin-once)
    (ros::sleep)))
#|
(ros::load-ros-manifest "sensor_msgs")
(setq tem (send (one-shot-subscribe "/dual_panda/joint_states" sensor_msgs::JointState) :position))
(mapcar #'(lambda (x) (rad2deg x)) (list pi pi/2 pi))
(coerce #f(1 2 3) cons)
|#
