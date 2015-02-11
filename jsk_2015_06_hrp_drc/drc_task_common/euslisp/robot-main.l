#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus "robot_main")
(setq *tf* (instance ros::transform-listener :init))
(require :robot-interface "package://pr2eus/robot-interface.l")
(init-robot-from-name "HRP2JSKNTS")

(defun go-pos-execute-cb (server goal)
  (let ((msg (send server :result))
        (goal-pose-stamped (send goal :goal :target_pose)))
    ;; resolve tf
    (let ((trans (send *tf* :lookup-transform "BODY" 
                       (send goal-pose-stamped :header :frame_id)
                       (send goal-pose-stamped :header :stamp))))
      (send trans :transform
            (ros::tf-pose-stamped->coords goal-pose-stamped))
      (let ((x (* 0.001 (elt (send trans :worldpos) 0)))
            (y (* 0.001 (elt (send trans :worldpos) 1)))
            (theta (rad2deg (elt (car (send trans :rpy-angle)) 0))))
        (send *ri* :go-pos x y theta))
      (send server :set-succeeded msg)
      )))

(setq *go-pos-action-server*
      (instance ros::simple-action-server :init
                "/go_pos" move_base_msgs::MoveBaseAction
                :execute-cb #'go-pos-execute-cb))


(ros::rate 10)
(do-until-key
 (send *go-pos-action-server* :worker)
 (ros::spin-once)
 (ros::sleep))
