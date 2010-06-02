#!/usr/bin/env runeus
;(unix:putenv "ROS_MASTER_URI=http://pr1012:11311")
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "geometry-util.l")

(roseus-add-actionlibs "move_base_msgs")

(setq pose-table
      (list
       (`desk1 . (make-coords :pos (float-vector 0 0 0) :rpy (0 0 0)))
       (`desk2 . (make-coords :pos (float-vector 5 5 0) :rpy (0 0 0)))
       ))

;;
(defun move (from to)
  (let* ((coords (assoc to pose-table)))
    (if coords
	(let (c goal (pose (coords->geometry_msgs_pose (cdr coords))))
	  (setq c (instance ros::simple-action-client :init
			    "move_base" move_base_msgs::MoveBaseAction))
	  (warning-message 1 "wait-for-server~%")
	  (send c :wait-for-server)
	  (setq goal (instance move_base_msgs::MoveBaseActionGoal :init))
	  (send goal :header :frame_id "/map")
	  (send goal :goal :target_pose :header :frame_id "/map")
	  (send goal :goal :target_pose :pose pose)
	  (send c :send-goal goal)
	  (send c :wait-for-result)
	  )
      )))
