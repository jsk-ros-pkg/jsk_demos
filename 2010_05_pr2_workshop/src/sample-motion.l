;; export ROS_IP=10.68.0.168
;; export ROS_HOSTNAME=10.68.0.168
;; export ROS_MASTER_URI=http://pr1012:11311
(if (not (and (unix:getenv "ROS_IP")
              (unix:getenv "ROS_HOSTNAME")))
    (error "YOU HAVE TO SPECIFY $ROS_IP AND $ROS_HOSTNAME"))
(load (format nil "~A/src/eus-pddl.l"
              (read-line (piped-fork "rospack find pddl_planner"))))
(roseus "pr2_jsk_sample")

(load (format nil "~A/src/pr2.l"
              (read-line (piped-fork
                          "rospack find 2010_05_pr2_workshop"))))

(objects (list *pr2*))
(ros::spin-once)
(update-pr2-joint-angles *pr2* *pr2-state*)
(send *pr2* :inverse-kinematics :rarm-target (make-coords :pos (float-vector 400 -800 1200 )) :rotation-axis nil)
(send-pr2-joint-angles *pr2*)
