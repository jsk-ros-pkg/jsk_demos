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

(defun send-sample-pose (&optional (duration 3))
  (update-pr2-joint-angles *pr2* *pr2-state*)
  (send *pr2* :inverse-kinematics :rarm-target
        (make-coords :pos (float-vector 400 -800 1200 )) :rotation-axis nil)
  (send-pr2-joint-angles *pr2* duration))

(defun send-reset-pose (&optional (duration 3))
  (send *pr2* :reset-pose)
  (send-pr2-joint-angles *pr2* duration))

(defun hold-cup (&optional (duration 3))
  (send *pr2* :reset-pose)
  (send *pr2* :inverse-kinematics
        :larm-target (make-coords :pos #f(500 300 800))
        :rotation-axis :z)
  (send-pr2-joint-angles *pr2* duration))


(defun hold-cap (&optional (duration 3))
  ;;(update-pr2-joint-angles *pr2* *pr2-state*)
  (send *pr2* :inverse-kinematics
        :rarm-target
        (send (send (send *pr2* :larm :end-coords) :copy-worldcoords)
              :transform (make-coords :pos (float-vector 0 0 150)
                                      :rpy (float-vector 0 pi/2 pi)))
        :larm-target :relative
        :rotation-axis :x)
  (send-pr2-joint-angles *pr2* duration))

(warn ";; send-sample-pose~%")
