(ros::roseus-add-msgs "rosgraph_msgs")

(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))

(setq *robot-model-files*
      (list "package://pr2eus/pr2-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/urataleg-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l"
            "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l"))

(dolist (f *robot-model-files*)
  (when (probe-file (ros::resolve-ros-path f))
    (ros::ros-info "loading ~A" f)
    (load f)))

(defun coords-to-float-vector (coords)
  (float-vector (elt (send coords :worldpos) 0)
                (elt (send coords :worldpos) 1)
                (elt (send coords :worldpos) 2)
                (elt (car (send coords :rpy-angle)) 0)
                (elt (car (send coords :rpy-angle)) 1)
                (elt (car (send coords :rpy-angle)) 2)
                0 0))

(defun float-vector-to-coords (fv)
  (make-coords :pos (subseq fv 0 3)
               :rpy (subseq fv 3 6)))

