(ros::roseus-add-msgs "rosgraph_msgs")

(defun robot-file (name)
  (let ((lower-name (string-downcase name)))
    (cond
     ((string= lower-name "pr2")
      ("pr2" "package://pr2eus/pr2-interface.l"))
     ((string= lower-name "hrp2jsk")
      "hrp2jsk" "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
     ((string= lower-name "hrp2jsknt")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
     ((string= lower-name "hrp2jsknts")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
     ((string= lower-name "staro")
      "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
     ((string= lower-name "urataleg")
      "package://hrpsys_ros_bridge_tutorials/euslisp/urataleg-interface.l")
     ((string= lower-name "samplerobot")
      "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
     ((string= lower-name "hrp2w")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
     (t (error "uknown robot ~A" name))
     )))


(defun model2real (&key (time 5000) (wait? t) (mode :real))
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  (when wait? (send *ri* :wait-interpolation)))

(defun hand-model2real (&key (time 1000) (wait? t) (mode :real))
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) time)
  (when wait? (send *ri* :hand-wait-interpolation)))

