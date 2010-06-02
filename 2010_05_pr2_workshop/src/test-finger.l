;(unix::putenv "ROS_MASTER_URI=http://pr1012:11311")
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "pr2_msgs")
(roseus-add-actionlibs "pr2_controllers_msgs")

(roseus "test_grip_control")

(setq *gripper-client* (instance ros::simple-action-client :init
                                 "r_gripper_controller/gripper_action"
                                 ;;"l_gripper_controller/gripper_action"
                                 pr2_controllers_msgs::Pr2GripperCommandAction))
(send *gripper-client* :wait-for-server)

(defun move-gripper (pos &key (effort -1) (wait t))
  (let* ((goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
         (c (send (send goal :goal) :command)))
    (send c :position pos)
    (send c :max_effort -1)
    (send *gripper-client* :send-goal goal)
    (ros::spin-once)
    )
  (if wait (send *gripper-client* :wait-for-result))
  )

;;
;; for reading fingertip pressure
;;
(defun pressure-cb (msg)
  (if (boundp '*p*) (setq *pre-p* *p*))
  (setq *p* (cons (send msg :l_finger_tip) (send msg :r_finger_tip)) )
  ;;(pprint *p*)
  (when (and (boundp '*pre-p*) *pre-p*)
    (setq *diff-p* (cons (v- (car *p*) (car *pre-p*))
                         (v- (cdr *p*) (cdr *pre-p*))))
    ;;(pprint *diff-p*)
    )
  )
(setq *pressure-sub* (ros::subscribe "pressure/r_gripper_motor"
                                     pr2_msgs::PressureState #'pressure-cb))
