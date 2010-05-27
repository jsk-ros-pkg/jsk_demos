#!/usr/bin/env runeus

(if (not (and (unix:getenv "ROS_IP")
              (unix:getenv "ROS_HOSTNAME")))
    (error "YOU HAVE TO SPECIFY $ROS_IP AND $ROS_HOSTNAME"))

;; mostly same to demo-eus.l in pddl package under jsk subversion trunk
(load (format nil "~A/src/eus-pddl.l"
              (read-line (piped-fork "rospack find pddl_planner"))))
(load (format nil "~A/src/problem.l"
              (read-line (piped-fork
                          "rospack find 2010_05_pr2_workshop"))))

(roseus "pr2_jsk_main")
(load (format nil "~A/src/pr2.l"
              (read-line (piped-fork
                          "rospack find 2010_05_pr2_workshop"))))
;; make an interface to pddl_planner

(defun pddl-solve ()
  (setq *pddl-client* (instance ros::simple-action-client :init
                                "pddl_planner" pddl_msgs::PDDLPlannerAction))
  (warning-message 1 "waiting pddl_planner~%")
  (send *pddl-client* :wait-for-server)
  (let ((goal (instance pddl_msgs::PDDLPlannerActionGoal :init)))
    (send goal :goal :domain (send *domain* :ros-message))
    (send goal :goal :problem (send *problem* :ros-message))
    (send *pddl-client* :send-goal goal))
  
  (warning-message 1 "waiting goal...~%")
  (send *pddl-client* :wait-for-result)
  (let ((result (mapcar #'(lambda (x)
                            (cons (intern (send x :action))
                                  (mapcar #'intern (send x :args))))
                        (send (send *pddl-client* :get-result) :sequence))))
    (setq *pddl-result* result)
    (format t "planning result~%")
    (format t "-------------------------------------~%")
    (pprint result)
    (format t "-------------------------------------~%")
    result))

;;((pick rarm pet desk1)
;; (pick larm cup desk2)
;; (pour pet cup rarm larm)
;; (put larm cup desk)
;; (put rarm pet desk))

(pddl-solve)
;;(ros::spin)
