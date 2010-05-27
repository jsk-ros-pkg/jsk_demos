#!/usr/bin/env runeus
;; mostly same to demo-eus.l in pddl package under jsk subversion trunk
(load (format nil "~A/src/eus-pddl.l"
              (read-line (piped-fork "rospack find pddl_planner"))))
(load (format nil "~A/src/problem.l"
              (read-line (piped-fork
                          "rospack find pr2_workshop_jsk_2010_main"))))

(roseus "pr2_jsk_main")
(load (format nil "~A/src/pr2.l"
              (read-line (piped-fork
                          "rospack find pr2_workshop_jsk_2010_main"))))
;; make an interface to pddl_planner

(defun pddl-solve ()
  (setq *pddl-client* (instance ros::simple-action-client :init
                                "pddl_planner" pddl_msgs::PDDLPlannerAction))
  (warning-message 1 "waiting pddl_planner~%")
  (send *pddl-client* :wait-for-server)
  (let ((goal (instance pddl_msgs::PDDLPlannerActionGoal :init)))
    (send goal :goal :domain (send *domain* :ros-message))
    (send goal :goal :problem (send *problem* :ros-message))
    (send *pddl-client* :send-goal goal)
    (warning-message 1 "sent goal~%")
    )
  
  (warning-message 1 "waiting goal...~%")
  (send *pddl-client* :wait-for-result)
  (let ((result (mapcar #'(lambda (x)
                            (cons (intern (send x :action))
                                  (mapcar #'intern (send x :args))))
                        (send (send *pddl-client* :get-result) :sequence))))
    (setq *pddl-result* result)
    (pprint result)
    result))
;;((pick rarm pet desk)
;; (pick larm cup desk)
;; (pour pet cup rarm larm)
;; (put larm cup desk)
;; (put rarm pet desk))

;;(pddl-solve)

;;(ros::spin)
