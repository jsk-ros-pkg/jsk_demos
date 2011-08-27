#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates
      '(
	(AT ?SPOT - spot) ;; the spot which robot is in
	(CLEAN ?OBJ - object)
	(GRASP ?OBJ - object)
	(REACHABLE ?OBJ - object ?SPOT - spot)
	(IWANTPASS ?OBJ - object ?SPOT - spot)
	))
;;making action
(setq *actlist*
      (list
       (instance pddl-action :init
		 :name "move"
		 :parameters '((?TO spot))
		 :precondition '((NOT (AT ?TO)))
		 :effect '((AT ?TO)
			   (forall (?from - spot)
				   (NOT (AT ?from)))))
       (instance pddl-action :init
		 :name "pick"
		 :parameters '((?OBJ object) (?SPOT spot))
		 :precondition '((REACHABLE ?OBJ ?SPOT)
				 (AT ?SPOT)
				 (forall (?objx - object)
					 (NOT (GRASP ?objx))))
		 :effect '((GRASP ?OBJ)
			   (NOT (REACHABLE ?OBJ ?SPOT))
			   ))
       (instance pddl-action :init
		 :name "put"
		 :parameters '((?OBJ object)(?SPOT spot))
		 :precondition '((AT ?SPOT)
				 (GRASP ?OBJ))
		 :effect '((NOT (GRASP ?OBJ))
			   (REACHABLE ?OBJ ?SPOT)))
       (instance pddl-action :init
		 :name "pass"
		 :parameters '((?OBJ object)(?SPOT spot))
		 :precondition '((IWANTPASS ?OBJ ?spot)
				 (AT ?spot)
				 (GRASP ?OBJ))
		 :effect '((NOT (IWANTPASS ?OBJ ?SPOT))
			   (NOT (GRASP ?OBJ))))
#|       (instance pddl-action :init
		 :name "wipe"
		 :parameters '((?OBJ object))
		 :precondition '((exists (?spot - spot)
					 (AND (REACHABLE ?OBJ ?spot)
					      (AT ?spot)))
				 (forall (?gomi - object)
					 (NOT (ON ?gomi ?OBJ)))
				 (NOT (CLEAN ?OBJ)))
		 :effect '((CLEAN ?OBJ)))|#
       ))

;;add action to domain
(dolist (ac *actlist*)
  (send *domain* :add :action ac))

;;
;; problem
;;
(setq *problem*
      (instance pddl-problem :init :name 'plan-based-demo :domain 'pr2_action))

;; symbol definition in this scenerio
(send *problem* :objects
      '((CUP . object)
	(MASTER . object)
	(KITCHEN . spot)
        (OFFICE . spot))
      )

;; initial condition
(send *problem* :initial-condition
      '(;; from sensor-data or heuristics
	(AT OFFICE)
	;; generate from knowledge-base/manip-planner
	(REACHABLE MASTER OFFICE)
	(REACHABLE CUP KITCHEN)
	;; generate fron static information
	(IWANTPASS CUP OFFICE)
	))

;;
;; solve planning
;;

;; we want to enter a room
(send *problem* :goal-condition
;      '((REACHABLE CUP OFFICE)))
      '((NOT (IWANTPASS CUP OFFICE))))
(pprint (setq *result1* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq results (list *result1*))

;; make graph from PDDL answer
(setq *graph* (make-graph-from-pddl-results results :node-name :pprint))
;; swap nodes ( check_open <-> check_open_f ) for making task flow
;(dolist (res results)
;  (setq *graph* (add-failed-nodes res (list 'check_open) :graph *graph*)))

(send *graph* :name (send *domain* :name))

;(send *graph* :write-to-pdf "knock_demo.pdf")
;(when (ros::get-param "~display_graph")
;  (piped-fork "gnome-open knock_demo.pdf"))

;; instance <--> symbol
(setq cup (make-cube 10 10 10))
(setq kitchen (make-cascoords :name "kitchen"))
(setq office (make-cascoords :name "office"))

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))
(defun PR2_ACTION::move (to)
  (let ((coords (eval to)))
    (format t ";; move action [-> ~a]~%" coords) t))
(defun PR2_ACTION::wipe (obj)
  (format t ";; wiping [~a]       ~%" obj) t)
(defun PR2_ACTION::pick (s-obj spot)
  (let ((obj (eval s-obj)))
    (format t ";; picking [~a @ ~a] ~%" obj spot) t))
(defun PR2_ACTION::put (s-obj spot)
  (let ((obj (eval s-obj)))
    (format t ";; put [~a @ ~a]    ~%" obj spot) t))
(defun PR2_ACTION::pass (s-obj spot)
  (let ((obj (eval s-obj)))
    (format t ";; pass [~a @ ~a]    ~%" obj spot) t))

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
