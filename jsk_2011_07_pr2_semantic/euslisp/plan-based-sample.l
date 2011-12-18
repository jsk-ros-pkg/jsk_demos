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
				   (when (NOT (= ?TO ?from))
				     (NOT (AT ?from))))))
       (instance pddl-action :init
		 :name "pick"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?spot - spot)
					 (OR (NOT (AT ?spot))
					     (AND (AT ?spot)
						  (REACHABLE ?OBJ ?spot))))
				 (forall (?objx - object)
					 (NOT (GRASP ?objx))))
		 :effect '((GRASP ?OBJ)
			   (forall (?spot - spot)
				   (NOT (REACHABLE ?OBJ ?spot)))
			   ))
       (instance pddl-action :init
		 :name "pick_f"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?spot - spot)
					 (OR (NOT (AT ?spot))
					     (AND (AT ?spot)
						  (REACHABLE ?OBJ ?spot))))
				 (forall (?objx - object)
					 (NOT (GRASP ?objx))))
		 :effect '((NOT (GRASP ?OBJ))
			   ))
       (instance pddl-action :init
		 :name "put"
		 :parameters '((?OBJ object))
		 :precondition '((GRASP ?OBJ))
		 :effect '((NOT (GRASP ?OBJ))
			   (forall (?spot -spot)
				   (when (AT ?spot)
				     (REACHABLE ?OBJ ?SPOT)))
			   ))
       (instance pddl-action :init
		 :name "pass"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?spot - spot)
					 (OR (NOT (AT ?spot))
					     (AND (AT ?spot)
						  (IWANTPASS ?OBJ ?spot))))
				 (GRASP ?OBJ))
		 :effect '((forall (?spot - spot)
				   (when (AT ?spot)
				     (NOT (IWANTPASS ?OBJ ?SPOT))))
			   (NOT (GRASP ?OBJ))))
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
(dolist (res results)
  (setq *graph* (add-failed-nodes res (list 'pick) :graph *graph*)))

(send *graph* :name (send *domain* :name))

(send (make-readable-graph *graph*) :write-to-pdf "knock_demo.pdf")
;(send *graph* :write-to-pdf "knock_demo.pdf")
;(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open knock_demo.pdf")
;)

;; instance <--> symbol
(setq cup (make-cube 10 10 10))
(send cup :move-to (make-coords :pos #f(53430.0 6200.0 752.0) :rpy #f(3.142 0.0 0.0)))
(setq kitchen (make-cascoords :name "kitchen" :pos #f(53430.0 5500.0 0.0) :rpy #f(1.571 0.0 0.0)))
(setq office (make-cascoords :name "office" :pos #f(56141.062 4734.342 1.105) :rpy #f(-2.675 0.01 -0.011)))

;; action definition
;; domain -> package
;; this file is created by (output-domain-template "pr2_action.l" *domain*)
(let ((ns (string-upcase (send *domain* :name))))
  (when (and ns (not (find-package ns)))
      (make-package ns)))
(defun PR2_ACTION::move (to)
  (let ((coords (eval to)))
    (format t ";; move action [-> ~a]~%" coords)
    (clear-costmap)
    (send *ri* :move-to coords)))
(defun PR2_ACTION::wipe (obj)
  (format t ";; wiping [~a]       ~%" obj) t)
(defun PR2_ACTION::pick (s-obj)
  (let ((obj (eval s-obj)))
    (format t ";; picking [~a] ~%" obj)
    (ros::spin-once)
    (send *pr2* :move-to (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)) :world)
    (send *pr2* :head :look-at (send obj :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (speak-jp "こっぷをさがしています")
    (let ((neck-av (send *pr2* :head :angle-vector)))
      (setq flag
	    (or (check :timeout 6 :type name)
		(progn (send *pr2* :head :angle-vector (v+ #f(20 0) neck-av))
		       (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		       (send *ri* :wait-interpolation)
		       (check :timeout 6 :type name))
		(progn (send *pr2* :head :angle-vector (v+ #f(-20 0) neck-av))
		       (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		       (send *ri* :wait-interpolation)
		       (check :timeout 6 :type name)))
	    ))
    (when (not flag)
      (return-from pr2_action::pick nil))
    (speak-jp "こっぷがみつかりました")
    (send *pr2* :head :look-at (send *obj* :worldpos))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 500)
    (or (pick :larm)
	(progn
	  (check :timeout 4)
	  (pick :larm)))))
(defun PR2_ACTION::put (s-obj)
  (let ((obj (eval s-obj)))
    (format t ";; put [~a]    ~%" obj) t))
(defun PR2_ACTION::pass (s-obj)
  (let ((obj (eval s-obj)))
    (format t ";; pass [~a]    ~%" obj)
    (speak-jp "こっぷをもってきました")
    (hand-over :larm)
    t ))

;; convert to smach
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

;; global data is not used (nil)
;(exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
