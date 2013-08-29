#!/usr/bin/env roseus
;; mostly same to demo-eus.l in pddl package under jsk subversion trunk
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
;; room610
(setq *problem* (instance pddl-problem :init :name 'room610 :domain 'manip))

;; variables
(send *problem* :objects
      '((tray   . object)
        (cloth  . object)
        (broom  . object)

        (chair-back . object)
        (chair . object)
        (table . object)
        (sink  . object)
        (laundry . object)

        (room . object)
        (world . object)
        ))
                                        ;; 簡単のため使う型は一つのみに
                                        ;; disk0,1,2 pole0,1,2すべてobject型

;; initial condition
(send *problem* :initial-condition
      '((on tray  table)
        (on cloth chair)
        (on broom laundry)
        (under chair table)
        (at world)
        (not (in cloth laundry))
        ))

;; goal condition
(send *problem* :goal-condition
      '((clean room)
        (clean-under table)
        (on tray sink)
        (under chair table)
        (on broom laundry)
        (in cloth laundry)
        (working laundry)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'manip))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object))
(send *domain* :predicates '((on ?obj ?from - object)
                             (at ?from - object)
                             (in ?obj0 ?obj1 - object)
                             (hold ?obj - object)
                             (clean ?room - object)
                             (clean-under ?obj - object)
                             (space ?room - object)
                             (open-door ?obj - object)
                             (under ?obj ?space - object)
                             (working ?obj - object)
                             ))
;;making action
(setq *move* (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?to object))
                       :precondition '()
                       :effect '((at ?to)
								 (forall (?from - object)
								  (when (not (= ?from ?to))
									(not (at ?from)))))))

(setq *pick* (instance pddl-action :init
                       :name "pick"
                       :parameters '((?obj ?from object))
                       :precondition '((at ?from)
                                       (on ?obj ?from)
                                       (forall (?any - object) (not (hold ?any))))
                       :effect '((not (on ?obj ?from))
                                 (hold ?obj))))

(setq *place* (instance pddl-action :init
                       :name "place"
                       :parameters '((?obj ?to object))
                       :precondition '((at ?to)
                                       (hold ?obj))
                       :effect '((not (hold ?obj))
                                 (on ?obj ?to))))

(setq *put* (instance pddl-action :init
                       :name "put"
                       :parameters '((?obj ?to object))
                       :precondition '((at ?to)
                                       (hold ?obj)
                                       (open-door ?to))
                       :effect '((not (hold ?obj))
                                 (in ?obj ?to))))

(setq *push-button* (instance pddl-action :init
                              :name "push-button"
                              :parameters '((?obj object))
                              :precondition '((at ?obj)
                                              (in cloth ?obj)
                                              (not (open-door ?obj))
                                              )
                              :effect '((working ?obj))))


(setq *open-door* (instance pddl-action :init
                       :name "open-door"
                       :parameters '((?obj object))
                       :precondition '((at ?obj)
                                       (not (open-door ?obj)))
                       :effect '((open-door ?obj))))

(setq *close* (instance pddl-action :init
                       :name "close"
                       :parameters '((?obj object))
                       :precondition '((at ?obj)
                                       (open-door ?obj))
                       :effect '((not (open-door ?obj)))))

(setq *sweep* (instance pddl-action :init
                       :name "sweep"
                       :parameters '((?space object))
                       :precondition '((hold broom)
                                       (under chair table))
                       :effect '((clean ?space))))

(setq *sweep-under* (instance pddl-action :init
                       :name "sweep-under"
                       :parameters '((?space object))
                       :precondition '((hold broom)
                                       (space ?space)
                                       (at ?space))
                       :effect '((clean-under ?space))))

(setq *pull* (instance pddl-action :init
                       :name "pull-chair"
                       :parameters '((?from object))
                       :precondition '((under chair ?from) ; (under ?obj ?from)
                                       (at chair))
                       :effect '((not (under chair ?from))
                                 (space ?from)
                                 (at chair-back))))

(setq *push* (instance pddl-action :init
                       :name "push-chair"
                       :parameters '((?to object))
                       :precondition '((not (under chair ?to))
                                       (at chair))
                       :effect '((under chair ?to) ; (under ?obj ?to)
                                 (not (space ?to))
                                 (at chair))))




;;add action to domain
(send *domain* :add :action *move*)
(send *domain* :add :action *pick*)
(send *domain* :add :action *place*)
(send *domain* :add :action *sweep*)
(send *domain* :add :action *sweep-under*)
(send *domain* :add :action *put*)
(send *domain* :add :action *open-door*)
(send *domain* :add :action *close*)
(send *domain* :add :action *pull*)
(send *domain* :add :action *push*)
(send *domain* :add :action *push-button*)


;;
;; solve planning
;;
(setq *result* (solve-pddl-planning *domain* *problem*))
(format t "plan = ")
(pprint (cdr (assoc :plan *result*)))
;(format t "result = ~A " *result*)
;(ros::exit)

;; generate graph
(load "package://pddl_planner/src/pddl-result-graph.l")
(defun make-graph ()
  (setq *graph* (make-graph-from-pddl-results (list *result*)))
  (send (make-readable-graph *graph*) :write-to-pdf "pddl_graph.pdf"))

(defun make-smach ()
  (make-graph)
  (send *graph* :name (send *domain* :name))
  ;; convert to smach
  (load "package://roseus_smach/src/state-machine-ros.l")
  (load "package://roseus_smach/src/state-machine-utils.l")
  (load "package://roseus_smach/src/pddl2smach.l")

  (defun move-to (&rest args)
	(warn "move-to ~A~%" args)
	t)

  (defun pick (&rest args)
	(warn "pick ~A~%" args)
	t)
 
  (defun sweep (&rest args)
	(warn "sweep ~A~%" args)
	t)

  (defun open-door (&rest args)
	(warn "open-door ~A~%" args)
	t)

  (defun close (&rest args)
	(warn "close ~A~%" args)
	t)

  (defun push-button (&rest args)
	(warn "push-button ~A~%" args)
	t)
  (defun place (&rest args)
	(warn "place ~A~%" args)
	t)

  (defun sweep-under (&rest args)
	(warn "sweep-under ~A~%" args)
	t)

  (defun push-chair (&rest args)
	(warn "push-chair ~A~%" args)
	t)

  (defun put (&rest args)
	(warn "put ~A~%" args)
	t)

  (defun pull-chair (&rest args)
	(warn "pull-chair ~A~%" args)
	t)


  ;; global data is not used (nil)
  (exec-smach-with-spin (convert-smach *graph*) nil :hz 1.0)

  (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
	(ros::exit)))
