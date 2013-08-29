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
        (table-front . object)
        (table-side . object)
        (sink  . object)
        (laundry . object)

        (room . object)
        (world . object)
        ))
                                        ;; 簡単のため使う型は一つのみに
                                        ;; disk0,1,2 pole0,1,2すべてobject型

;; initial condition
(send *problem* :initial-condition
      '((on tray  table-side)
        (on cloth chair)
        (on broom laundry)
        (under chair table-front)
        (at world)
        (not (in cloth laundry))
;        (= (total-cost) 0)
;        (= (move-to) 10)
;        (= (sweep) 100)
        ))

;; goal condition
(send *problem* :goal-condition
      '((clean room)
        (clean-under table-front)
        (on tray sink)
        (under chair table-front)
        (on broom laundry)
        (in cloth laundry)
        (working laundry)
        ))

;(send *problem* :metric '(minimize (total-cost)))

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
#|
(send *domain* :functions '((total-cost)
                            (move-to)
                            (sweep)
							))
|#

;;making action
(setq *move* (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?to object))
                       :precondition '()
                       :effect '((forall (?any - object) (not (at ?any)))
                                 (at ?to)
;                                 (increase (total-cost) (move-to))
                                 )))

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
                                       (under chair table-front))
                       :effect '((clean ?space)
;                                 (increase (total-cost) (sweep))
								 )))

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
(defun debug-solve ()
  (setq *result* (solve-pddl-planning *domain* *problem*))
  (format t "plan = ")
  (pprint (cdr (assoc :plan *result*)))
  )

;; generate graph
(load "package://pddl_planner/src/pddl-result-graph.l")
(defun debug-make-graph ()
  (setq *graph* (make-graph-from-pddl-results (list *result*)))
  (send (make-readable-graph *graph*) :write-to-pdf "pddl_graph.pdf"))
#|
;(format t "result = ~A " *result*)
(ros::exit)
|#
