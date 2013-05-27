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

        (chair . object)
        (table . object)
        (sink  . object)

        (room . object)
        (world . object)
        (laundry . object)
        ))
                                        ;; 簡単のため使う型は一つのみに
                                        ;; disk0,1,2 pole0,1,2すべてobject型

;; initial condition
(send *problem* :initial-condition
      '((on tray  table)
        (on cloth chair)
        (at world)
        ))

;; goal condition
(send *problem* :goal-condition
      '((clean room)
        (on tray sink)
        (on cloth laundry)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'manip))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object))
(send *domain* :predicates '((on ?obj ?from - object)
                             (at ?from - object)
                             (hold ?obj - object)
                             (clean ?room - object)))
;;making action
(setq *move* (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?from ?to object))
                       :precondition '((at ?from))
                       :effect '((not (at ?from))
                                 (at ?to))))

(setq *pick* (instance pddl-action :init
                       :name "pick"
                       :parameters '((?obj ?from object))
                       :precondition '((at ?from)
                                       (on ?obj ?from))
                       :effect '((not (on ?obj ?from))
                                 (hold ?obj))))

(setq *place* (instance pddl-action :init
                       :name "place"
                       :parameters '((?obj ?to object))
                       :precondition '((at ?to)
                                       (hold ?obj))
                       :effect '((not (hold ?obj))
                                 (on ?obj ?to))))

(setq *sweep* (instance pddl-action :init
                       :name "clean"
                       :parameters '((?room object))
                       :precondition '()
                       :effect '((clean ?room))))


;;add action to domain
(send *domain* :add :action *move*)
(send *domain* :add :action *pick*)
(send *domain* :add :action *place*)
(send *domain* :add :action *sweep*)


;;
;; solve planning
;;
(setq *result* (solve-pddl-planning *domain* *problem*))
(format t "plan = ")
(pprint (cdr (assoc :plan *result*)))
(format t "result = ~A " *result*)
(ros::exit)