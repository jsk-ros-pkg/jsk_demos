#!/usr/bin/env roseus
;; solve-fridge-demo.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")

(setq *domain* (instance pddl-domain :init :name 'room73b2))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item))
(send *domain* :predicates '((reachable ?OBJ - item)
                             (found ?OBJ - item)
                             (hidden ?OBJ)
                             (onhand ?OBJ - item)
                             (closed ?OBJ - item)
                             (empty ?OBJ - item)
                             (notfound)
                             (in ?OBJ - item ?OBJ2 - item)
                             (on ?OBJ - item ?OBJ2 - item)))

(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "look-around"
                 :parameters '()
                 :precondition '((notfound))
                 :effect '((not (notfound))))
       (instance pddl-action :init
                 :name "approach-to"
                 :parameters '((?OBJ item))
                 :precondition '((found ?OBJ) (not (notfound)))
                 :effect '((forall (?OBJ2 - item)
                            (when (not (= ?OBJ ?OBJ2))
                              (and (not (reachable ?OBJ2))
                                   (not (found ?OBJ2)))))
                           (reachable ?OBJ)))
       (instance pddl-action :init
                 :name "approach-to_f"
                 :parameters '((?OBJ item))
                 :precondition '((found ?OBJ) (not (notfound)))
                 :effect '((forall (?OBJ2 - item)
                            (and (not (reachable ?OBJ2))
                                 (not (found ?OBJ2))))
                           (notfound)))
       (instance pddl-action :init
                 :name "find-object"
                 :parameters '((?OBJ item))
                 :precondition '((not (found ?OBJ)) (not (hidden ?OBJ)) (not (notfound)))
                 :effect '((found ?OBJ)))
       (instance pddl-action :init
                 :name "find-object_f"
                 :parameters '((?OBJ item))
                 :precondition '((not (found ?OBJ)) (not (hidden ?OBJ)) (not (notfound)))
                 :effect '((notfound)))
       (instance pddl-action :init
                 :name "grasp-object"
                 :parameters '((?OBJ item))
                 :precondition '((reachable ?OBJ))
                 :effect '((forall (?OBJ2 - item)
                            (not (on ?OBJ ?OBJ2)))
                           (onhand ?OBJ)))
       (instance pddl-action :init
                 :name "grasp-object_f"
                 :parameters '((?OBJ item))
                 :precondition '((reachable ?OBJ))
                 :effect '((not (found ?OBJ))))
       (instance pddl-action :init
                 :name "open-door"
                 :parameters '((?OBJ item))
                 :precondition '((reachable ?OBJ)
                                 (closed ?OBJ))
                 :effect '((not (closed ?OBJ))
                           (forall (?OBJ2 - item)
                            (when (in ?OBJ2 ?OBJ)
                              (not (hidden ?OBJ2))))))
       (instance pddl-action :init
                 :name "close-door"
                 :parameters '((?OBJ item))
                 :precondition '((not (closed ?OBJ)))
;;                                 (reachable ?OBJ))
                 :effect '((closed ?OBJ)
                           (forall (?OBJ2 - item)
                            (when (in ?OBJ2 ?OBJ)
                              (and
                               (not (found ?OBJ2))
                               (not (reachable ?OBJ2))
                               (hidden ?OBJ2))))))
       (instance pddl-action :init
                 :name "place-object"
                 :parameters '((?OBJ item))
                 :precondition '((onhand ?OBJ))
                 :effect '((not (onhand ?OBJ))
                           (forall (?OBJ2 - item)
                            (when (reachable ?OBJ2)
                              (on ?OBJ ?OBJ2)))))
       ))
(dolist (ac *action-list*)
  (send *domain* :add :action ac))

(setq *problem* (instance pddl-problem :init :name 'fridge_demo :domain 'room73b2))

(send *problem* :objects
      '((CAN . item)
        (TABLE . item)
        (DUSTBOX . item)
        (FRIDGE . item)))

(send *problem* :initial-condition '((closed FRIDGE)
                                     (in CAN FRIDGE)
                                     (hidden CAN)
                                     (on CAN FRIDGE)))
(send *problem* :goal-condition '((closed FRIDGE)
                                  (on CAN TABLE)))
#|
(send *problem* :initial-condition '((on CAN TABLE)))
(send *problem* :goal-condition '((on CAN DUSTBOX)))
|#
;;(setq *failed-nodes* '(approach-to find-object grasp-object))
(setq *failed-nodes* '(approach-to))
(setq *graph*
      (pddl-plan-to-graph nil :domain *domain* :problem *problem* :failed-nodes *failed-nodes* :debug t))

(load "package://roseus_smach/src/pddl2smach.l")
(load "package://roseus_smach/src/state-machine-ros.l")

(setq *sm-inspect* (pddl-graph-to-smach *graph*))

(send *sm-inspect* :publish-structure)
(send *sm-inspect* :reset-state)
(send *sm-inspect* :publish-all-status)
(send *sm-inspect* :publish-all-status)


(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
