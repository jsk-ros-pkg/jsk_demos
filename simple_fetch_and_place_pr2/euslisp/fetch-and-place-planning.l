;; pick-and-place-plan.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'kitchen73b2))
(send *domain* :requirements '(:typing :action-costs))
(send *domain* :types '(object spot arm action))
(send *domain* :predicates '((ONHAND ?OBJ - object ?ARM - arm)
                             (PICKABLE ?OBJ - object ?ARM - arm)
                             (ON ?OBJ - object ?OBJ2 - object)
                             (OBJECT-AT ?OBJ - object ?SPOT - spot)
                             (AT ?SPOT - spot)
                             (FOUND ?OBJ)
                             ))
(send *domain* :constants '(RARM - arm
                            ;; LARM - arm
                            ;; ARMS - arm
                            FIND PICK PLACE-ON PLACE MOVE-TO - action
                            ))

(send *domain* :functions '((total-cost)
                            (cost ?ACT - action)))


(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "find-object"
                 :parameters '((?OBJ object))
                 :precondition '((NOT (FOUND ?OBJ))
                                 (exists (?SPOT - spot)
                                  (and
                                   (AT ?SPOT)
                                   (OBJECT-AT ?OBJ ?SPOT))))
                 :effect '((FOUND ?OBJ)
                           (increase (total-cost) (cost FIND))))
       (instance pddl-action :init
                 :name "approach"
                 :parameters '((?OBJ object) (?ARM arm))
                 :precondition '((FOUND ?OBJ)
                                 (NOT (PICKABLE ?OBJ ?ARM)))
                 :effect '((PICKABLE ?OBJ ?ARM)))
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ object) (?ARM arm))
                 :precondition '((FOUND ?OBJ)
                                 (PICKABLE ?OBJ ?ARM)
                                 (forall (?OBJ2 - object)
                                  (NOT (ON ?OBJ2 ?OBJ)))
                                 (forall (?OBJ2 - object)
                                  (AND
                                   (NOT (ONHAND ?OBJ2 ARMS))
                                   (NOT (ONHAND ?OBJ2 ?ARM))))
                                 (OR
                                  (= ?ARM LARM)
                                  (= ?ARM RARM)
                                  (forall (?ARM2 - arm)
                                    (forall (?OBJ2 - object)
                                     (NOT (ONHAND ?OBJ2 ?ARM2)))))
                                 (exists (?SPOT - spot)
                                  (and
                                   (AT ?SPOT)
                                   (OBJECT-AT ?OBJ ?SPOT))))
                 :effect '((ONHAND ?OBJ ?ARM)
                           (forall (?OBJ2 - object)
                            (NOT (ON ?OBJ ?OBJ2)))
                           (forall (?SPOT - spot)
                            (NOT (OBJECT-AT ?OBJ ?SPOT)))
                           (increase (total-cost) (cost PICK))))
       (instance pddl-action :init
                 :name "pick_f"
                 :parameters '((?OBJ object) (?ARM arm))
                 :precondition '((FOUND ?OBJ)
                                 (PICKABLE ?OBJ ?ARM)
                                 (forall (?OBJ2 - object)
                                  (NOT (ON ?OBJ2 ?OBJ)))
                                 (forall (?OBJ2 - object)
                                  (AND
                                   (NOT (ONHAND ?OBJ2 ARMS))
                                   (NOT (ONHAND ?OBJ2 ?ARM))))
                                 (OR
                                  (= ?ARM LARM)
                                  (= ?ARM RARM)
                                  (forall (?ARM2 - arm)
                                    (forall (?OBJ2 - object)
                                     (NOT (ONHAND ?OBJ2 ?ARM2)))))
                                 (exists (?SPOT - spot)
                                  (and
                                   (AT ?SPOT)
                                   (OBJECT-AT ?OBJ ?SPOT))))
                 :effect '((NOT (PICKABLE ?OBJ ?ARM))
                           (increase (total-cost) (cost PICK))))
       (instance pddl-action :init
                 :name "place-on"
                 :parameters '((?OBJ ?TO object) (?ARM arm))
                 :precondition '((FOUND ?OBJ)
                                 (FOUND ?TO)
                                 (ONHAND ?OBJ ?ARM)
                                 (exists (?SPOT - spot)
                                  (and
                                   (AT ?SPOT)
                                   (OBJECT-AT ?TO ?SPOT))))
                 :effect '((NOT (ONHAND ?OBJ ?ARM))
                           (forall (?SPOT - spot)
                            (when (AT ?SPOT)
                              (OBJECT-AT ?OBJ ?SPOT)))
                           (ON ?OBJ ?TO)
                           (increase (total-cost) (cost PLACE-ON))
                           ))
       (instance pddl-action :init
                 :name "place"
                 :parameters '((?OBJ object) (?ARM arm))
                 :precondition '((ONHAND ?OBJ ?ARM))
                 :effect '((forall (?SPOT - spot)
                            (when (AT ?SPOT)
                              (OBJECT-AT ?OBJ ?SPOT)))
                           (NOT (ONHAND ?OBJ ?ARM))
                           (increase (total-cost) (cost PLACE))
                           ))
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?SPOT spot))
                 :precondition '((NOT (AT ?SPOT)))
                 :effect '((forall (?SPOT2 - spot)
                            (NOT (AT ?SPOT2)))
                           (AT ?SPOT)
                           (increase (total-cost) (cost MOVE-TO))
                           ))
))
(dolist (ac *action-list*)
  (send *domain* :add :action ac))


;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'serve_drink :domain 'kitchen73b2))

;; decleration of variables
(send *problem* :objects
      '((TABLE . spot)
        (COOK . spot)
        (COE . spot)
        (BOTTLE . object)
        (MUG-CUP . object)
        (CUP . object)
        ))

;; initial condition
(send *problem* :initial-condition
      '((OBJECT-AT BOTTLE COE)
        (OBJECT-AT MUG-CUP COOK)
        (OBJECT-AT TRAY COOK)
        (OBJECT-AT CUP TABLE)
        (= (COST PICK) 5)
        (= (COST PLACE) 5)
        (= (COST PLACE-ON) 10)
        (= (COST MOVE-TO) 5)
        (= (COST FIND) 5)
        (= (TOTAL-COST) 0)
        ))
#|
(send *problem* :goal-condition
      '((OBJECT-AT BOTTLE TABLE)
        (OBJECT-AT MUG-CUP TABLE)))
|#


(send *problem* :goal-condition
      '((OBJECT-AT CUP COOK)))

(send *problem* :metric '(minimize (total-cost)))

;; declare defined-failure nodes
(setq *failed-nodes* '(pick)) ;;(list 'push-button 'look))

;; solve plan and make transition graph
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(setq *graph*
      (pddl-plan-to-graph nil :domain *domain* :problem *problem* :failed-nodes *failed-nodes* :debug t))

;; show graph
(when (ros::get-param "~display_graph" t)
  (send *graph* :write-to-pdf "/tmp/graph.pdf" nil "simple_failure_torelant")
  (piped-fork "gnome-open /tmp/graph.pdf"))

;; convert into SMACH
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

(setq *sm-inspect* (pddl-graph-to-smach *graph*
                                        :add-self-return-failure-path nil))
(unix::sleep 1)
(send *sm-inspect* :reset-state)
