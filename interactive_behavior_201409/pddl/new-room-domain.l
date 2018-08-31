#!/usr/bin/env roseus
;; test-plan.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "test_plan")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")


(defun make-domain ()
  (let ((domain (instance pddl-domain :init :name 'room-domain))
        actions)
    (send domain :requirements
          '(:adl :typing :action-costs :negative-preconditions)
          )
    (send domain :types '(spot arm item person))
    (send domain :constants '(arms - arm cleaner - item somewhere - spot someone - person))
    (send domain :predicates
          '((ROBOT-AT ?AT - spot)
            (SURE-ON ?OBJ - item ?AT - spot)
            (ON ?OBJ - item ?AT - spot)
            (SURE-SEE ?WHO - person ?AT - spot)
            (SEE ?WHO - person ?AT - spot)
            (DELIVERED ?OBJ - item ?WHO - person)
            (ASKED ?WHO - person)
            (ONHAND ?OBJ - item ?ARM - arm)
            (OPEN ?AT - spot)
            (AVAILABLE ?AT - spot)
            (PLACABLE ?OBJ - item ?AT - spot)
            (OPENABLE ?ARM - arm ?AT - spot)
            (CLEAN)
            ))
    (send domain :functions '((total-cost)
                              (distance ?FROM ?TO - spot)
                              (manip-cost ?OBJ - item ?ARM - arm)
                              (find-cost)
                              (door-cost)
                              ))
    (setq actions
          (list
           (instance pddl-action :init
                     :name "move-to"
                     :parameters '((?FROM ?TO spot))
                     :precondition '((ROBOT-AT ?FROM)
                                     (NOT (OPEN ?FROM)))
                     :effect '((NOT (ROBOT-AT ?FROM))
                               (ROBOT-AT ?TO)
                               (increase (total-cost) (distance ?FROM ?TO))))
           (instance pddl-action :init
                     :name "find-object"
                     :parameters '((?OBJ item) (?AT spot))
                     :precondition '((ROBOT-AT ?AT)
                                     (ON ?OBJ ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((SURE-ON ?OBJ ?AT)
                               (NOT (ON ?OBJ ?AT))
                               (INCREASE (TOTAL-COST) (FIND-COST))))
           (instance pddl-action :init
                     :name "pick"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((forall (?OTHER - item)
                                      (AND (NOT (ONHAND ?OTHER ?ARM))
                                           (NOT (ONHAND ?OTHER ARMS))))
                                     (ROBOT-AT ?AT)
                                     (SURE-ON ?OBJ ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((ONHAND ?OBJ ?ARM)
                               (NOT (SURE-ON ?OBJ ?AT))
                               (increase (total-cost) (manip-cost ?OBJ ?ARM))))
           (instance pddl-action :init
                     :name "find-placement"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (ROBOT-AT ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((PLACABLE ?OBJ ?AT)
                               (increase (total-cost) (find-cost))))
           (instance pddl-action :init
                     :name "place"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (ROBOT-AT ?AT)
                                     (AVAILABLE ?AT)
                                     (PLACABLE ?OBJ ?AT)
                                     )
                     :effect '((NOT (ONHAND ?OBJ ?ARM))
                               (ON ?OBJ ?AT)
                               (NOT (PLACABLE ?OBJ ?AT))
                               (increase (total-cost) (manip-cost ?OBJ ?ARM))))
           (instance pddl-action :init
                     :name "open-door"
                     :parameters '((?ARM arm) (?AT spot))
                     :precondition '((FORALL (?OBJ - item)
                                      (AND (NOT (ONHAND ?OBJ ?ARM))
                                           (NOT (ONHAND ?OBJ ARMS))))
                                     (ROBOT-AT ?AT)
                                     (NOT (AVAILABLE ?AT))
                                     (OPENABLE ?ARM ?AT))
                     :effect '((AVAILABLE ?AT)
                               (OPEN ?AT)
                               (increase (total-cost) (door-cost))))
           (instance pddl-action :init
                     :name "close-door"
                     :parameters '((?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ - item)
                                      (AND (NOT (ONHAND ?OBJ ?ARM))
                                           (NOT (ONHAND ?OBJ ARMS))))
                                     (ROBOT-AT ?AT)
                                     (OPENABLE ?ARM ?AT)
                                     (OPEN ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((NOT (AVAILABLE ?AT))
                               (NOT (OPEN ?AT))
                               (increase (total-cost) (door-cost))))
           (instance pddl-action :init
                     :name "clean-room"
                     :parameters '((?ARM arm) (?AT spot))
                     :precondition '((ONHAND CLEANER ?ARM)
                                     (ROBOT-AT ?AT))
                     :effect '((CLEAN)
                               (NOT (ROBOT-AT ?AT))
                               (ROBOT-AT SOMEWHERE)))
           (instance pddl-action :init
                     :name "find-person"
                     :parameters '((?WHO person) (?AT spot))
                     :precondition '((ROBOT-AT ?AT)
                                     (SEE ?WHO ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((SURE-SEE ?WHO ?AT)
                               (NOT (SEE ?WHO ?AT))))
           (instance pddl-action :init
                     :name "deliver"
                     :parameters '((?OBJ item) (?ARM arm) (?WHO person) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (ROBOT-AT ?AT)
                                     (SURE-SEE ?WHO ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((NOT (ONHAND ?OBJ ?ARM))
                               (DELIVERED ?OBJ ?WHO)))
           (instance pddl-action :init
                     :name "ask"
                     :parameters '((?WHO person) (?AT spot))
                     :precondition '((ROBOT-AT ?AT)
                                     (SURE-SEE ?WHO ?AT)
                                     (AVAILABLE ?AT))
                     :effect '((ASKED ?WHO)))
           ))

    ;; failed actions

    (setq actions
          (append actions
                  (list
                   (instance pddl-action :init
                             :name "find-object_f"
                             :parameters '((?OBJ item) (?AT spot))
                             :precondition '((ROBOT-AT ?AT)
                                             (ON ?OBJ ?AT)
                                             (AVAILABLE ?AT))
                             :effect '((NOT (ON ?OBJ ?AT))))
                   (instance pddl-action :init
                             :name "pick_f"
                             :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                             :precondition '((forall (?OTHER - item)
                                                     (AND (NOT (ONHAND ?OTHER ?ARM))
                                                          (NOT (ONHAND ?OTHER ARMS))))
                                             (ROBOT-AT ?AT)
                                             (SURE-ON ?OBJ ?AT)
                                             (AVAILABLE ?AT))
                             :effect '((ON ?OBJ ?AT)
                                       (NOT (SURE-ON ?OBJ ?AT))
                                       (increase (total-cost) (manip-cost ?OBJ ?ARM))))
                   #|
                   (instance pddl-action :init
                             :name "find-placement_f"
                             :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                             :precondition '((ONHAND ?OBJ ?ARM)
                                             (ROBOT-AT ?AT)
                                             (AVAILABLE ?AT))
                             :effect '((NOT (AVAILABLE ?AT))))
                   |#
                   (instance pddl-action :init
                             :name "find-person_f"
                             :parameters '((?WHO person) (?AT spot))
                             :precondition '((ROBOT-AT ?AT)
                                             (SEE ?WHO ?AT)
                                             (AVAILABLE ?AT))
                             :effect '((NOT (SEE ?WHO ?AT))))
                   )))

    (dolist (ac actions)
      (send domain :add :action ac))
    domain))

(defun make-spot-distance (spots)
  ;; TODO: compute from eus
  (let (init)
    (dolist (from spots)
      (dolist (to spots)
        (unless (eq from to)
          (push `(= (distance ,from ,to) 10) init))))
    (dolist (spot spots)
      (push `(= (distance ,spot ,spot) 1000) init))
    init))

(defun make-problem (&key init goal items people)
  (let ((problem (instance pddl-problem :init
                           :name 'house-keeping
                           :domain 'room-domain))
        (spots '(START TABLE FRIDGE DISHWASHER DRAWER SINK KITCHEN COUNTER ROOM73A1))
        objs)

    ;; manip cost
    (let ((unique-items (remove-duplicates items)))
      (dolist (item (append unique-items (list 'cleaner)))
        (push `(= (manip-cost ,item larm) 10) init)
        (push `(= (manip-cost ,item rarm) 10) init)
        (push `(= (manip-cost ,item arms) 20) init)))

    ;; distance cost
    (let ((unique-spots (remove-duplicates spots)))
      (setq init (nconc init (make-spot-distance
                              (append unique-spots (list 'somewhere))))))

    ;; other costs
    (push '(= (total-cost) 0) init)
    (push '(= (door-cost) 10) init)
    (push '(= (find-cost) 50) init)

    ;; default conds
    (setq init (nconc init '((OPENABLE RARM DISHWASHER)
                             (OPENABLE RARM FRIDGE)
                             ;; (OPENABLE LARM FRIDGE)
                             (OPENABLE RARM ROOM73A1)
                             (AVAILABLE SINK)
                             (AVAILABLE COUNTER)
                             (AVAILABLE KITCHEN)
                             (AVAILABLE TABLE)
                             (AVAILABLE DRAWER))))

    ;; objects
    (dolist (item items)
      (push `(,item . item) objs))
    (dolist (spot spots)
      (push `(,spot . spot) objs))
    (dolist (person people)
      (push `(,person . person) objs))
    (setq objs (nconc objs '((LARM . arm) (RARM . arm))))

    (send problem :objects objs)
    (send problem :initial-condition init)
    (send problem :goal-condition goal)
    (send problem :metric '(minimize (total-cost)))
    problem))

(defun make-prepare-problem ()
  (make-problem :init
                '((ROBOT-AT START)
                  (ON BOWL DISHWASHER)
                  (ON BOTTLE FRIDGE)
                  (ON SPOON KITCHEN)
                  (ON CUP KITCHEN))
                :goal
                '((ON BOWL TABLE)
                  (ON CUP TABLE)
                  (ON BOTTLE TABLE))
                :items
                '(CUP BOTTLE BOWL SPOON)
                ))

(defun make-clean-room-problem ()
  (make-problem :init
                '((ROBOT-AT START)
                  (ON BOWL DISHWASHER)
                  (ON BOTTLE FRIDGE)
                  (ON SPOON KITCHEN)
                  (ON CUP KITCHEN)
                  (ON CLEANER DRAWER))
                :goal
                '((ON CLEANER DRAWER)
                  (CLEAN))
                :items
                '(CUP BOTTLE BOWL SPOON)
                ))

(defun make-ask-inaba-problem ()
  (make-problem :init
                '((ROBOT-AT START)
                  (ON BOWL DISHWASHER)
                  (ON BOTTLE FRIDGE)
                  (ON SPOON KITCHEN)
                  (ON CUP KITCHEN)
                  (ON CLEANER DRAWER)
                  (SEE INABA ROOM73A1))
                :goal
                '((ASKED INABA))
                :items
                '(CUP BOTTLE BOWL SPOON)
                :people
                '(INABA)
                ))
#|
(defun temp-func ()
    (send problem :objects '((START . spot)
                             (KITCHEN . spot)
                             (TABLE . spot)
                             (FRIDGE . spot)
                             (DISHWASHER . spot)
                             (DRAWER . spot)
                             (ROOM73A1 . spot)
                             (INABA . person)
                             (LARM . arm)
                             (RARM . arm)
                             (SPOON . item)
                             (CUP . item)
                             (BOTTLE . item)
                             (BOWL . item)))

    (send problem :initial-condition
          '((ROBOT-AT START)
            (ON BOWL DISHWASHER)
            (ON BOTTLE FRIDGE)
            (ON SPOON KITCHEN)
            (ON CUP KITCHEN)
            (ON CLEANER DRAWER)
            (SEE INABA ROOM73A1)
            (OPENABLE RARM DISHWASHER)
            (OPENABLE LARM FRIDGE)
            (OPENABLE RARM FRIDGE)
            (OPENABLE RARM ROOM73A1)
            (AVAILABLE KITCHEN)
            (AVAILABLE TABLE)
            (AVAILABLE DRAWER)
            (= (total-cost) 0)
            (= (door-cost) 1000)))

    (let (inits
          (spots (mapcar #'car
                         (remove-if-not
                          #'(lambda (x) (eq (cdr x) 'spot)) (send problem :objects))))
          (items (mapcar #'car
                         (remove-if-not
                          #'(lambda (x) (eq (cdr x) 'item)) (send problem :objects)))))
      (push 'somewhere spots)
      (push 'cleaner items)
      ;; distance
      (let ((cost 100))
        (dolist (from spots)
          (dolist (to spots)
            (unless (eq from to)
              (push `(= (distance ,from ,to) ,(incf cost 1)) inits)))))
      (dolist (s spots)
        (push `(= (distance ,s ,s) 10000) inits))
      ;; manip
      (let ((cost 10))
        (dolist (a '(larm rarm))
          (dolist (i items)
            (push `(= (manip-cost ,i ,a) ,(incf cost 1)) inits))))
      (let ((cost 20))
        (dolist (i items)
          (push `(= (manip-cost ,i arms) ,(incf cost 1)) inits)))

      (send problem :initial-condition
            (append (send problem :initial-condition) inits)))

    (send problem :goal-condition
          '((ON BOWL TABLE)
            (ON CUP TABLE)
            (ON BOTTLE TABLE)
            ))

    ;; (send problem :goal-condition
    ;;       '((ON CLEANER DRAWER)
    ;;         (CLEAN)))

    ;; (send problem :goal-condition
    ;;       '((ASKED INABA)))

    (send problem :metric '(minimize (total-cost)))
    problem)
|#
(defun sample-plan (&optional (func #'make-prepare-problem))
  (setq *domain* (make-domain))
  (setq *problem* (funcall func))
  (setq *failed-nodes*
        (remove-if-not #'(lambda (n)
                           (string= "_f"
                                    (subseq (send n :name)
                                            (- (length (send n :name)) 2))))
                       (send *domain* :action)))
  (setq *failed-nodes*
        (mapcar #'(lambda (n)
                    (read-from-string
                     (subseq (send n :name) 0 (- (length (send n :name)) 2))))
                *failed-nodes*))
  (setq *graph* (pddl-plan-to-graph nil :domain *domain* :problem *problem*
                               :failed-nodes *failed-nodes* :timeout 10
                               :debug t))
  (send *graph* :write-to-pdf "/tmp/graph.pdf" nil "hoge")
  (setq insp (pddl-graph-to-smach *graph*
               :add-self-return-failure-path nil
               :use-userdata t))
  (send insp :publish-structure)
  (send insp :publish-all-status)
  insp)

(defun sample-run (&optional (func #'make-prepare-problem))
  (setq *insp* (sample-plan func))
  (exec-preemptive-task (send *insp* :state-machine)))
