#!/usr/bin/env roseus
;; tidyup-cup.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")

(defun make-domain ()
  (let ((domain (instance pddl-domain :init :name 'tidyup-domain))
        actions)
    (send domain :requirements '(:typing :action-costs))
    (send domain :types '(item spot arm))
    (send domain :predicates
          '((ON ?OBJ - item ?SPOT - spot)
            (ONHAND ?OBJ - item ?ARM - arm)
            (AT ?SPOT - spot)))
    (send domain :functions '((total-cost)))
    (setq actions
          (list
             (instance pddl-action :init
                       :name "pick"
                       :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                       :precondition '((forall (?OBJ2 - item)
                                        (not (onhand ?OBJ2 ?ARM)))
                                       (AT ?AT)
                                       (ON ?OBJ ?AT))
                       :effect '((ONHAND ?OBJ ?ARM)
                                 (NOT (ON ?OBJ ?AT))))
             (instance pddl-action :init
                       :name "pick_f"
                       :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                       :precondition '((forall (?OBJ2 - item)
                                        (not (onhand ?OBJ2 ?ARM)))
                                       (AT ?AT)
                                       (ON ?OBJ ?AT))
                       :effect '())
             (instance pddl-action :init
                       :name "place"
                       :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                       :precondition '((ONHAND ?OBJ ?ARM)
                                       (AT ?AT))
                       :effect '((NOT (ONHAND ?OBJ ?ARM))
                                 (ON ?OBJ ?AT)))
             (instance pddl-action :init
                       :name "place_f"
                       :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                       :precondition '((ONHAND ?OBJ ?ARM)
                                       (AT ?AT))
                       :effect '())
             (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?FROM ?TO spot))
                       :precondition '((NOT (AT ?TO))
                                       (AT ?FROM))
                       :effect '((NOT (AT ?FROM))
                                 (AT ?TO)))))
    (dolist (ac actions)
      (send domain :add :action ac))
    domain))

(defun make-problem ()
  (let ((problem (instance pddl-problem :init
                           :name 'tidyup-cup
                           :domain 'tidyup-domain)))
    (send problem :objects
          '((START . spot)
            (TABLE . spot)
            (KITCHEN . spot)
            (COUNTER . spot)
            (CUP . item)
            (LARM . arm)
            (RARM . arm)))
    ;;
    (send problem :initial-condition
          '((AT START)
            (ON CUP TABLE)))
    ;;
    (send problem :goal-condition
          '((ON CUP KITCHEN)))
    problem))

(defun make-state-machine (&key (debug))
  (let ((domain (make-domain))
        (problem (make-problem))
        failed-nodes graph insp)
    (setq failed-nodes
          (remove-if-not #'(lambda (n)
                             (string= "_f"
                                      (subseq (send n :name)
                                              (- (length (send n :name)) 2))))
                         (send domain :action)))
    (setq failed-nodes
          (mapcar #'(lambda (n)
                      (read-from-string
                       (subseq (send n :name) 0 (- (length (send n :name)) 2))))
                  failed-nodes))
    (setq graph (pddl-plan-to-graph nil
                                    :domain domain :problem problem
                                    :failed-nodes failed-nodes
                                    :debug debug))
    (when debug
      (send graph :write-to-pdf "/tmp/graph.pdf" nil "hoge")
      (unix:system "xdg-open /tmp/graph.pdf"))
    (setq insp (pddl-graph-to-smach graph :add-self-return-failure-path nil))
    (send insp :state-machine)))

(defaction pick (target arm spot)
  (ros::ros-info "pick ~A ~A ~A" target arm spot)
  t)

(defaction place (target arm spot)
  (ros::ros-info "place ~A ~A ~A" target arm spot)
  t)

(defaction move-to (from to)
  (ros::ros-info "move-to ~A ~A" from to)
  t)
