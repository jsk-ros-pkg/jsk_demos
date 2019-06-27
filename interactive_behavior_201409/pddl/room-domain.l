#!/usr/bin/env roseus
;; room-domain.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :room-domain)

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")

(defun make-domain ()
  (let ((domain (instance pddl-domain :init :name 'room-domain))
        actions)
    (send domain :requirements
          '(:typing :action-costs)
          )
    (send domain :types '(item spot arm person))
    (send domain :constants '(something something-else - item someone - person))
    (send domain :predicates
          '((ON ?OBJ - item ?AT - spot)
            (SURELYON ?OBJ - item ?AT - spot)
            (SEE ?WHO - person ?AT - spot)
            (ONHAND ?OBJ - item ?ARM - arm)
            (UNAVAILABLE ?AT - spot)
            (OCCUPIED ?OBJ - item ?AT - spot)
            (OPEN ?AT - spot)
            (ASKED ?AT - spot)
            (DELIVERED ?OBJ - item ?WHO - person)
            (AT ?AT - spot)))
    (send domain :functions '((total-cost) (distance ?FROM ?TO) (manip-cost ?OBJ)))
    (setq actions
          (list
           (instance pddl-action :init
                     :name "move-to"
                     :parameters '((?FROM ?TO spot))
                     :precondition '((NOT (AT ?TO))
                                     (AT ?FROM))
                     :effect '((NOT (AT ?FROM))
                               (AT ?TO)
                               (increase (total-cost) (distance ?FROM ?TO))))
           (instance pddl-action :init
                     :name "identify-object"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND SOMETHING ?ARM))
                     :effect '((ONHAND ?OBJ ?ARM)
                               (NOT (ONHAND SOMETHING ?ARM))
                               (NOT (ON ?OBJ ?AT))))
           (instance pddl-action :init
                     :name "identify-object_f"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND SOMETHING ?ARM))
                     :effect '((ONHAND SOMETHING-ELSE ?ARM)
                               (NOT (ONHAND SOMETHING ?ARM))))
           (instance pddl-action :init
                     :name "find-object"
                     :parameters '((?OBJ item) (?AT spot))
                     :precondition '((AT ?AT)
                                     (ON ?OBJ ?AT)
                                     (NOT (SURELYON ?OBJ ?AT))
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((SURELYON ?OBJ ?AT)))
           (instance pddl-action :init
                     :name "find-object_f"
                     :parameters '((?OBJ item) (?AT spot))
                     :precondition '((AT ?AT)
                                     (ON ?OBJ ?AT)
                                     (NOT (SURELYON ?OBJ ?AT))
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((NOT (ON ?OBJ ?AT))))
           (instance pddl-action :init
                     :name "pick"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ2 - item)
                                      (not (onhand ?OBJ2 ?ARM)))
                                     (AT ?AT)
                                     (SURELYON ?OBJ ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((ONHAND ?OBJ ?ARM)
                               (NOT (ON ?OBJ ?AT))
                               (NOT (OCCUPIED ?OBJ ?AT))
                               (increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "pick_f"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ2 - item)
                                      (not (onhand ?OBJ2 ?ARM)))
                                     (AT ?AT)
                                     (SURELYON ?OBJ ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "place"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (NOT (UNAVAILABLE ?AT))
                                     (NOT (OCCUPIED ?OBJ ?AT)))
                     :effect '((NOT (ONHAND ?OBJ ?ARM))
                               (ON ?OBJ ?AT)
                               (increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "place_f"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (NOT (OCCUPIED ?OBJ ?AT))
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((increase (total-cost) (manip-cost ?OBJ))
                               (OCCUPIED ?OBJ ?AT)))
           (instance pddl-action :init
                     :name "open-door"
                     :parameters '((?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ - item)
                                      (NOT (ONHAND ?OBJ ?ARM)))
                                     (AT ?AT)
                                     (UNAVAILABLE ?AT))
                     :effect '((NOT (UNAVAILABLE ?AT))))
           (instance pddl-action :init
                     :name "close-door"
                     :parameters '((?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ - item)
                                      (NOT (ONHAND ?OBJ ?ARM)))
                                     (AT ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((UNAVAILABLE ?AT)))
           (instance pddl-action :init
                     :name "deliver"
                     :parameters '((?OBJ item) (?ARM arm) (?WHO person) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (SEE ?WHO ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((NOT (ONHAND ?OBJ ?ARM))
                               (DELIVERED ?OBJ ?WHO)
                               (increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "deliver_f"
                     :parameters '((?OBJ item) (?ARM arm) (?WHO person) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (SEE ?WHO ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((NOT (SEE ?WHO ?AT))))
           (instance pddl-action :init
                     :name "ask"
                     :parameters '((?WHO person) (?AT spot))
                     :precondition '((NOT (ASKED ?AT))
                                     (AT ?AT)
                                     (SEE ?WHO ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((ASKED ?AT)))
           (instance pddl-action :init
                     :name "ask_f"
                     :parameters '((?WHO person) (?AT spot))
                     :precondition '((NOT (ASKED ?AT))
                                     (AT ?AT)
                                     (SEE ?WHO ?AT)
                                     (NOT (UNAVAILABLE ?AT)))
                     :effect '((NOT (SEE ?WHO ?AT))))
           ))
    (dolist (ac actions)
      (send domain :add :action ac))
    domain))

(defun distance (from to)
  (norm (v- from to)))

(defun make-spot-symbols (syms &key (test-func #'distance))
  "syms = alist of (sym . pos)"
  (let ((objs (mapcar #'car syms))
        inits goals)
    (dolist (from objs)
      (dolist (to objs)
        (unless (eq from to)
          (push `(= (distance ,from ,to) ,(floor (funcall test-func
                                               (cadr (assoc from syms))
                                               (cadr (assoc to syms)))))
                inits))))
    ;; NOTE: downward does not allow partial cost function definition
    ;;       so we must add distance for the same start/goal
    (dolist (obj objs)
      (push `(= (distance ,obj ,obj) 100000000000) inits))
    ;;
    (dolist (s syms)
      (when (memq :unavailable (cddr s))
        (push `(unavailable ,(car s)) inits)
        (push `(unavailable ,(car s)) goals)))
    ;;
    (list (cons :objects (mapcar #'(lambda (s) (cons s 'spot)) objs))
          (cons :initial-condition inits)
          (cons :goal-condition goals))))

(defun make-object-symbols (syms)
  (list (cons :objects (mapcar #'(lambda (s) (cons s 'item)) syms))
        (cons :initial-condition
              (mapcar #'(lambda (s)
                          `(= (manip-cost ,s) 1))
                      syms))))

(defun make-problem (&key init goal items people)
  (let ((problem (instance pddl-problem :init
                           :name 'house-keeping
                           :domain 'room-domain))
        (spots (make-spot-symbols
                     `((START ,(float-vector 0 0 0))
                       (TABLE ,(float-vector 0 5 0))
                       (FRIDGE ,(float-vector 10 5 0) :unavailable)
                       (DISHWASHER ,(float-vector 0 10 0) :unavailable)
                       (KITCHEN ,(float-vector 10 0 0))
                       (room73a2 ,(float-vector -10 -10 0) :unavailable)
                       (COUNTER ,(float-vector 5 0 0)))))
        (objs (make-object-symbols items)))
    (send problem :objects
          (append (mapcar #'(lambda (x) `(,x . person)) people)
                  (append (cdr (assoc :objects objs))
                          (append (cdr (assoc :objects spots))
                                  '((LARM . arm)
                                    (RARM . arm))))))
    ;;
    (send problem :initial-condition
          (append (cdr (assoc :initial-condition objs))
                  (append (cdr (assoc :initial-condition spots)) init)))
    (send problem :initial-condition
          (append (send problem :initial-condition)
                  '((= (manip-cost SOMETHING) 1)
                    (= (manip-cost SOMETHING-ELSE) 1))))
    ;;
    (send problem :goal-condition
          (append (cdr (assoc :goal-condition objs))
                  (append (cdr (assoc :goal-condition spots)) goal)))
    ;;
    problem))
