#!/usr/bin/env roseus
;; tidyup-domain.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :actionlib "package://interactive_behavior_201409/euslisp/actionlib.l")


(defun make-tidyup-domain ()
  (let ((domain (instance pddl-domain :init :name 'tidyup-domain))
        actions)
    (send domain :requirements '(:typing :action-costs))
    (send domain :types '(item spot arm))
    (send domain :predicates
          '((ON ?OBJ - item ?SPOT - spot)
            (ONHAND ?OBJ - item ?ARM - arm)
            (UNAVAILABLE ?OBJ - item ?AT - spot)
            (AT ?SPOT - spot)))
    (send domain :functions '((total-cost) (distance ?FROM ?TO) (manip-cost ?OBJ)))
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
                               (NOT (ON ?OBJ ?AT))
                               (increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "pick_f"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((forall (?OBJ2 - item)
                                      (not (onhand ?OBJ2 ?ARM)))
                                     (AT ?AT)
                                     (ON ?OBJ ?AT))
                     :effect '((increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "place"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (NOT (UNAVAILABLE ?OBJ ?AT)))
                     :effect '((NOT (ONHAND ?OBJ ?ARM))
                               (ON ?OBJ ?AT)
                               (increase (total-cost) (manip-cost ?OBJ))))
           (instance pddl-action :init
                     :name "place_f"
                     :parameters '((?OBJ item) (?ARM arm) (?AT spot))
                     :precondition '((ONHAND ?OBJ ?ARM)
                                     (AT ?AT)
                                     (NOT (UNAVAILABLE ?OBJ ?AT)))
                     :effect '((increase (total-cost) (manip-cost ?OBJ))
                               (UNAVAILABLE ?OBJ ?AT)))
           (instance pddl-action :init
                     :name "move-to"
                     :parameters '((?FROM ?TO spot))
                     :precondition '((NOT (AT ?TO))
                                     (AT ?FROM))
                     :effect '((NOT (AT ?FROM))
                               (AT ?TO)
                               (increase (total-cost) (distance ?FROM ?TO))))
           ))
    (dolist (ac actions)
      (send domain :add :action ac))
    domain))

(defun distance (from to)
  (norm (v- from to)))

(defun make-spot-symbols (syms &key (test-func #'distance))
  "syms = alist of (sym . pos)"
  (let ((objs (mapcar #'car syms))
        dists)
    (dolist (from objs)
      (dolist (to objs)
        (unless (eq from to)
          (push `(= (distance ,from ,to) ,(floor (funcall test-func
                                               (cdr (assoc from syms))
                                               (cdr (assoc to syms)))))
                dists))))
    ;; NOTE: downward does not allow partial cost function definition
    ;;       so we must add distance for the same start/goal
    (dolist (obj objs)
      (push `(= (distance ,obj ,obj) 100000000000) dists))
    ;;
    (list (cons :objects (mapcar #'(lambda (s) (cons s 'spot)) objs))
          (cons :condition dists))))

(defun make-object-symbols (syms)
  (list (cons :objects (mapcar #'(lambda (s) (cons s 'item)) syms))
        (cons :condition
              (mapcar #'(lambda (s)
                          `(= (manip-cost ,s) 1))
                      syms))))

(defun make-problem (&key init goal)
  (let ((problem (instance pddl-problem :init
                           :name 'tidyup-cup
                           :domain 'tidyup-domain))
        (spots (make-spot-symbols
                     `((START . ,(float-vector 0 0 0))
                       (TABLE . ,(float-vector 0 5 0))
                       (KITCHEN . ,(float-vector 10 0 0))
                       (COUNTER . ,(float-vector 5 0 0)))))
        (objs (make-object-symbols
               '(CUP BOTTLE))))
    (send problem :objects
          (append (cdr (assoc :objects objs))
                  (append (cdr (assoc :objects spots))
                          '((LARM . arm)
                            (RARM . arm)))))
    ;;
    (send problem :initial-condition
          (append (cdr (assoc :condition objs))
                  (append (cdr (assoc :condition spots)) init)))
    ;;
    (send problem :goal-condition goal)
    ;;
    problem))

(provide :tidyup-domain)
