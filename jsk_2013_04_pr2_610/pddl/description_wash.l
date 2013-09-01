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
      '((tray   . item)
        (cloth  . item)
        (broom  . item)

        (chair-back-spot . spot)
        (chair . item)
        (table-front-spot . spot)
        (table-side-spot . spot)
        (sink-spot  . spot)
        (laundry . item)
        (broom-spot . spot)
        (laundry-spot . spot)

        (room . spot)
        (world . spot)
        ))
                                        ;; 簡単のため使う型は一つのみに
                                        ;; disk0,1,2 pole0,1,2すべてobject型

;; initial condition
(send *problem* :initial-condition
      '((on tray  table-side-spot)
        (on cloth table-front-spot)
        (on broom broom-spot)
        (under chair table-front-spot)
        (at world)
        (not (in cloth laundry))
        (not (open-door laundry))
;        (= (total-cost) 0)
;        (= (move-to) 10)
;        (= (sweep) 100)
        ))

;; goal condition
(send *problem* :goal-condition
      '(
;        (clean room)
;        (clean-under table-front-spot)
;        (on tray sink-spot)
;        (under chair table-front-spot)
;        (on broom broom-spot)
        (at laundry-spot)
        (in cloth laundry)
        (not (open-door laundry))
        (working laundry)
        ))

;(send *problem* :metric '(minimize (total-cost)))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'manip))
(send *domain* :requirements '(:typing))
(send *domain* :types '(item spot))
(send *domain* :predicates '((on ?obj - item ?from - spot)
                             (at ?from - spot)
                             (in ?obj0 ?obj1 - item)
                             (hold ?obj - item)
                             (clean ?room - spot)
                             (clean-under ?obj - spot)
                             (space ?room - spot)
                             (open-door ?obj - item)
                             (need-closed-door ?obj - item)
                             (under ?obj - item ?space - spot)
                             (working ?obj - item)
                             ))
#|
(send *domain* :functions '((total-cost)
                            (move-to)
                            (sweep)
							))
|#

;;making action
(setq *action-list* (list
(setq *move* (instance pddl-action :init
                       :name "move-to"
                       :parameters '((?to spot))
                       :precondition '((not (at ?to)))
                       :effect '((forall (?any - spot) (when (not (= ?any ?to))(not (at ?any))))
                                 (at ?to)
;                                 (increase (total-cost) (move-to))
                                 )))

(setq *pick* (instance pddl-action :init
                       :name "pick"
                       :parameters '((?obj item) (?from spot))
                       :precondition '((at ?from)
                                       (on ?obj ?from)
                                       (forall (?any - item) (not (hold ?any))))
                       :effect '((not (on ?obj ?from))
                                 (hold ?obj))))

(setq *pick-f* (instance pddl-action :init
                       :name "pick_f"
                       :parameters '((?obj item) (?from spot))
                       :precondition '((at ?from)
                                       (on ?obj ?from)
                                       (forall (?any - item) (not (hold ?any))))
                       :effect '((not (hold ?obj)))))

(setq *place* (instance pddl-action :init
                       :name "place"
                       :parameters '((?obj item) (?to spot))
                       :precondition '((at ?to)
                                       (hold ?obj))
                       :effect '((not (hold ?obj))
                                 (on ?obj ?to))))

(setq *put* (instance pddl-action :init
                       :name "put"
                       :parameters '((?obj item) (?to item) (?at spot))
                       :precondition '((at ?at)
                                       (hold ?obj)
                                       (open-door ?to)
                                       (not (need-closed-door ?to))
                                       )
                       :effect '((not (hold ?obj))
                                 (in ?obj ?to))))

(setq *push-button* (instance pddl-action :init
                              :name "push-button"
                              :parameters '((?obj item) (?at spot))
                              :precondition '((at ?at)
                                              (in cloth ?obj)
                                              (not (open-door ?obj))
                                              (not (need-closed-door ?obj))
                                              )
                              :effect '((working ?obj))))

(setq *open-door* (instance pddl-action :init
                       :name "open-door"
                       :parameters '((?obj item) (?at spot))
                       :precondition '((at ?at)
                                       (not (open-door ?obj))
                                       (not (need-closed-door ?obj))
                                       )
                       :effect '((open-door ?obj)
                                 )))

(setq *more-open-door* (instance pddl-action :init
                       :name "more-open-door"
                       :parameters '((?obj item) (?at spot))
                       :precondition '((at ?at)
                                       (not (open-door ?obj))
                                       (need-closed-door ?obj)
                                       )
                       :effect '((open-door ?obj)
                                 (not (need-closed-door ?obj))
                                 )))


(setq *open-door-f* (instance pddl-action :init
                       :name "open-door_f"
                       :parameters '((?obj item) (?at spot))
                       :precondition '((at ?at)
                                       (not (open-door ?obj))
                                       (not (need-closed-door ?obj))
                                       )
                       :effect '((need-closed-door ?obj))))

(setq *close* (instance pddl-action :init
                       :name "close"
                       :parameters '((?obj item) (?at spot))
                       :precondition '((at ?at)
                                       (open-door ?obj)
                                       )
                       :effect '((not (open-door ?obj))
                                 (not (need-closed-door ?obj)))))

(setq *close-f* (instance pddl-action :init
                       :name "close_f"
                       :parameters '((?obj item) (?at spot))
                       :precondition '((at ?at)
                                       (open-door ?obj)
                                       )
                       :effect '((need-closed-door ?obj)
                                 )))

(setq *sweep* (instance pddl-action :init
                       :name "sweep"
                       :parameters '((?space spot))
                       :precondition '((hold broom)
                                       (under chair table-front-spot))
                       :effect '((clean ?space)
;                                 (increase (total-cost) (sweep))
								 )))

(setq *sweep-under* (instance pddl-action :init
                       :name "sweep-under"
                       :parameters '((?space spot))
                       :precondition '((hold broom)
                                       (space ?space)
                                       (at ?space))
                       :effect '((clean-under ?space))))

(setq *pull* (instance pddl-action :init
                       :name "pull-chair"
                       :parameters '((?from spot))
                       :precondition '((under chair ?from) ; (under ?obj ?from)
                                       (at table-front-spot))
                       :effect '((not (under chair ?from))
                                 (space ?from)
                                 (at chair-back-spot))))

(setq *push* (instance pddl-action :init
                       :name "push-chair"
                       :parameters '((?to spot))
                       :precondition '((not (under chair ?to))
                                       (at chair-back-spot))
                       :effect '((under chair ?to) ; (under ?obj ?to)
                                 (not (space ?to))
                                 (at table-front-spot))))
))



;;add action to domain
#|
(send *domain* :add :action *move*)
(send *domain* :add :action *pick*)
(send *domain* :add :action *pick-f*)
(send *domain* :add :action *place*)
(send *domain* :add :action *sweep*)
(send *domain* :add :action *sweep-under*)
(send *domain* :add :action *put*)
(send *domain* :add :action *open-door*)
(send *domain* :add :action *open-door-f*)
(send *domain* :add :action *close*)
(send *domain* :add :action *close-f*)
(send *domain* :add :action *pull*)
(send *domain* :add :action *push*)
(send *domain* :add :action *push-button*)
|#
(dolist (ac *action-list*)
  (send *domain* :add :action ac))


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
  (setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
  (setq act-names (mapcar #'string-upcase
                          (send-all (send *domain* :action) :name)))
  (setq act-names
        (mapcan
         #'(lambda(s)
             (if (find (concatenate string s "_F") act-names :test #'string=)
                 (list (intern s))))
         act-names))
  (when act-names
    (setq *graph* (add-failed-nodes *result* act-names :graph *graph*)))

  (send *graph* :name (send *domain* :name))

  (send (make-readable-graph *graph*) :write-to-pdf "pddl_graph.pdf"))
#|
;(format t "result = ~A " *result*)
(ros::exit)
|#

(defun debug-make-smach ()
  ;; convert to smach
  (load "package://roseus_smach/src/state-machine-ros.l")
  (load "package://roseus_smach/src/state-machine-utils.l")
  (load "package://roseus_smach/src/pddl2smach.l")

  (defun move-to (&rest args)
	(warn "move-to ~A~%" args)
	t)

  (defun pick (&rest args)
	(warn "pick ~A~%" args)
	(if (> (random 1.0) 0.3) nil t))
 
  (defun sweep (&rest args)
	(warn "sweep ~A~%" args)
	t)

  (defun open-door (&rest args)
	(warn "open-door ~A~%" args)
	(if (> (random 1.0) 0.3) nil t))

  (defun close (&rest args)
	(warn "close ~A~%" args)
	(if (> (random 1.0) 0.3) nil t))

  (defun more-open-door (&rest args)
    (warn "more-open-door ~A~%" args)
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
	(ros::exit))
  )

(defun debug-all ()
;  (load "package://jsk_2013_04_pr2_610/pddl/description_wash.l")
  (debug-solve)
  (debug-make-graph)
  (debug-make-smach)
)