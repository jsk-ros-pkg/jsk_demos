(load "package://pddl_planner/src/eus-pddl-client.l")
(load "package://jsk_2013_04_pr2_610/pddl/description.l")
(load "package://jsk_2013_04_pr2_610/euslisp/setup-for-pddl.l")
(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")

(defmacro plan-to-condition (goal-condition)
  `(let ((problem (copy-object *problem*)))
	 (send problem :goal-condition ,goal-condition)
	 (exec-plan problem)
	 ))

(defun exec-plan (problem)
  (let ((result (solve-pddl-planning *domain* problem)))
  (format t "plan= ")
  (pprint (cdr (assoc :plan result)))

  ;; send plan to pr2
  (setup)
  (pr2-reset-pose)
  (dolist (order (cdr (assoc :plan result)))
	(eval order))
  (speak-jp "おわりました")
  )
)

;; -------------------------------------------

(defun plan-all ()
  (plan-to-condition
   '((clean room)
	 (clean-under table)
	 (on tray sink)
	 (under chair table)
	 (on broom laundry)
	 (in cloth laundry)
	 (working laundry)
	 )
   ))

(defun plan-tray ()
  (plan-to-condition
   '((on tray sink))
   )
  )

(defun plan-wash ()
  (plan-to-condition
   '((in cloth laundry)
	 (working laundry)
	 )
   ))

(defun plan-sweep ()
  (plan-to-condition
   '((clean room)
	 (clean-under table)
	 (under chair table)
	 (on broom laundry)
	 )
   ))
