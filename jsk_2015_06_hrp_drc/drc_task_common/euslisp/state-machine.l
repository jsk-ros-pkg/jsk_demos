;; state-machine.l
;; this file provides a simple wrapper of roseus_smach/src/state-machine.l
;; 
;; Goal of this file is to provide simple state machine.
;; The state machine is used to control state of UI, communication and so on.
;; Each state has one additional state called 'context'.
;; Several states might be belongs to one context.
;; State is not a subgraph of context because graph is only composed from
;; connection of state and we do not consider connection between contexts.
(require :state-machine "package://roseus_smach/src/state-machine.l")

(defclass task-state-machine
  :super state-machine
  :slots (transitions))

(defmethod task-state-machine
  (:context ()
    "return current context"
    (send (send self :active-state) :context))
  (:state ()
    "return current state"
    (send (send self :active-state) :name))
  (:next? (next-state)
    "return t if state machine can go to next-state from the current state"
    (let ((candidate-transitions (send self :lookup-transitions active-state)))
      (not (null (remove-if-not #'(lambda (trans)
                                    (eq next-state
                                        (send (send trans :to) :name)))
                                candidate-transitions)))))
  (:go-to (next-state)
    (send self :active-state (send self :node next-state)))
  (:lookup-transitions (start-state)
    (remove-if-not #'(lambda (trans)
                       (eq (send trans :from) start-state))
                   transitions))
  (:register-transition (trans)
    (setq transitions (cons trans transitions)))
  (:transitions ()
    transitions)
  )

(defclass task-state
  :super state
  :slots (context))

(defmethod task-state
  (:init (name acontext)
    (send-super :init name nil)
    (setq context acontext)
    ;; print
    (warning-message 4 "state ~A [~A]~%" name acontext)
    self)
  (:context () context)
  )

(defun make-state-machine (graph-list context-map initial-state)
  "
Utility function to make state machine.

Usage:
  (make-state-machine '((a -> b)
                        (b -> c)
                        (c -> d))
                      '((a task0)
                        (b task1)
                        (c task1)
                        (d taask3))
                       'a)
"
  (let ((sm (instance task-state-machine :init)))
    ;; list up all the states
    (let ((all-states (unique
                       (flatten
                        (mapcar #'(lambda (connection)
                                    (append (list (car connection))
                                            (list (caddr connection))))
                                graph-list)))))
      (warning-message 2 "~A states are defined~%" (length all-states))
      (dolist (state-name all-states)
        ;; lookup context
        (let ((context (cadr
                        (find state-name context-map
                              :test #'(lambda (name context)
                                        (eq name (car context)))))))
          (unless context
            (warning-message 2 "Cannot find context for ~A~%" state-name)
            (error))
          (send sm :add-node (instance task-state :init state-name context)))))
    ;; register transition
    (dolist (connection graph-list)
      (send sm :register-transition
            (send sm :add-transition
                  (send sm :node (car connection))
                  (send sm :node (caddr connection)) t))
      (warning-message 2 "connection ~A => ~A~%"
                       (car connection) (caddr connection)))
    (send sm :start-state (send sm :node initial-state))
    (send sm :active-state (send sm :start-state))
    sm))
      
