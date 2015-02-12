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
  :slots (transitions output-image-file initial-hook-obj initial-hook-method))

(defmethod task-state-machine
  (:init (&optional output)
    (setq output-image-file output)
    self)
  (:context ()
    "return current context"
    (send (send self :active-state) :context))
  (:state ()
    "return current state"
    (send (send self :active-state) :name))
  (:register-initial-hook
    (obj method)
    (setq initial-hook-obj obj)
    (setq initial-hook-method method)
    )
  (:next? (next-state)
    "return t if state machine can go to next-state from the current state"
    (let ((candidate-transitions (send self :lookup-transitions active-state)))
      (not (null (remove-if-not #'(lambda (trans)
                                    (eq next-state
                                        (send (send trans :to) :name)))
                                candidate-transitions)))))
  (:go-to (next-state)
    (format t "next state: ~A~%" next-state)
    (send self :active-state (send self :node next-state))
    (if output-image-file
        (send self :write-to-png output-image-file))
    (if (eq (send self :active-state) (send self :start-state))
        (if (and initial-hook-obj initial-hook-method)
            (send initial-hook-obj initial-hook-method)))
    )
  (:lookup-transitions (start-state)
    (remove-if-not #'(lambda (trans)
                       (eq (send trans :from) start-state))
                   transitions))
  (:register-transition (trans)
    (setq transitions (cons trans transitions)))
  (:transitions ()
    transitions)
  (:write-to-png (fname &optional result-path
                        (title (string-right-trim ".png" fname)))
    (ros::ros-info "writing to ~A" fname)
    (let ((dot-fname
           (format nil "~A.dot" (string-right-trim ".png" fname))))
      (send self :write-to-dot dot-fname result-path title)
      (unix:system (format nil "dot ~A -Tpng -o ~A" dot-fname fname))
      t))
  (:write-to-dot (fname &optional result-path (title "output"))
    (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
           (mapcar #'(lambda (n)
                       (cons n (string (gensym))))
                   nodes)))
      (labels ((graphviz-node-name
                (n)
                (cdr (assoc n node-alist))))
      (with-open-file (f fname :direction :output)
        (format f "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (format f "   ")
          (if (eq target-node (send self :active-state))
              (format f "~A [label = \"~s\", color = red];~%"
                      (graphviz-node-name target-node)
                      (send target-node :name))
            ;; hoge = "hoge";
            (format f "~A [label = \"~s\"];~%"
                    (graphviz-node-name target-node)
                    (send target-node :name))
            ))
        ;; first of all, write result-path
        (let ((drawed-arc nil))
          (let (anode)
            (while (and (setq anode (pop result-path)) result-path)
              (format f "   ")
              (format f "~A -> ~A [color = red];~%"
                      (graphviz-node-name (send anode :state))
                      (graphviz-node-name (send (car result-path) :state)))))
          (dolist (target-node nodes)
            (let ((neighbors
                   (mapcar #'cdr (send self :successors target-node))))
              (dolist (neighbor neighbors)
                (when (not (or (find (cons target-node neighbor) drawed-arc
                                     :test #'equal)
                               (find (cons neighbor target-node) drawed-arc
                                     :test #'equal)))
                    (if (member target-node
                              (mapcar #'cdr (send self :successors neighbor)))
                      (progn
                        (format f "   ")
                        (format f "~A -> ~A [dir = both];~%"
                                (graphviz-node-name target-node)
                                (graphviz-node-name neighbor)))
                    (progn
                      (format f "   ")
                      (format f "~A -> ~A;~%"
                              (graphviz-node-name target-node)
                              (graphviz-node-name neighbor))))
                  (push (cons target-node neighbor) drawed-arc)))))
          (format f "}~%")
          t)))))
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

(defun make-state-machine (graph-list context-map initial-state &optional output-file)
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
  (let ((sm (instance task-state-machine :init output-file)))
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
      

(defun make-ocs-state-machine ()
  (make-state-machine
   '(                                           
     ;; go-pos
     (:initial -> :selecting-region-for-go-pos) ; click go-pos
     (:selecting-region-for-go-pos -> :initial) ; cancel
     (:selecting-region-for-go-pos -> :recognizing-go-pos-goal)
     ;;(:recognizing-go-pos-goal -> :initial) ; cancel
     (:recognizing-go-pos-goal -> :selecting-exec-for-go-pos) ; recognition done
     (:selecting-exec-for-go-pos -> :executing-go-pos)        ; exec
     (:selecting-exec-for-go-pos -> :initial)                 ; cancel
     (:initial -> :executing-go-pos)    ; from rviz interface
     (:executing-go-pos -> :initial)    ; go pos done
     ;; look-at
     (:initial -> :selecting-region-for-look-at)
     (:selecting-region-for-look-at -> :initial) ;cancel
     (:selecting-region-for-look-at -> :recognizing-look-at-point)
     (:recognizing-look-at-point -> :selecting-exec-for-look-at) ;recognition done
     (:selecting-exec-for-look-at -> :executing-look-at)          ;exec
     (:selecting-exec-for-look-at -> :initial)                   ;cancel
     (:executing-look-at -> :initial)                            ;done
     )
   '((:initial :initial)
     ;; go-pos
     (:selecting-exec-for-go-pos :go-pos)
     (:selecting-region-for-go-pos :go-pos)
     (:recognizing-go-pos-goal :go-pos)
     (:executing-go-pos :go-pos)
     ;; look-at
     (:selecting-region-for-look-at :look-at)
     (:recognizing-look-at-point :look-at)
     (:selecting-exec-for-look-at :look-at)
     (:executing-look-at :look-at)
     )
   :initial "ocs_state.png"))
