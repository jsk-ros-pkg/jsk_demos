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
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l") ;; for smach visualize only

(defclass task-state-machine
  :super state-machine
  :slots (transitions contexts output-image-file initial-hook-obj initial-hook-method smach-visualizer))

(defmethod task-state-machine
  (:init (&optional output smach-visualize-p)
    (setq output-image-file output)
    (when smach-visualize-p
      (setq smach-visualizer (instance state-machine-inspector :init nil)))
    self)
  (:nodes-from-context (context)
    (remove-if-not #'(lambda (n)
                       (eq (send n :context) context))
                   nodes))
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
  (:state? (st)
    (eq (send self :state) st))
  (:next? (next-state)
    "return t if state machine can go to next-state from the current state"
    (or (eq (send self :state) next-state)
        (let ((candidate-transitions (send self :lookup-transitions (send self :active-state))))
          (not (null (remove-if-not #'(lambda (trans)
                                        (eq next-state
                                            (send (send trans :to) :name)))
                                    candidate-transitions))))))
  (:go-to! (next-state)
    (format t "next state: ~A~%" next-state)
    (send self :active-state (send self :node next-state))
    (if smach-visualizer
        (send smach-visualizer :publish-status self))
    (if output-image-file
        (send self :write-to-png output-image-file))
    (if (eq (send self :active-state) (send self :start-state))
        (if (and initial-hook-obj initial-hook-method)
            (send initial-hook-obj initial-hook-method))))
  (:go-to (next-state)
    (if (send self :next? next-state)
        (send self :go-to! next-state)
      (ros::ros-warn "Cannot change state from ~A to ~A"
             (send self :state) next-state)))
  (:lookup-transitions (start-state &key (direction :from))
    (remove-if-not #'(lambda (trans)
                       (eq (send trans direction) start-state))
                   transitions))
  (:register-transition (trans)
    (setq transitions (cons trans transitions))
    (setq contexts (union contexts
                          (list (send (send trans :from) :context)
                                (send (send trans :to) :context)))))
  (:transitions ()
    transitions)
  (:contexts ()
    contexts)
  (:write-to-png (fname &optional result-path
                        (title (string-right-trim ".png" fname)))
    (ros::ros-info "writing to ~A" fname)
    (let ((dot-fname
           (format nil "~A.dot" (string-right-trim ".png" fname))))
      (send self :write-to-dot dot-fname result-path title)
      (unix:system (format nil "dot ~A -Tpng -o ~A" dot-fname fname))
      t))
  (:write-to-dot-org (&rest args)
    (send-super* :write-to-dot args))
  (:write-to-dot (fname &optional result-path (title "output"))
    ;; list up all the nodes now we are interested in
    (let* ((initial-nodes (send self :nodes-from-context (send self :context)))
           ;; extend nodes which can be arrived at from initial-nodes
           (next-nodes (flatten (mapcar #'(lambda (n)
                                            (mapcar #'cdr (send n :successors)))
                                        initial-nodes)))
           (target-nodes (unique (append initial-nodes next-nodes))))
      (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
             (mapcar #'(lambda (n)
                         (cons n (string (gensym))))
                     target-nodes)))
        (labels ((graphviz-node-name
                  (n)
                  (cdr (assoc n node-alist))))
          (with-open-file (f fname :direction :output)
            (format f "digraph ~A {~%" title)
            (dolist (target-node target-nodes)
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
            (let ((drawed-arc nil))
              (dolist (target-node target-nodes)
                (let ((neighbors
                       (mapcar #'cdr (send self :successors target-node))))
                  (dolist (neighbor neighbors)
                    (when (and (not (or (find (cons target-node neighbor) drawed-arc
                                              :test #'equal)
                                        (find (cons neighbor target-node) drawed-arc
                                              :test #'equal)))
                               (member neighbor target-nodes))
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
              t))))))
  (:write-to-dot-full (fname &optional result-path (title "output"))
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
  (:reset-state ()
   (send-super :reset-state)
   (when smach-visualizer
     (unix:sleep 1) ;; omajinai
     (send smach-visualizer :publish-structure self)
     (send smach-visualizer :publish-status self))
   self)
  )

(defmethod state-machine-inspector
  (:publish-status-org
   (&optional (userdata nil) (machine sm) (path (format nil "/~A" root-name)))
   (send-super* self userdata machine path))
  (:publish-status
   (machine &optional only-context (path (format nil "/~A" root-name)))
   (let* ((context (or only-context (send (send machine :start-state) :context)))
          (msg (instance smach_msgs::SmachContainerStatus :init))
          (node-names (send-all (send machine :nodes-from-context context) :name))
          active initial)
     (send msg :header :seq (incf state-counter))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (if only-context
         (dolist (tr (send machine :transitions))
           (when (and (eq only-context (send (send tr :to) :context))
                      (not (eq only-context (send (send tr :from) :context))))
             (push (send (send tr :to) :name) initial)
             (return)))
       (push (send (send machine :active-state) :name) initial))
     (send msg :initial_states (mapcar #'string initial))
     (cond
      ((eq (send (send machine :active-state) :context) context)
       (setq active (send (send machine :active-state) :name)))
      ((eq (send (send machine :active-state) :context) :initial)
       (setq active :initial)))
     (when active
       (send msg :active_states (list (string active))))
     (send msg :local_data (pickle::dump nil))
     (when (eq only-context (send (send machine :active-state) :context))
       (send msg :info path))
     (when (eq (send (send machine :active-state) :context) :initial)
       (send msg :info (format nil "/~A" root-name)))
     (ros::publish (concatenate string srv-name "/smach/container_status") msg)
     ;; recursive call for publish structure for subgraphs
     (unless only-context
       (let ((contexts-w/o-current-context
              (remove context (send machine :contexts))))
         (dolist (ctx contexts-w/o-current-context)
           (send self :publish-status machine ctx
                 (format nil "~A/~A" path (string ctx))))))))
  (:publish-structure-org
   (&optional (machine sm) (path (format nil "/~A" root-name)))
   (send-super* self :publish-structure machine path))
  (:publish-structure
   (machine &optional only-context (path (format nil "/~A" root-name)))
   (let* ((context (or only-context (send (send machine :start-state) :context)))
          (msg (instance smach_msgs::SmachContainerStructure :init))
          (nodes (send machine :nodes-from-context context))
          transitions from-nodes to-nodes)
     (send msg :header :seq (incf structure-counter))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (dolist (n nodes)
       (dolist (tr (send n :arc-list))
         (push (string (send tr :name)) transitions)
         (push (send (send tr :from) :name) from-nodes)
         (if (eq (send (send tr :to) :context) context)
             (push (send (send tr :to) :name) to-nodes)
           (push (send (send tr :to) :context) to-nodes))))
     (send msg :internal_outcomes transitions)
     (send msg :outcomes_from (mapcar #'string from-nodes))
     (send msg :outcomes_to (mapcar #'string to-nodes))
     (send msg :children (mapcar #'string
                                 (union
                                  (union from-nodes to-nodes)
                                  (send-all nodes :name))))
     (ros::publish (concatenate string srv-name "/smach/container_structure") msg)
     ;; recursive call for publish structure for subgraphs
     (unless only-context
       (let ((contexts-w/o-current-context
              (remove context (send machine :contexts))))
         (dolist (ctx contexts-w/o-current-context)
           (send self :publish-structure machine ctx
                 (format nil "~A/~A" path (string ctx)))))))))

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

;; timered state machine
;; timered-state-machine and timered-node provide
;; state machine with time limit.
;; You can spcify following attributes to state:
;;   1. time limit
;;   2. next state when time limit has come
;;   3. hook method and object to be called when time limit has come
(defclass timered-state-machine
  :super task-state-machine
  :slots ())

(defmethod timered-state-machine
  (:proc ()
    "check current state has timelimit and if it has timelimit,
state machine automatticaly transition to next state"
    (let ((current-state (send self :active-state)))
      (when (send current-state :has-timelimit-p)
        (when (send current-state :check-timelimit)
          (ros::ros-info "automatically transition to ~A"
            (send current-state :timelimit-next-state))
          (send current-state :call-hook)
          (send self :go-to (send current-state :timelimit-next-state))
          )))
    )
  (:go-to (next)
    "call :enter hook method"
    (send-super :go-to next)
    (send (send self :node next) :enter))
  )

(defclass timered-state
  :super task-state
  :slots (timelimit deligate-object deligate-method
          timelimit-next-state started-time end-time))

(defmethod timered-state
  (:timelimit-next-state ()
    timelimit-next-state)
  (:add-timelimit (timelimit-duration next-state
                   &key
                   ((:deligate-object adeligate-object) nil)
                   ((:deligate-method adeligate-method) nil))
    "add timelimit to state"
    (setq timelimit timelimit-duration)
    (setq timelimit-next-state next-state)
    (if (and adeligate-object adeligate-method)
        (progn
          (setq deligate-object adeligate-object)
          (setq deligate-method adeligate-method))))
  (:has-timelimit-p ()
    "return t if state has timelimit"
    (not (null timelimit)))
  (:enter ()
    "should be called when this state is activated"
    (ros::ros-info "entering ~A" (send self :name))
    (setq started-time (ros::time-now))
    (if timelimit
        (setq end-time (ros::time+ started-time (ros::time timelimit))))
    )
  (:check-timelimit ()
    "return t if timelimit has come"
    (ros::time> (ros::time-now) end-time))
  (:call-hook ()
    "call hook method if deligate-object and deligate-method are
specified"
    (if (and deligate-object deligate-method)
        (send deligate-object deligate-method))
    )
  )
  
#|
;; sample of timered-state-machine
(load "state-machine.l")
(setq white-rabbit-state (make-state-machine
                          '((:initial -> :tea-party)
                            (:tea-party -> :duchess)
                            (:duchess -> :heart-trial)
                            (:heart-trial -> :initial))
                          '((:initial :alice-in-wonderland)
                            (:tea-party :alice-in-wonderland)
                            (:duchess :alice-in-wonderland)
                            (:heart-trial :alice-in-wonderland))
                          :initial nil))
(send (send white-rabbit-state :node :initial)
      :add-timelimit 10 :tea-party)
(send (send white-rabbit-state :node :tea-party)
      :add-timelimit 10 :duchess)
(send (send white-rabbit-state :node :duchess)
      :add-timelimit 10 :heart-trial)
(send (send white-rabbit-state :node :heart-trial)
      :add-timelimit 10 :initial)
(send white-rabbit-state :go-to :initial)
(do-until-key
   (send white-rabbit-state :proc))
|#


(defun make-state-machine (graph-list context-map initial-state &optional output-file smach-visualize-p)
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
  (let ((sm (instance timered-state-machine :init output-file smach-visualize-p)))
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
          (send sm :add-node (instance timered-state
                                       :init state-name context)))))
    ;; register transition
    (dolist (connection graph-list)
      (send sm :register-transition
            (send sm :add-transition
                  (send sm :node (car connection))
                  (send sm :node (caddr connection)) t))
      (warning-message 2 "connection ~A => ~A~%"
                       (car connection) (caddr connection)))
    (send sm :start-state (send sm :node initial-state))
    (send sm :reset-state)
    sm))


(defun make-ocs-state-machine ()
  (make-state-machine
   '(                                           
     ;; go-pos
     (:initial -> :selecting-region-for-go-pos) ; click go-pos
     (:selecting-region-for-go-pos -> :initial) ; cancel
     (:selecting-region-for-go-pos -> :recognizing-go-pos-goal)
     (:recognizing-go-pos-goal -> :initial) ; cancel
     (:recognizing-go-pos-goal -> :selecting-exec-for-go-pos) ; recognition done
     (:selecting-exec-for-go-pos -> :executing-go-pos)        ; exec
     (:selecting-exec-for-go-pos -> :initial)                 ; cancel
     (:initial -> :executing-go-pos)    ; from rviz interface
     (:executing-go-pos -> :initial)    ; go pos done
     ;; look-at
     (:initial -> :selecting-region-for-look-at)
     (:selecting-region-for-look-at -> :initial) ;cancel
     (:selecting-region-for-look-at -> :executing-look-at)
     (:executing-look-at -> :initial)                            ;done
     ;; look-around
     (:initial -> :executing-look-around)
     (:executing-look-around -> :initial)                            ;done
     ;; valve
     (:initial -> :selecting-region-for-valve)
     (:selecting-region-for-valve -> :initial) ;cancel
     (:selecting-region-for-valve -> :recognizing-valve-pose)
     (:recognizing-valve-pose -> :selecting-exec-for-valve) ;recognition done
     (:recognizing-valve-pose -> :initial) ;cancel
     (:selecting-exec-for-valve -> :executing-valve)          ;exec
     (:executing-valve -> :selecting-exec-for-valve)          ;cancel
     (:selecting-exec-for-valve -> :initial)                   ;cancel
     (:executing-valve -> :initial)                            ;done

     ;; door
     (:initial -> :selecting-region-for-door)
     (:selecting-region-for-door -> :initial) ;cancel
     (:selecting-region-for-door -> :recognizing-door-pose)
     (:recognizing-door-pose -> :selecting-exec-for-door) ;recognition done
     (:recognizing-door-pose -> :initial) ;cancel
     (:selecting-exec-for-door -> :executing-door)          ;exec
     (:executing-door -> :selecting-exec-for-door)          ;cancel
     (:selecting-exec-for-door -> :initial)                   ;cancel
     (:executing-door -> :initial)                            ;done
     ;; debri
     (:initial -> :selecting-region-for-debri)
     (:selecting-region-for-debri -> :initial) ;cancel
     (:selecting-region-for-debri -> :recognizing-debri-pose)
     (:recognizing-debri-pose -> :selecting-exec-for-debri) ;recognition done
     (:recognizing-debri-pose -> :initial) ;cancel
     (:selecting-exec-for-debri -> :executing-debri)          ;exec
     (:executing-debri -> :selecting-exec-for-debri)          ;cancel
     (:selecting-exec-for-debri -> :initial)                   ;cancel
     (:executing-debri -> :initial)                            ;done
     ;; hose-grasp
     (:initial -> :selecting-region-for-hose-grasp)
     (:selecting-region-for-hose-grasp -> :initial) ;cancel
     (:selecting-region-for-hose-grasp -> :recognizing-hose-grasp-pose)
     (:recognizing-hose-grasp-pose -> :selecting-exec-for-hose-grasp) ;recognition done
     (:recognizing-hose-grasp-pose -> :initial) ;cancel
     (:selecting-exec-for-hose-grasp -> :executing-hose-grasp)          ;exec
     (:executing-hose-grasp -> :selecting-exec-for-hose-grasp)          ;cancel
     (:selecting-exec-for-hose-grasp -> :initial)                   ;cancel
     (:executing-hose-grasp -> :initial)                            ;done
     ;; hose-connect
     (:initial -> :selecting-region-for-hose-connect)
     (:selecting-region-for-hose-connect -> :initial) ;cancel
     (:selecting-region-for-hose-connect -> :recognizing-hose-connect-pose)
     (:recognizing-hose-connect-pose -> :selecting-exec-for-hose-connect) ;recognition done
     (:recognizing-hose-connect-pose -> :initial) ;cancel
     (:selecting-exec-for-hose-connect -> :executing-hose-connect)          ;exe
     (:executing-hose-connect -> :selecting-exec-for-hose-connect)          ;cancel
     (:selecting-exec-for-hose-connect -> :initial)                   ;cancel
     (:executing-hose-connect -> :initial)                            ;done
     ;; drill_grasp
     (:initial -> :selecting-region-for-drill-grasp)
     (:initial -> :recognizing-drill-grasp-pose)
     (:selecting-region-for-drill-grasp -> :initial) ;cancel
     (:selecting-region-for-drill-grasp -> :recognizing-drill-grasp-pose) 
     (:recognizing-drill-grasp-pose -> :selecting-exec-for-drill-grasp) ;recognition done
     (:recognizing-drill-grasp-pose -> :initial) ;cancel
     (:selecting-exec-for-drill-grasp -> :executing-drill-grasp)          ;exec
     (:executing-drill-grasp -> :selecting-exec-for-drill-grasp)          ;cancel
     (:selecting-exec-for-drill-grasp -> :initial)                   ;cancel
     (:executing-drill-grasp -> :initial)                            ;done
     ;; drill_put
     (:initial -> :selecting-region-for-drill-put)
     (:selecting-region-for-drill-put -> :initial) ;cancel
     (:selecting-region-for-drill-put -> :recognizing-drill-put-pose) 
     (:recognizing-drill-put-pose -> :selecting-exec-for-drill-put) ;recognition done
     (:recognizing-drill-put-pose -> :initial) ;cancel
     (:selecting-exec-for-drill-put -> :executing-drill-put)          ;exec
     (:executing-drill-put -> :selecting-exec-for-drill-put)          ;cancel
     (:selecting-exec-for-drill-put -> :initial)                   ;cancel
     (:executing-drill-put -> :initial)                            ;done
     ;; drill_wall
     (:initial -> :selecting-region-for-drill-wall)
     (:selecting-region-for-drill-wall -> :initial) ;cancel
     (:selecting-region-for-drill-wall -> :recognizing-drill-wall)
     (:recognizing-drill-wall -> :selecting-exec-for-drill-wall) ;recognition done
     (:recognizing-drill-wall -> :initial) ;cancel
     (:selecting-exec-for-drill-wall -> :executing-drill-wall)          ;exec
     (:executing-drill-wall -> :selecting-exec-for-drill-wall)          ;cancel
     (:selecting-exec-for-drill-wall -> :initial)                   ;cancel
     (:executing-drill-wall -> :initial)                            ;done
     ;; drill_button
     (:initial -> :selecting-region-for-drill-button)
     (:selecting-region-for-drill-button -> :initial) ;cancel
     (:selecting-region-for-drill-button -> :recognizing-drill-button-pose)
     (:recognizing-drill-button-pose -> :selecting-exec-for-drill-button) ;recognition done
     (:recognizing-drill-button-pose -> :initial) ;cancel
     (:selecting-exec-for-drill-button -> :executing-drill-button)          ;exec
     (:executing-drill-button -> :selecting-exec-for-drill-button)          ;cancel
     (:selecting-exec-for-drill-button -> :initial)                   ;cancel
     (:executing-drill-button -> :initial)                            ;done
     ;; drill_rotate
     (:initial -> :selecting-region-for-drill-rotate)
     (:selecting-region-for-drill-rotate -> :initial) ;cancel
     (:selecting-region-for-drill-rotate -> :recognizing-drill-rotate-pose)
     (:recognizing-drill-rotate-pose -> :selecting-exec-for-drill-rotate) ;recognition done
     (:recognizing-drill-rotate-pose -> :initial) ;cancel
     (:selecting-exec-for-drill-rotate -> :executing-drill-rotate)          ;exec
     (:executing-drill-rotate -> :selecting-exec-for-drill-rotate)          ;cancel
     (:selecting-exec-for-drill-rotate -> :initial)                   ;cancel
     (:executing-drill-rotate -> :initial)                            ;done
     ;; driving
     (:initial -> :vehicle)
     (:vehicle -> :initial)
     )
   '((:initial :initial)
     ;; go-pos
     (:selecting-exec-for-go-pos :go-pos)
     (:selecting-region-for-go-pos :go-pos)
     (:recognizing-go-pos-goal :go-pos)
     (:executing-go-pos :go-pos)
     ;; look-at
     (:selecting-region-for-look-at :look-at)
     (:executing-look-at :look-at)
     ;; look-around
     (:executing-look-around :look-around)
     ;; valve
     (:selecting-region-for-valve :valve)
     (:recognizing-valve-pose :valve)
     (:selecting-exec-for-valve :valve)
     (:executing-valve :valve)
     ;; door
     (:selecting-region-for-door :door)
     (:recognizing-door-pose :door)
     (:selecting-exec-for-door :door)
     (:executing-door :door)
     ;; debri
     (:selecting-region-for-debri :debri)
     (:recognizing-debri-pose :debri)
     (:selecting-exec-for-debri :debri)
     (:executing-debri :debri)
     ;; hose-grasp
     (:selecting-region-for-hose-grasp :hose-grasp)
     (:recognizing-hose-grasp-pose :hose-grasp)
     (:selecting-exec-for-hose-grasp :hose-grasp)
     (:executing-hose-grasp :hose-grasp)
     ;; hose-connect
     (:selecting-region-for-hose-connect :hose-connect)
     (:recognizing-hose-connect-pose :hose-connect)
     (:selecting-exec-for-hose-connect :hose-connect)
     (:executing-hose-connect :hose-connect)
     ;; drill-grasp
     (:selecting-region-for-drill-grasp :drill-grasp)
     (:recognizing-drill-grasp-pose :drill-grasp)
     (:selecting-exec-for-drill-grasp :drill-grasp)
     (:executing-drill-grasp :drill-grasp)
     ;; drill-put
     (:selecting-region-for-drill-put :drill-put)
     (:recognizing-drill-put-pose :drill-put)
     (:selecting-exec-for-drill-put :drill-put)
     (:executing-drill-put :drill-put)
     ;; drill-wall
     (:selecting-region-for-drill-wall :drill-wall)
     (:recognizing-drill-wall :drill-wall)
     (:selecting-exec-for-drill-wall :drill-wall)
     (:executing-drill-wall :drill-wall)
     ;; drill-button
     (:selecting-region-for-drill-button :drill-button)
     (:recognizing-drill-button-pose :drill-button)
     (:selecting-exec-for-drill-button :drill-button)
     (:executing-drill-button :drill-button)
     ;; drill-rotate
     (:selecting-region-for-drill-rotate :drill-rotate)
     (:recognizing-drill-rotate-pose :drill-rotate)
     (:selecting-exec-for-drill-rotate :drill-rotate)
     (:executing-drill-rotate :drill-rotate)
     ;; driving
     (:vehicle :vehicle)
     )
   :initial "ocs_state.png" t))

(defun make-fc-state-machine ()
  (make-state-machine
   '(
     (:initial -> :recognizing-debri-pose)
     (:recognizing-debri-pose -> :initial)
     
     (:initial -> :recognizing-hose-grasp-pose)
     (:recognizing-hose-grasp-pose -> :initial)
     
     (:initial -> :recognizing-hose-connect-pose)
     (:recognizing-hose-connect-pose -> :initial)

     (:initial -> :recognizing-drill-grasp-pose)
     (:recognizing-drill-grasp-pose -> :initial)

     (:initial -> :recognizing-drill-put-pose)
     (:recognizing-drill-put-pose -> :initial)

     (:initial -> :recognizing-drill-wall)
     (:recognizing-drill-wall -> :initial)

     (:initial -> :recognizing-drill-button-pose)
     (:recognizing-drill-button-pose -> :initial)

     (:initial -> :recognizing-drill-rotate-pose)
     (:recognizing-drill-rotate-pose -> :initial)

     (:initial -> :recognizing-door-pose)
     (:recognizing-door-pose -> :initial)

     (:initial -> :recognizing-valve-pose)
     (:recognizing-valve-pose -> :initial)

     (:initial -> :recognizing-go-pos-location)
     (:recognizing-go-pos-location -> :initial)
     
     (:initial -> :executing-debri)
     (:executing-debri -> :initial)

     (:initial -> :executing-hose-grasp)
     (:executing-hose-grasp -> :initial)

     (:initial -> :executing-hose-connect)
     (:executing-hose-connect -> :initial)

     (:initial -> :executing-drill-grasp)
     (:executing-drill-grasp -> :initial)

     (:initial -> :executing-drill-put)
     (:executing-drill-put -> :initial)

     (:initial -> :executing-drill-wall)
     (:executing-drill-wall -> :initial)

     (:initial -> :executing-drill-button)
     (:executing-drill-button -> :initial)

     (:initial -> :executing-drill-rotate)
     (:executing-drill-rotate -> :initial)
     
     (:initial -> :executing-door)
     (:executing-door -> :initial)

     (:initial -> :executing-valve)
     (:executing-valve -> :initial)

     (:initial -> :executing-go-pos)
     (:executing-go-pos -> :initial)
     
     (:initial -> :recognizing-look-at-point)
     (:recognizing-look-at-point -> :initial)

     (:initial -> :recognizing-look-at-point-panorama)
     (:recognizing-look-at-point-panorama -> :recognizing-look-at-point)
     (:recognizing-look-at-point-panorama -> :initial)
     
     (:initial -> :executing-look-around)
     (:executing-look-around -> :initial)

     (:initial -> :executing-angle-vector)
     (:executing-angle-vector -> :initial)

     (:initial -> :executing-reach-until-touch)
     (:executing-reach-until-touch -> :initial)

     (:initial -> :executing-vehicle)
     (:executing-vehicle -> :initial)
     )
   '((:initial :initial)
     
     (:recognizing-debri-pose :debri)
     (:executing-debri :debri)
     
     (:recognizing-hose-grasp-pose :hose)
     (:recognizing-hose-connect-pose :hose)
     (:executing-hose-grasp :hose)
     (:executing-hose-connect :hose)

     (:recognizing-drill-grasp-pose :drill)
     (:executing-drill-grasp :drill)
     (:recognizing-drill-put-pose :drill)
     (:executing-drill-put :drill)
     (:recognizing-drill-wall :drill)
     (:executing-drill-wall :drill)
     (:recognizing-drill-button-pose :drill)
     (:executing-drill-button :drill)
     (:recognizing-drill-rotate-pose :drill)
     (:executing-drill-rotate :drill)


     (:recognizing-door-pose :door)
     (:executing-door :door)
     
     (:recognizing-go-pos-location :go-pos)
     (:executing-go-pos :go-pos)
     
     (:recognizing-valve-pose :valve)
     (:executing-valve :valve)

     (:recognizing-look-at-point :look-at)
     (:recognizing-look-at-point-panorama :look-at)
     
     (:executing-look-around :look-around)

     (:executing-angle-vector :angle-vector)

     (:executing-reach-until-touch :reach-until-touch)

     (:executing-vehicle :vehicle)
     )
   :initial nil t
   ))
