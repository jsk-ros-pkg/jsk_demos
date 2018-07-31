#!/usr/bin/env roseus
;; preemptlib.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

;; imports
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

;; parameters
(defparameter *preempt* t "set nil to disable preemption")
(defparameter *preempt-param-key* "/preempt/preempted")
(defparameter *preempted-state-param-key* "/preempt/preempted_state")
(defparameter *preempt-failure-value* :fail)
(defparameter *preempt-success-value* :success)

;; utilities
(defun pformat (sexp)
  "pprint but returns output as a string"
  (with-output-to-string (s)
    (pprint sexp s)))


;; add interface for pddl / smach
(defmethod transition
  (:from-state () (car (get self :pddl-state)))
  (:to-state () (cdr (get self :pddl-state))))


(defmacro with-check-preemption (&rest form)
  "check / raise preemption signal on form block"
  (let ((ret (gensym "PREEMPT")) (val (gensym "PREEMPT")))
    `(let ((,ret ,@form) ,val)
       (when (and (memq :preempt-barrier (sys::list-all-catchers))
                  (or (if (atom ,ret) (null ,ret) (every #'null ,ret))
                      (ros::has-param *preempt-param-key*)))
         (setq ,val (ros::get-param *preempt-param-key*))
         (throw :preempt-barrier
           (if (stringp ,val)
               (intern (string-upcase ,val) *keyword-package*) ,val)))
       ,ret)))


(defmacro defpreemptrobot (name &rest form)
  "This macro defines a subclass of `robot`-interface, where preemption is supported.
The original init function (e.g. pr2-init) is overwritten to initialize an instance of the class.
Example: (defpreemptrobot pr2) creates `pr2-preempt-interface` and `pr2-init`"
  (let ((cls (read-from-string (format nil "~A-preempt-interface" name)))
        (sprcls (read-from-string (format nil "~A-interface" name)))
        (initfunc (read-from-string (format nil "~A-init" name)))
        (initorigfunc (read-from-string (format nil "~A-init-orig" name))))
  `(prog1
    (defclass ,cls
      :super ,sprcls
      :slots ())
    (defmethod ,cls
  (:wait-interpolation
   (&rest args)
   (with-check-preemption
       (some #'null (send-super* :wait-interpolation args))))
  (:move-to
   (&rest args)
   (with-check-preemption
       (send-super* :move-to args)))
  (:move-to-wait
   (&rest args)
   (let ((ret (send-super* :move-to-wait args)))
     (if (send self :is-preempted) nil ret)))
  (:go-pos
   (&rest args)
   (with-check-preemption
       (send-super* :go-pos args)))
  (:go-pos-unsafe
   (&rest args)
   (with-check-preemption
       (send-super* :go-pos-unsafe args)))
  (:move-gripper
   (&rest args)
   (with-check-preemption
       (send-super* :move-gripper args)))
  (:ros-wait
   (&rest args)
   (with-check-preemption
     (send-super* :ros-wait args)))
  (:robot-interface-simulation-callback
   ()
   (when (or (send self :interpolatingp) current-goal-coords)
     (send self :ros-wait 0.03)
     (if (send self :is-preempted) (setq current-goal-coords nil)))
   (send-super :robot-interface-simulation-callback)
   nil)
  (:wait-interpolation-raw
   (&rest args)
   (send-super* :wait-interpolation args))
  (:move-to-raw
   (&rest args)
   (send-super* :move-to args))
  (:go-pos-raw
   (&rest args)
   (send-super* :go-pos args))
  (:go-pos-unsafe-raw
   (&rest args)
   (send-super* :go-pos-unsafe args))
  (:move-gripper-raw
   (&rest args)
   (send-super* :move-gripper args))
  (:is-preempted ()
   (and (ros::has-param *preempt-param-key*)
        (not (null (ros::get-param *preempt-param-key*)))))
  (:reset-preempt ()
   (ros::delete-param *preempt-param-key*)
   (ros::delete-param *preempted-state-param-key*)
   (unix:usleep (* 500 1000)))
  (:preempt
   (&optional (reason :preempted))
   (ros::set-param *preempt-param-key* reason)
   (progn ,@form)
   reason)
) ;; defmethod
;;
(unless (fboundp ',initorigfunc)
  (setf (symbol-function ',initorigfunc) (symbol-function ',initfunc)))
;;
(defun ,initfunc (&rest args)
  ,(format nil "Initializes *robot* and *ri* for ~A" cls) ;; docstring
  (when (and (not (boundp '*ri*)) *preempt*)
    (setq *ri* (instance ,cls :init)))
  (prog1
      (apply #',initorigfunc args)
    (setq *robot* *pr2*)))
)))

(defmacro defaction (name args &rest form)
  "This is similar to defun, but preemption is supported"
  (let ((ret (gensym "DEFACTION"))
        (name-raw (read-from-string (format nil "~A-raw" name))))
    `(progn
       (defun ,name-raw ,args
         (block ,name (progn ,@form)))
       (defun ,name ,args
         (unless *preempt*
           (return-from ,name (funcall #',name-raw ,@args)))
         (let (,ret)
           (prog1
               (setq ,ret
                     (catch :preempt-barrier
                       (ros::ros-info ";; ~A" (list ',name ,@args))
                       (setq ,ret (block ,name (progn ,@form)))
                       (ros::ros-warn ";; return value: ~A" ,ret)
                       (unless ,ret (throw :preempt-barrier *preempt-failure-value*))
                       *preempt-success-value*))
             (when (and (memq :exec-barrier (sys::list-all-catchers))
                        (not (memq ,ret (list *preempt-failure-value* *preempt-success-value*))))
               (throw :exec-barrier ,ret))) ;; caught by exec-preempt-task
           ,ret)))))

(defun exec-preemptive-task (sm &key (mydata '(nil)))
  "This function is like exec-state-machine in roseus_smach/src/state-machine-utils.l, but supports preemption"
  (let ((insp (instance state-machine-inspector :init sm))
        result pddl-state)
    (unix::sleep 2)
    ;;
    ;; setup state machine
    (send sm :reset-state)
    (send insp :publish-structure) ;; publish once and latch
    (apply #'send sm :arg-keys (union (send sm :arg-keys) (mapcar #'car mydata)))
    ;;
    ;; check / reset preempt status
    (when (boundp '*ri*)
      (send *ri* :ros-wait 0.0 :spin nil :spin-self t)
      (when (send *ri* :is-preempted)
        (ros::ros-warn "The current task is preempted.")
        (unless (y-or-n-p ";; Reset preempt status?")
          (return-from exec-preemptive-task))
        (send *ri* :reset-preempt)))
    ;;
    ;; exec loop
    (setq result
          (catch :exec-barrier
            (while (ros::ok)
              (ros::spin-once)
              (when (boundp '*ri*)
                (send *ri* :ros-wait 0.0 :spin-self t :spin nil))
              ;;
              (send insp :publish-status mydata)
              ;;
              (when (send sm :goal-reached) (return))
              (setq result (send insp :state-machine :execute mydata :step -1))
              ;;
              (let ((trans (car (remove-if-not
                                 #'(lambda (tr) (send tr :check result))
                                 (send (send insp :state-machine :active-state) :arc-list)))))
                (when trans
                  (setq pddl-state (send trans :to-state))
                  (ros::ros-info ";; state -> ~A" (pformat pddl-state))))
              ) ;; while
            :finish))
    ;;
    ;; check the last pddl state
    (when (and (not (send sm :goal-reached)) (null pddl-state))
      (ros::ros-warn "current pddl-state is empty. use active state transition")
      (let ((trans (car (send (send sm :active-state) :arc-list))))
        (setq pddl-state (send trans :from-state))))
    (ros::ros-info "task finished with ~A -> ~A" result (pformat pddl-state))
    (when (and (send *ri* :is-preempted)
               (not (null pddl-state)))
      (ros::set-param *preempted-state-param-key* (format nil "~A" pddl-state)))
    (list result pddl-state))) ;; defaction

(defun merge-states (current updated &key (debug))
  "Resolve conflicts in pddl states and merge them into one state"
  (let ((cur (copy-object current))
        (upd (copy-object updated)))
    ;; prefer current onhand to updated on
    ;; (onhand ?OBJ ?ARM)
    (let ((c (remove-if-not #'(lambda (s) (eq 'onhand (car s))) cur)))
      (dolist (obj (mapcar #'(lambda (s) (cadr s)) c))
        (setq upd (remove-if #'(lambda (s)
                                 (and (eq 'on (car s))
                                      (eq obj (cadr s)))) upd))))
    (when debug
      (warn "remove on~%")
      (warn "cur: ~A~% upd: ~A~%" (pformat cur) (pformat upd)))
    ;;
    ;; prefer current on to updated onhand
    ;; (on ?OBJ ?SPOT)
    ;; NOTE: 逆かも
    (let ((c (remove-if-not #'(lambda (s) (eq 'onhand (car s))) upd)))
      (dolist (obj (mapcar #'(lambda (s) (cadr s)) c))
        (when (find-if #'(lambda (s)
                           (and (eq 'on (car s))
                                (eq obj (cadr s)))) cur)
          (setq upd (remove-if #'(lambda (s)
                                   (and (eq 'onhand (car s))
                                        (eq obj (cadr s)))) upd)))))
    (when debug
      (warn "remove onhand~%")
      (warn "cur: ~A~% upd: ~A~%" (pformat cur) (pformat upd)))
    ;;
    ;; prefer current at
    ;; (at ?SPOT)
    (let ((c (find-if #'(lambda (s) (eq 'at (car s))) cur)))
      (when c
        (setq upd (remove-if #'(lambda (s) (eq 'at (car s))) upd))))
    (when debug
      (warn "remove at~%")
      (warn "cur: ~A~% upd: ~A~%" (pformat cur) (pformat upd)))
    ;;
    ;; prefer updated function to current
    ;; (= (func) num) or (= num (func))
    (let ((fs (remove-if-not #'(lambda (s) (eq '= (car s))) upd)))
      (dolist (f fs)
        (setq cur (remove-if #'(lambda (s)
                                 (and (eq '= (car s))
                                      (equal (find-if #'listp f)
                                             (find-if #'listp s)))) cur))))
    (when debug
      (warn "remove func~%")
      (warn "cur: ~A~% upd: ~A~%" (pformat cur) (pformat upd)))
    ;;
    ;; prefer updated on
    (let ((c (remove-if-not #'(lambda (s) (eq 'on (car s))) upd)))
      (dolist (obj (mapcar #'(lambda (s) (cadr s)) c))
        (setq cur (remove-if #'(lambda (s)
                                 (and (eq 'on (car s))
                                      (eq obj (cadr s)))) cur))))
    ;;
    ;; merge states
    (ros::ros-info "merging current: ~A" (pformat cur))
    (ros::ros-info "updated: ~A" (pformat upd))
    (unique (append cur upd))
    ))


(defun plan-task (domain problem &key (debug t) (failed-nodes))
  "Plan action sequence with failure recovery, given domain and problem using PDDL solver.
This function also supports resume from preempted tasks"
  (let ((prob (copy-object problem))
        graph insp)
    ;;
    ;; count failed nodes if not specified
    (unless failed-nodes
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
                    failed-nodes)))
    ;;
    ;; merge preempted initial states into original initial condition
    (when (ros::has-param *preempted-state-param-key*)
      (let ((cnd (ros::get-param *preempted-state-param-key*)))
        (if (or (not (stringp cnd)) (null-string-p cnd))
            (ros::ros-error "preempted state is empty")
            (setq cnd (read-from-string cnd)))
        (send prob :initial-condition (merge-states cnd (send prob :initial-condition)))))
    ;;
    (ros::ros-info "planning from ~A" (pformat (send prob :initial-condition)))
    (ros::ros-info "to goal ~A" (pformat (send prob :goal-condition)))
    ;;
    ;; send domain / problem to pddl solver
    (setq graph (pddl-plan-to-graph nil
                                    :domain domain :problem prob
                                    :failed-nodes failed-nodes
                                    :readable nil
                                    :debug debug))
    (when debug
      (send graph :write-to-pdf "/tmp/graph.pdf" nil "hoge")
      (unix:system "xdg-open /tmp/graph.pdf"))
    ;;
    (setq insp (pddl-graph-to-smach graph :add-self-return-failure-path nil))
    (send insp :state-machine)))


(provide :preemptlib)
