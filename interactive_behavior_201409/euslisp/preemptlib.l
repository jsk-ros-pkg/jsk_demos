#!/usr/bin/env roseus
;; preemptlib.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")


(defparameter *preempt-param-key* "/preempt/preempted")
(defparameter *preempt-failure-value* :fail)
(defparameter *preempt-success-value* :success)


(defmacro with-check-preemption (&rest form)
  (let ((ret (gensym "PREEMPT")) (val (gensym "PREEMPT")))
    `(let ((,ret ,@form) ,val)
       (when (and (memq :preempt-barrier (sys::list-all-catchers))
                  (if (atom ,ret) (null ,ret) (every #'null ,ret))
                  (ros::has-param *preempt-param-key*))
         (setq ,val (ros::get-param *preempt-param-key*))
         (throw :preempt-barrier
           (if (stringp ,val)
               (intern (string-upcase (ros::get-param *preempt-param-key*))
                       *keyword-package*)
               ,val))
       ,ret))))


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
       (some #'identity (send-super* :wait-interpolation args))))
  (:move-to
   (&rest args)
   (with-check-preemption
       (send-super* :move-to args)))
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
   (unix:usleep (* 500 1000)))
  (:preempt
   (&optional (reason :preempted))
   (ros::set-param *preempt-param-key* reason)
   ,@form)) ;; defmethod
    ;;
    (unless (fboundp ',initorigfunc)
      (setf (symbol-function ',initorigfunc) (symbol-function ',initfunc)))
    ;;
    (defun ,initfunc (&rest args)
      (unless (boundp '*ri*)
        (setq *ri* (instance ,cls :init)))
      (prog1
          (apply #',initorigfunc args)
        (setq *robot* *pr2*)))
    )))


(defmacro defaction (name args &rest form)
  "This is similar to defun, but preemption is supported"
  (let ((ret (gensym "DEFACTION")))
    `(progn
       (defun ,(read-from-string (format nil "~A-raw" name)) ,args ,@form)
       (defun ,name ,args
         (let (,ret)
           (unwind-protect
                (prog1
                  (setq ,ret
                        (catch :preempt-barrier
                          (setq ,ret (progn ,@form))
                          (ros::ros-warn ";; return value: ~A" ,ret)
                          (unless ,ret (throw :preempt-barrier *preempt-failure-value*))
                          *preempt-success-value*))
                  (when (and (memq :exec-barrier (sys::list-all-catchers))
                             (not (memq ,ret (list *preempt-failure-value* *preempt-success-value*))))
                    (throw :exec-barrier ,ret))) ;; caught by exec-preempt-task
             (ros::delete-param *preempt-param-key*))
           ,ret)))))

;; planning
#|
(defun state-match-p (state query context)
  (case (car query)
    ('not
     (null (state-match-p state (cadr query))))
    ('and
     (every #'(lambda (q) (state-match-p state q)) (cdr query)))
    ('or
     (some #'(lambda (q) (state-match-p state q)) (cdr query)))
    ('=
     (equal (cadr query) (caddr query)))
    ('when
     (if (state-match-p state (cadr query)) (state-match-p state (caddr query)) t))
    ('exists
     ())
    ('forall
     ())
    (t
     (member state query :test #'equal))))
|#

(defun exec-preemptive-task (sm &key (mydata '(nil)))
  (let ((insp (instance state-machine-inspector :init sm))
        result current-state)
    (unix::sleep 2)
    ;; setup state machine
    (send sm :reset-state)
    (send insp :publish-structure) ;; publish once and latch
    (apply #'send sm :arg-keys (union (send sm :arg-keys) (mapcar #'car mydata)))
    ;; check / reset preempt status
    (when (boundp '*ri*)
      (send *ri* :ros-wait 0.0 :spin nil :spin-self t)
      (when (send *ri* :is-preempted)
        (ros::ros-warn "The current task is preempted.")
        (unless (y-or-n-p ";; Reset preempt status?")
          (return-from exec-preemptive-task))
        (send *ri* :reset-preempt)))
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
              (let ((trans (remove-if-not #'(lambda (tr)
                                              (send tr :check result))
                                          (send (send insp :state-machine :active-state) :arc-list))))
                (when trans
                  (setq trans (car trans))
                  (setq current-state (send trans :state))
                  (ros::ros-info ";; -> ~A" current-state)))) ;; while
            :finish))
    ;; check current state
    (unless (send sm :goal-reached)
      (let ((trs (remove-if-not #'(lambda (tr)
                                    (send tr :check result))
                                (send (send sm :active-state) :arc-list))))
        (assert (eq (length trs) 1)
                (format nil "Found ~A multiple transisions." (lrngth trs)))
        (setq current-state (send (car trs) :state))))
    (ros::ros-info "task finished with ~A -> ~A" result current-state)
    (list result current-state)))


(provide :preemptlib)
