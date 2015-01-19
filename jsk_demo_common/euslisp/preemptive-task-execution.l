#!/usr/bin/env roseus

(ros::roseus "preemptive_task_execute_manager")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :attention-observation "package://jsk_demo_common/euslisp/attention-observation.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

(defmacro lambda-motion (name &rest forms)
  `#'(lambda (args)
       (let ((as (cdr (assoc 'current-action-state args)))
             (motion-name (symbol-name ',name)))
         (ros::ros-info "args: ~A" args)
         (if (find-if #'(lambda (s)
                              (and (stringp s) (string= motion-name s))) as)
             (progn
               (warn ";; skipping ~A~%" (quote ,name))
               :success)
             (let (attention-barrier-return)
               (warn ";; exec ~A~%" (quote ,name))
               (setq attention-barrier-return
                     (catch :attention-barrier
                       (let ((attention-motion-return (progn ,@forms)))
                         (unless attention-motion-return
                           (throw :attention-barrier :fail))
                         (nreverse as)
                         (push motion-name as)
                         (nreverse as)
                         (setf (cdr (assoc 'current-action-state args)) as)
                         :success
                         )))
               (warn ";; exec ~A finish with ~A~%" (quote ,name) attention-barrier-return)
               (cond
                 ((stringp attention-barrier-return)
                  (let ((str (send *ri* :get-cancel-flag)))
                    (setq attention-barrier-return
                          (intern (string-upcase attention-barrier-return) *keyword-package*))
                    ))
                 ((null attention-barrier-return)
                  (warn ";; attention-barrier-return = nil~%");;
                  (setq attention-barrier-return :fail)) ;; just fail action
                 ((eq t attention-barrier-return) ;; normal cancel
                  (setq attention-barrier-return :fail))
                 )
               (send *ri* :clear-cancel-flag)
               (unless (or (eq attention-barrier-return :fail)
                           (eq attention-barrier-return :success))
                 (warn ";; throw ~A~%" attention-barrier-return)
                 (throw :all-barrier attention-barrier-return))
               attention-barrier-return)
             ))))

(defmacro motion (name &rest forms)
   `(cons ',name (lambda-motion ,name ,@forms)))

(defun pause-action (args)
  (ros::rate 1)
  (let ((param-name "pause_action_recover"))
    (while (ros::ok)
     (when (ros::has-param param-name)
       (unless (string= (ros::get-param param-name) "recover")
         (ros::set-param param-name "")
         (return-from pause-action :success)))
     (ros::ros-info "pausing...")
     (ros::spin-once)
     (ros::sleep))))

(defun pause-motion (args)
  (ros::rate 1)
  (let ((param-name "pause_motion_recover"))
    (while (ros::ok)
     (when (ros::has-param param-name)
       (cond
         ((string= (ros::get-param param-name) "recover")
          (ros::set-param param-name "")
          (return-from pause-motion :success))
         ((string= (ros::get-param param-name) "fallback")
          (ros::set-param param-name "")
          (return-from pause-motion :fail))))
     (ros::ros-info "pausing...")
     (ros::spin-once)
     (ros::sleep))))

(defun make-action (name &rest motions)
  (let ((sm-action (instance state-machine :init))
        (pause-action-name (concatenate string name "_PAUSE"))
        prev-motion-name)
    (send sm-action :arg-keys 'current-action-state 'current-task-state)
    (send sm-action :goal-state (list :goal :fallback))
    (send sm-action :add-node
          (instance state :init pause-action-name #'pause-motion))
    (send sm-action :add-node
          (instance state :init :mark-action-done #'(lambda (args)
                                                      (let ((ts (cdr (assoc 'current-task-state args)))
                                                            (as (cdr (assoc 'current-action-state args))))
;;                                                        (nreverse ts)
;;                                                        (push pause-action-name ts) FIXME: cannot use closure
;;                                                        (nreverse ts)
;;                                                        (setf (cdr (assoc 'current-task-state args)) ts)
                                                        (setf (cdr (assoc 'current-action-state args)) nil)
                                                        :success
                                                      ))
                    :remap-list '((curren-task-state . current-task-state)
                                  (current-action-state . current-action-state))))
    (send sm-action :add-transition :mark-action-done :goal :success)
    (dolist (m motions)
      (let ((motion-name (symbol-name (car m))))
        (send sm-action :add-node
              (instance state :init motion-name (cdr m)
                        :remap-list '((current-action-state . current-action-state))))
        (send sm-action :add-transition motion-name pause-action-name :fail)
        (if prev-motion-name
            (send sm-action :add-transition prev-motion-name motion-name :success)
            (send sm-action :start-state motion-name))
        (setq prev-motion-name motion-name)))
    (send sm-action :add-transition pause-action-name (symbol-name (caar motions)) :success)
    (send sm-action :add-transition pause-action-name :fallback :fail)
    (send sm-action :add-transition prev-motion-name :mark-action-done :success)
    sm-action))

(defmacro action (name &rest actions)
  `(cons ',name (make-action (symbol-name ',name) ,@actions)))

(defun task (actions)
  (let ((sm-task (instance state-machine :init))
        prev-action-name)
    (send sm-task :arg-keys 'task-state 'action-state)
    (send sm-task :goal-state (list :goal :failure))
    (dolist (ac actions)
      (let* ((action-name (symbol-name (car ac)))
             (pause-action-name (concatenate string action-name "_ACTION_PAUSE"))
             (sm-action (cdr ac)))
        (send sm-task :add-node
              (instance state :init action-name sm-action
                        :remap-list '((current-task-state . task-state)
                                      (current-action-state . action-state))))
        (send sm-task :add-node
              (instance state :init pause-action-name #'pause-action))
        (send sm-task :add-transition action-name pause-action-name :fallback)
        (send sm-task :add-transition pause-action-name action-name :success)
        (send sm-task :add-transition pause-action-name :failure :fail)
        (if prev-action-name
            (send sm-task :add-transition prev-action-name action-name :goal)
            (send sm-task :start-state action-name))
        (setq prev-action-name action-name)))
    (send sm-task :add-transition prev-action-name :goal :goal)
    sm-task))

(defun execute-task (task &key state userdata (hz 1))
  (let ((sm-inspector (instance state-machine-inspector :init task))
        task-state action-state)
    (if state
        (send sm-inspector :reset-state :on-state state :user-data userdata)
        (send sm-inspector :reset-state :user-data userdata))

    (when (null userdata)
      (push '(task-state . nil) userdata)
      (push '(action-state . nil) userdata))
    (ros::ros-info "userdata: ~A" userdata)
    (unix:sleep 1)
    (exec-smach-with-interrupt sm-inspector :state state :mydata userdata :hz hz)))

(provide :preemptive-task-execution)
