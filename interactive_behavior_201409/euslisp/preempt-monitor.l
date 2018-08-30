#!/usr/bin/env roseus
;; preempt-monitor.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "speech_recognition_msgs")
(ros::load-ros-manifest "app_manager")

(ros::roseus "preempt_monitor")

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")
(require :conv-utils "package://interactive_behavior_201409/euslisp/conv-utils.l")

(defparameter *preempt-phrases*
  (ros::get-param "/preempt/preempt_phrases"
                  (list "ちょっと待って" "待って" "ストップ")))

(defparameter *app-running-p* nil)
(defparameter *robot-name* (ros::get-param "/robot/name" "robot"))
(defparameter *current-state* nil)

(defun app-list-cb (msg)
  (let ((runningp (not (null (send msg :running_apps)))))
    (when (and runningp (not *app-running-p*))
      (ros::ros-info "App resumed")
      (setq *current-state* :running))
    (setq *app-running-p* runningp)))

(defun speech-to-text-cb (msg)
  (let ((text (car (send msg :transcript))))
    (unless (and (stringp text)
                 (not (null-string-p text)))
      (return-from speech-to-text-cb nil))
    (ros::ros-info "preempt-monitor current-state: ~A" *current-state*)
    (case *current-state*
     (:running
      (when (find-if #'(lambda (p) (substringp p text)) *preempt-phrases*)
        (ros::ros-warn "Preempt from speech!")
        (send *ri* :preempt :speech)
        (send *ri* :speak-jp "中断します")
        (setq *current-state* :interrupted)))
     (:interrupted
      (unless (send *ri* :preempted-p)
        (ros::ros-info "中断が解除されているみたいです．")
        (setq *current-state* nil)
        (return-from speech-to-text-cb nil))
      ;;
      (let ((command (parse-preempt-speech text)))
        (case (cdr (assoc :command command))
          (:add t)
          (:remove
           ;; remove object from goal condition and resume
           (when (ros::has-param *preempted-goal-param-key*)
             (let ((cnd (ros::get-param *preempted-goal-param-key*)))
               (when (or (not (stringp cnd)) (null-string-p cnd))
                 (ros::ros-error "invalid preempted-goal-param: ~A" cnd)
                 (send *ri* :reset-preempt)
                 (send *ri* :speak-jp "すみません．もう一度タスク指示をおねがいします．")
                 (setq cnd (read-from-string cnd))
                 (return-from speech-to-text-cb nil))
               ;;
               ;; TODO: remove state including sym
               (ros::ros-info "Removing ~A" (cdr (assoc :before command)))
               (ros::ros-info "before: ~A" (pformat cnd))
               (ros::ros-info "after: ~A" (pformat (remove-state-from-condition cnd)))
               (ros::set-param *preempted-goal-param-key*
                               (format nil "~A" (remove-state-from-condition cnd)))
               (ros::ros-info "~Aはやめます" (cdr (assoc :before command)))
               t)))
          (:update t)
          ;;
          )))
     )))


(defun remove-state-from-condition (sym cnd)
  "Remove state related to `sym` from condition `cnd`"
  (cond
   ((and (listp (car cnd)) (not (null (car cnd))))
    (remove nil (mapcar #'(lambda (x) (remove-state-from-condition sym x)) cnd)))
   (t
    (cond
     ((or (eq (car cnd) 'or) (eq (car cnd) 'and))
      (let ((scnd (remove-state-from-condition sym (cdr cnd))))
        (cond
         ((null scnd) nil)
         ((eq (length scnd) 1) (car scnd))
         (t (append (list (car cnd)) scnd)))))
     ((eq (car cnd) 'when)
      (let ((scnd (remove-state-from-condition sym (cadr cnd))))
        (if scnd (append (list (car cnd) scnd (cddr cnd))) nil)))
     (t
      (if (find sym (cdr cnd)) nil cnd))))))


(defun main ()
  (pr2-init)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-to-text-cb)
  (ros::subscribe (format nil "/~A/app_list" *robot-name*) app_manager::AppList #'app-list-cb)
  (ros::ros-info "Initialized")
  (ros::spin))

(main)
