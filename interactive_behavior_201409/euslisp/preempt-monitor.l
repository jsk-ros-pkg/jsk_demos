#!/usr/bin/env roseus
;; preempt-monitor.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "speech_recognition_msgs")
(ros::load-ros-manifest "app_manager")

(ros::roseus "preempt_monitor")

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")

(defparameter *preempt-phrases* (ros::get-param "/preempt/preempt_phrases"
                                                (list "ちょっと待って"
                                                      "待って"
                                                      "ストップ")))
(defparameter *app-running-p* nil)
(defparameter *robot-name* (ros::get-param "/robot/name" "robot"))

(defun app-list-cb (msg)
  (setq *app-running-p* (not (null (send msg :running_apps)))))

(defun speech-to-text-cb (msg)
  (let ((text (car (send msg :transcript))))
    (when (and *app-running-p*
               (stringp text)
               (not (null-string-p text))
               (find-if #'(lambda (p) (substringp p text)) *preempt-phrases*))
      (ros::ros-warn "Preempt from speech!")
      (send *ri* :preempt :speech))))


(defun main ()
  (pr2-init)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-to-text-cb)
  (ros::subscribe (format nil "/~A/app_list" *robot-name*) app_manager::AppList #'app-list-cb)
  (ros::ros-info "Initialized")
  (ros::spin))

(main)
