#!/usr/bin/env roseus
;; recovery-mode.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "speech_recognition_msgs")
(ros::load-ros-manifest "app_manager")

(ros::roseus "recovery_mode")

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :problem-parser "package://mbtk/euslisp/problem-parser.l")
(require :conv-utils "package://interactive_behavior_201409/euslisp/conv-utils.l")

(defun handle-grasping (arm text)
  (when (find (car (find-word text :class :verb)) (list "離す" "開ける" "開く") :test #'string=)
    (send *ri* :speak-jp "手を離します")
    (send *ri* :stop-grasp arm :wait t))
  (when (find (car (find-word text :class :verb)) (list "掴む" "閉じる" "握る") :test #'string=)
    (send *ri* :speak-jp "手を掴みます")
    (send *ri* :start-grasp arm)))

(defun on-speech-while-preempted-cb (reason text)
  (cond
    ;; task level
    ((or (find (car (find-word text :class :noun)) (list "続き" "再開") :test #'string=)
         (find (car (find-word text :class :verb)) (list "続ける") :test #'string=))
     (send *ri* :speak-jp "続きをします")
     (send *ri* :resume))
    ((find (car (find-word text :class :noun)) (list "中止") :test #'string=)
     (send *ri* :speak-jp "中止します")
     (send *ri* :reset-preempt))
    ;; action level
    (t
     (preempted-handler reason text))))

(defun preempted-handler (reason text)
  (case reason
    (:speech
     ;; preempted by speech
     (let ((result (parse-preempt-speech text))
           cnd)
       (case (cdr (assoc :command result))
         (:add
          (setq cnd (send *ri* :preempted-goal))
          (push `(on ,(cdr (assoc :after result)) usual) cnd)
          (send *ri* :preempted-goal cnd))
         (:remove
          (setq cnd (send *ri* :preempted-goal))
          (setq cnd (remove-state-from-condition (cdr (assoc :before result)) cnd))
          (send *ri* :preempted-goal cnd))
         (:update
          (setq cnd (send *ri* :preempted-goal))
          (setq cnd (remove-state-from-condition (cdr (assoc :before result)) cnd))
          (push `(on ,(cdr (assoc :after result)) usual) cnd)
          (send *ri* :preempted-goal cnd))
         (t t))
     )
    (:larm-torque
     ;; preempted by larm torque
     (handle-grasping :larm text)
     )
    (:rarm-torque
     ;; preempted by rarm torque
     (handle-grasping :rarm text)
     )
    (:preempted
     ;; preempted by eus repl
     ;; manual mode
     )
    (t
     ;; unhandled situation
     (ros::ros-error "unhandled preempted reason: ~A" reason)))))


(defun main ()
  (pr2-init)
  (ros::subscribe "/speech_to_text"
                  speech_recognition_msgs::SpeechRecognitionCandidates
                  #'(lambda (m)
                      (when (send *ri* :preempted-p)
                        (on-speech-while-preempted-cb
                         (send *ri* :preempted-reason)
                         (car (send m :transcript))))))
  (ros::ros-info "recovery_mode initialized")
  (ros::rate 5)
  (while (ros::ok)
    (ros::spin-once)
    (send *ri* :spin-once)
    (ros::sleep)))

(main)
