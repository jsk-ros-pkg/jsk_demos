#!/usr/bin/env roseus
;; sound-monitor.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "sound_monior")
(ros::load-ros-manifest "speech_recognition_msgs")

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")

(defparameter *preempt-phrase* (ros::get-param "~preempt_phrase"
                                               (list "ストップ" "待って" "やめて" "おーい")))
(defparameter *joint-state-table* (make-hash-table))

;; suppress warning messages
(unless (assoc :joint-angle-org (send rotational-joint :methods))
        (rplaca (assoc :joint-angle (send rotational-joint :methods)) :joint-angle-org))
(defmethod rotational-joint
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
              (setq v max-angle)))
       (cond ((< v min-angle)
              (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  )

(defun arm-moving-p (ri ctrl)
  (let* ((ct (robot-interface-controller-table ri))
         (ac (car (gethash ctrl ct)))
         (tm (send ac :last-feedback-msg-stamp))
         (prev-tm (or (gethash ctrl *joint-state-table*) tm)))
    (setf (gethash ctrl *joint-state-table*) tm)
    (> (send (ros::time- tm prev-tm) :to-sec) 0.0)))

(defun base-moving-p (ri)
  (let* ((ac (robot-move-base-interface-move-base-action ri))
         (status (send ac :last-status-msg)))
    (when (derivedp status actionlib_msgs::GoalStatusArray)
      (dolist (st (send status :status_list))
        (when (memq (send st :status) '(1 6 7))
          (return-from base-moving-p t))))))

(defun speech-cb (msg)
  (send *ri* :state)
  #|
  (ros::ros-info "when: ~A" (and (not (send *ri* :preempted-p))
                                 (or (arm-moving-p *ri* :rarm-controller)
                                     (arm-moving-p *ri* :larm-controller)
                                     (base-moving-p *ri*))))
  (ros::ros-info "speech: ~A" (send msg :transcript))
  (ros::ros-info "result: ~A" (find (car (send msg :transcript)) *preempt-phrase* :test #'string=))
  (ros::ros-info "arm: ~A ~A" (arm-moving-p *ri* :rarm-controller)
                 (arm-moving-p *ri* :larm-controller))
  (ros::ros-info "bsae: ~A" (base-moving-p *ri*))
  (ros::ros-info "preempt: ~A" (send *ri* :preempted-p))
  |#
  (when (and (not (send *ri* :preempted-p))
             (or (arm-moving-p *ri* :rarm-controller)
                 (arm-moving-p *ri* :larm-controller)
                 (base-moving-p *ri*)))
    (let* ((text (car (send msg :transcript)))
           (result (find text *preempt-phrase* :test #'string=)))
      (warning-message 1 "result2: ~A~%" result)
      (when result
        (ros::ros-warn "~A: preempted ~A" result :speech)
        (send *ri* :preempt :speech)))))

(defun main ()
  (pr2-init)
  (ros::subscribe "speech_to_text"
                  speech_recognition_msgs::SpeechRecognitionCandidates
                  #'speech-cb)
  (ros::ros-info "sound_monitor initialized")
  (ros::rate 5)
  (while (ros::ok)
    (ros::spin-once)
    (send *ri* :spin-once)
    (ros::sleep)))

(main)

