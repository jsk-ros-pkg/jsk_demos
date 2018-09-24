#!/usr/bin/env roseus
;; joint-torque-monitor.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "joint_torque_monitor")

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")

(defparameter *joint-state-table* (make-hash-table))
(defparameter *enable-monitor* nil)

(defun arm-moving-p (ri ctrl)
  (let* ((ct (robot-interface-controller-table ri))
         (ac (car (gethash ctrl ct)))
         (tm (send ac :last-feedback-msg-stamp))
         (prev-tm (or (gethash ctrl *joint-state-table*) tm)))
    (setf (gethash ctrl *joint-state-table*) tm)
    (> (send (ros::time- tm prev-tm) :to-sec) 0.0)))

(defun max-torque-ratio (jts)
  (apply #'max (mapcar #'(lambda (jt)
                           (/ (abs (send jt :joint-torque))
                              (send jt :max-joint-torque))) jts)))

(defun check-arm-preempt (arm thresh)
  (let ((cont (read-from-string (format nil "~A-controller" arm)))
        (torq (max-torque-ratio (send *ri* :robot arm :joint-list))))
    (when (arm-moving-p *ri* cont)
      (when (> torq thresh)
        (ros::ros-info "~A torque: ~A > ~A (threshold)" arm torq thresh)
        (ros::ros-warn "preempted with ~A" (read-from-string (format nil "~A-torque" arm)))
        (send *ri* :preempt (read-from-string (format nil "~A-torque" arm)))
        (send *ri* :start-mannequin-mode arm)))
    torq))


(defun mannequin-mode-p (arm)
  "PR2 depend function"
  (let ((ct (send *ri* :list-controllers)))
    (case arm
      (:larm (and (cdr (assoc "l_arm_controller_loose" ct :test #'string=))
                  (not (cdr (assoc "l_arm_controller" ct :test #'string=)))))
      (:rarm (and (cdr (assoc "r_arm_controller_loose" ct :test #'string=))
                  (not (cdr (assoc "r_arm_controller" ct :test #'string=)))))
      (t (error (format nil "invalid arm ~A" arm))))))

(defun main ()
  (pr2-init)
  (setq *enable-monitor* nil)
  (ros::advertise-service
   "~enable" std_srvs::Empty
   #'(lambda (req)
       (setq *enable-monitor* t)
       (ros::ros-info "joint_torque_monitor enabled")
       (send req :response)))
  (ros::advertise-service
   "~disable" std_srvs::Empty
   #'(lambda (req)
       (setq *enable-monitor* nil)
       (ros::ros-info "joint_torque_monitor disabled")
       (send req :response)))
  (ros::rate 10)
  (let ((preempt-thresh (ros::get-param "~max_torque_ratio" 0.45))
        prev-preempted-p preempted-p
        preempted-reason torq-lst)
    (setq preempted-p (send *ri* :preempted-p)
          prev-preempted-p preempted-p)
    (ros::ros-info "joint_torque_monitor initialized")
    (while (ros::ok)
      (send *ri* :state)
      (setq preempted-p (send *ri* :preempted-p))
      (cond
        ((and (null preempted-p) prev-preempted-p
              (memq preempted-reason (list :larm-torque :rarm-torque)))
         ;; preempted -> normal
         (ros::ros-warn "stopped mannequin mode")
         (when (mannequin-mode-p :larm) (send *ri* :stop-mannequin-mode :larm))
         (when (mannequin-mode-p :rarm) (send *ri* :stop-mannequin-mode :rarm))
         (setq preempted-reason nil))
        ((and preempted-p (null prev-preempted-p))
         ;; normal -> preempted
         (setq preempted-reason (send *ri* :preempted-reason)))
        ((and *enable-monitor* (null preempted-p))
         ;; normal mode
         (setq torq-lst (cons (check-arm-preempt :larm preempt-thresh)
                              (check-arm-preempt :rarm preempt-thresh)))
         (ros::ros-debug "torque: ~A" torq-lst))
        (t t))
      (setq prev-preempted-p preempted-p)
      (ros::spin-once)
      (ros::sleep))
    ))

(main)
(exit)
