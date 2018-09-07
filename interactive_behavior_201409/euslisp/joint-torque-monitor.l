#!/usr/bin/env roseus
;; joint-torque-monitor.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "joint_torque_monitor")

(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")

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


(defparameter *joint-state-table* (make-hash-table))

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
  (ros::rate 10)
  (let ((preempt-thresh (ros::get-param "~max_torque_ratio" 0.4))
        prev-preempted-p preempted-p
        torq-lst)
    (setq preempted-p (send *ri* :preempted-p)
          prev-preempted-p preempted-p)
    (ros::ros-info "joint_torque_monitor initialized")
    (while (ros::ok)
      (send *ri* :state)
      (setq preempted-p (send *ri* :preempted-p))
      (cond
        ((and (null preempted-p) prev-preempted-p)
         ;; recover from preemption
         (when (mannequin-mode-p :larm) (send *ri* :stop-mannequin-mode :larm))
         (when (mannequin-mode-p :rarm) (send *ri* :stop-mannequin-mode :rarm)))
        ((null preempted-p)
         ;; normal mode
         (setq torq-lst (cons (check-arm-preempt :larm preempt-thresh)
                              (check-arm-preempt :rarm preempt-thresh)))
         (ros::ros-debug "torque: ~A" torq-lst))
        (t t))
      (setq prev-preempted-p preempted-p)
      (ros::sleep))
    ))

(main)
