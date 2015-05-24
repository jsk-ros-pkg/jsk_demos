#!/usr/bin/env roseus

(ros::roseus "joint_states_storage")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")


;(setq *queue* nil)

(defun joint-state-array-callback (msg)
  (dotimes (i (length (send msg :joint_state_array)))
    (let ((msg-single (elt (send msg :joint_state_array) i)))
        (send msg-single :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
        (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg-single)
        ;(ros::spin-once)
        (unix::usleep (* (send msg :step_time) 1000))
      ;(push-back (list (* (send msg :step_time) 1000) msg-single) *queue*)
      )
    )
  )


(ros::subscribe "/model_joint_state_array" drc_task_common::JointStateArray #'joint-state-array-callback 5)
(ros::advertise "/jsk_model_marker_interface/robot/reset_joint_states_and_root" sensor_msgs::JointState 1)
(ros::rate 10)
;; (defun to-usec
;;   (time time-before)
;;   (+ (* (- (send time :sec) (send time-before :sec)) 1000000) (/ (- (send time :nsec) (send time-before :nsec)) 1000))
;;   )

;; (setq *time-before* nil)
;; (setq *time* nil)
(ros::spin)
;; (while t
;;   (ros::sleep)
;;   (ros::spin-once)
;;   (when (and *queue* (or (not *time-before*) (> (to-usec (ros::time-now) *time-before*) *time*) (< (to-usec (ros::time-now) *time-before*) 0)))
;;       (let* ((current (pop *queue*))
;;             (msg-single (elt current 1))
;;             (sleep-time (elt current 0))
;;             )
;;         (setq *time* sleep-time)
;;         (setq *time-before* (ros::time-now))
;;         (send msg-single :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
;;         (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg-single)
;;                                         ;(ros::spin-once)
;;         ;(unix::usleep (/ sleep-time 1))
;;         )
;;       )
;;   )
  