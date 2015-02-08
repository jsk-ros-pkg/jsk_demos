#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")

(defun init-robot-for-grasp
  ()
  (unless (boundp '*ri*) (hrp2jsknts-init))
  (setq *robot* *hrp2jsknts*)
  )

(format t "(init-robot-for-grasp)~%")

(defun pose-grasp
  (poses)
  (dotimes (i (length poses))
    (send *robot* :hand :arms (elt poses i))
    (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 1000)
    (send *ri* :hand-wait-interpolation)
    )
  )

(setq *t-grasp-data* (make-hash-table))
(setf (gethash :pose *t-grasp-data*) (list :hook-pose :extension-pose :grasp-pose))
(setf (gethash :false-answer *t-grasp-data*) (float-vector -11.4 50.3))
(setf (gethash :index *t-grasp-data*) (list 1 5))
(setf (gethash :threshold *t-grasp-data*) 100)
;
(defun grasp-from-data ;thumb grasp
  (&key (data *t-grasp-data*) (calib nil))
  (pose-grasp (gethash :pose *t-grasp-data*))
  ; service to get arm pose
  (let (res (state-vec (copy-object (gethash :false-answer *t-grasp-data*))) error-vec)
    (setq res (ros::service-call "/HRP3HandControllerServiceROSBridge/getRobotState" (instance jsk_hrp2_ros_bridge::OpenHRP_HRP3HandControllerService_getRobotStateRequest :init)))
    (dotimes (i (length (gethash :index *t-grasp-data*))) 
      (setf (elt state-vec i) (elt (send res :potentio) (elt (gethash :index *t-grasp-data*) i)))
      )
    ;; (setq state-vec (float-vector (elt (send res :potentio) 1) (elt (send res :potentio) 5)))
    (if calib (setf (gethash :false-answer *t-grasp-data*) state-vec))
    (setq error-vec (v- (gethash :false-answer *t-grasp-data*) state-vec))
    (if (<  (gethash :threshold *t-grasp-data*) (v. error-vec error-vec))
        (progn (ros::ros-info "grasp succeeded") )
      (progn (ros::ros-info "grasp failed") nil)
      )
    )
  )

(unless (boundp '*robot*) (init-robot-for-grasp))

(format t "(grasp-from-data)~%")
