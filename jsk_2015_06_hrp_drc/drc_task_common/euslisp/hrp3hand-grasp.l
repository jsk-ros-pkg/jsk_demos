#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")

(defun init-robot-for-grasp
  ()
  (unless (boundp '*ri*) (hrp2jsknts-init))
  (setq *robot* *hrp2jsknts*)
  )

;; grasp with pose
(defun pose-grasp
  (poses)
  (dotimes (i (length poses))
    (send *robot* :hand :arms (elt poses i))
    (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 1000)
    (send *ri* :hand-wait-interpolation)
    )
  )

;; grasp-data
;; thumb grasp
(setq *t-grasp-data* (make-hash-table))
(setf (gethash :pose-list *t-grasp-data*) (list :hook-pose :extension-pose :grasp-pose)) ;; do grasp pose sequence
(setf (gethash :link-index *t-grasp-data*) (list 1 5)) ;; reference link index for calc error
(setf (gethash :answer-vector-list *t-grasp-data*) (list (float-vector -11.4 50.3) (float-vector 34 45))) ;; learned angle
(setf 
 (gethash :answer-list *t-grasp-data*) (list :free :grasp) ;;(list nil t)
      ) ;; learned pose name


(defun grasp-from-data 
  (data &key (learn nil))
  (pose-grasp (gethash :pose-list *t-grasp-data*))
  ; service to get arm pose
  (let (res (state-vec (copy-object (elt (gethash :answer-vector-list data) 0))) error-vec (answer nil))
    (setq res (ros::service-call "/HRP3HandControllerServiceROSBridge/getRobotState" (instance jsk_hrp2_ros_bridge::OpenHRP_HRP3HandControllerService_getRobotStateRequest :init)))
    (dotimes (i (length (gethash :link-index data))) 
      (setf (elt state-vec i) (elt (send res :potentio) (elt (gethash :link-index data) i)))
      )
    (ros::ros-info (format nil "state-vec ~A" state-vec))
    ;;calc error with learned poses
    (let ((min-error 100000))
      (dotimes (i (length (gethash :link-index data)))
	(setq error-vec (v- (elt (gethash :answer-vector-list data) i) state-vec))
	(when (> min-error (v. error-vec error-vec))
	  (setq min-error (v. error-vec error-vec))
	  (setq answer (elt (gethash :answer-list data) i))
	  )
	)
      (when learn ;; learn new pose 
	(if (> min-error 60) 
	    (progn
	      (push state-vec (gethash :answer-vector-list data))
	      (push learn (gethash :answer-list data))
              (ros::ros-info (format nil "learn ~A pose (vector: ~A)" learn state-vec)))
          (ros::ros-info "data already exists")
	  )
	) 
      )
    answer ;;return pose
    )
  )
(unless (boundp '*robot*) (init-robot-for-grasp))

(format t "(grasp-from-data *t-grasp-data*
)~%")