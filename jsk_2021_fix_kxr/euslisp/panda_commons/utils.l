#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/parameters.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

;;; check if an id exists in target-list
(defun id-in-list (id target-list)
  (dolist (x target-list)
    (if (eq id x)
	(return-from id-in-list t)
      nil))
  nil)

(defun marker-name-to-id (marker-name)
    (if (string= marker-name "rpov-rarm_elbow_p_servo")
	(setq *target-id* 2)
    (if (string= marker-name "lpov-rarm_elbow_p_servo")
	(setq *target-id* 2))))

(defun move-rarm-x (delta)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :rarm :move-end-pos (float-vector delta 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun move-rarm-y (delta)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :rarm :move-end-pos (float-vector 0 delta 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun move-rarm-z (delta)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :rarm :move-end-pos (float-vector 0 0 delta))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun search-body ()
  (let ((detection-results-msg nil)
	(body-front-id 0)
	(body-back-id 1)
	(id-list nil))
    
    (ros::ros-info "searching body tag...")    
    (send *robot* :reset-coop-pose)
    (send *robot* :larm :move-end-pos #f(0 -80 0))
    (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(230 -140.079 1500.936) :rpy (float-vector 3.1 2.1 0))) :translation-axis t :rotation-axis t)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;;recognize marker
    (setq detection-results-msg (one-shot-subscribe "/right_tag_detections" apriltag_ros::AprilTagDetectionArray))
    (when detection-results-msg
      (ros::ros-info "Got a tag info")
      (setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
      (if (id-in-list body-front-id id-list)
	  (progn
	    (ros::ros-info "body-front was found")
	    (return-from search-body "front")))
      (if (id-in-list body-back-id id-list)
	  (progn
	    (ros::ros-info "body-back was found")
	    (return-from search-body "front"))))
      
    (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(230 -140.079 1500.936) :rpy (float-vector 3.1 1.7 0))) :translation-axis t :rotation-axis t)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (setq detection-results-msg (one-shot-subscribe "/right_tag_detections" apriltag_ros::AprilTagDetectionArray))
    (when detection-results-msg
      (ros::ros-info "Got a tag info")
      (setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
      (if (id-in-list body-front-id id-list)
	  (progn
	    (ros::ros-info "body-front was found")
	    (return-from search-body "front")))
      (if (id-in-list body-back-id id-list)
	  (progn
	    (ros::ros-info "body-back was found")
	    (return-from search-body "front"))))
    
    (ros::ros-info "couldn't find body")
    (return-from search-body nil)))
    
(defun rarm-search-servo-tag (marker-name)
  (let ((detection-results-msg nil)
	(target-id nil)
	(id-list nil)
	(base-to-search-pose nil)
	(count 0))
    (ros::ros-info "searching tag...")
    (setq target-id (marker-name-to-servo-id marker-name))
    (while t
      (setq detection-results-msg (one-shot-subscribe "/right_tag_detections" apriltag_ros::AprilTagDetectionArray))
      (when detection-results-msg
	(ros::ros-info "Got a message")
	(setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
	(print id-list)
	(if (id-in-list target-id id-list)
	    (progn
	      (ros::ros-info "target-marker was found")
	      (return-from rarm-search-servo-tag t))
	  (progn
	    (setq count (+ count 1))
	    (case count
		  (1
		   (setq base-to-search-pose (send (send *base_to_rarm_elbow_p_servo* :copy-worldcoords) :transform *servo-to-search-tag-pose-1*))
		   (send *robot* :rarm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (2
		   (setq base-to-search-pose (send (send *base_to_rarm_elbow_p_servo* :copy-worldcoords) :transform *servo-to-search-tag-pose-2*))
		   (send *robot* :rarm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (3
		   (setq base-to-search-pose (send (send *base_to_rarm_elbow_p_servo* :copy-worldcoords) :transform *servo-to-search-tag-pose-3*))
		   (send *robot* :rarm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (4
		   (ros::ros-info "Can't find tag")
		   (return-from rarm-search-servo-tag nil)))))))))

(defun larm-grasp-rarm-elbow-p-servo-from-below ()
  (let ((base-to-marker-origin nil)
	(base-to-inter1-pose nil)
	(base-to-inter2-pose nil)
	(base-to-inter3-pose nil)
	(base-to-pre-grasp-pose nil)
	(base-to-grasp-pose nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "start to grasp rarm-elbow-p-servo from below")
    (setq stamp (ros::time-now))
    (setq marker-name "lpov-rarm_elbow_p_servo")
    (when (send *tfl* :wait-for-transform "dual_arm_base" marker-name stamp 5)
      (setq base-to-marker-origin (send *tfl* :lookup-transform "dual_arm_base" marker-name stamp))
      (setq *base_to_rarm_elbow_p_servo* base-to-marker-origin))
    (setq base-to-inter1-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-inter1-pose*))
    (setq base-to-inter2-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-inter2-pose*))
    (setq base-to-pre-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-grasp-pose*))

    (send *ri* :stop-grasp :larm)
    (unix:sleep 1)

    ;;move larm under rarm-elbow-p-servo
    (send *robot* :larm :inverse-kinematics base-to-inter1-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *robot* :larm :inverse-kinematics base-to-inter2-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *robot* :larm :inverse-kinematics base-to-pre-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *robot* :larm :inverse-kinematics base-to-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :larm)
    (unix:sleep 1)))

#|
memo to get parameters
(send *robot* :angle-vector (send *ri* :state :potentio-vector))
(setq btgp (send *robot* :rarm :end-coords))
(setq param (send (send (send base-to-marker-origin :inverse-transformation) :copy-worldcoords) :transform (send btgp :copy-worldcoords)))
(setq test (send (send base-to-marker-origin :copy-worldcoords) :transform param))
(objects (list *robot* base-to-marker-origin test))
|#

(defun rarm-grasp-cable (servo-name side)
  (let ((base-to-servo-marker-origin nil)
	(base-to-cable-marker-origin nil)
	(base-to-pre-prasp-pose nil)
	(base-to-grasp-pose nil)
	(self-coords nil)
	(cable-id-list nil)
	(target-id-f nil)
	(target-id-b nil)
	(grasp-cable-id nil)
	(grasp-cable-side nil)
	(stamp nil)
	(ret nil))
    (setq count 0)
    (setq stamp (ros::time-now))
    (when (send *tfl* :wait-for-transform "dual_arm_base" servo-name stamp 5)
      (setq base-to-servo-marker-origin (send *tfl* :lookup-transform "dual_arm_base" servo-name stamp)))
    (ros::ros-info "searching cable...")
    (setq cable-id-list (servo-name-to-cable-id-list servo-name))
    (if (string= side "right")
	(progn
	  (ros::ros-info "target is right cable")
	  (setq target-id-f (elt cable-id-list 0))
	  (setq taregt-id-b (elt cable-id-list 1)))
      (progn
	(ros::ros-info "target is right cable")
	(setq target-id-f (elt cable-id-list 2))
	(setq taregt-id-b (elt cable-id-list 3))))
    (while t
      (setq detection-results-msg (one-shot-subscribe "/right_tag_detections" apriltag_ros::AprilTagDetectionArray))
      (when detection-results-msg
	(ros::ros-info "Got a message")
	(setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
	(if (id-in-list target-id-f id-list)
	    (progn
	      (ros::ros-info "front-marker was found")
	      (setq grasp-cable-side "front")
	      (return)))
	(if (id-in-list target-id-b id-list)
	    (progn
	      (ros::ros-info "back-marker was found")
	      (setq grasp-cable-side "back")
	      (return)))
	(setq count (+ count 1))
	(case count
	      (1
	       (setq base-to-search-pose (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *servo-to-search-cable-pose-1*))
	       (setq ret (send *robot* :rarm :inverse-kinematics base-to-search-pose))
	       (if (not ret) (ros::ros-error "IK-failed") nil)
	       (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	       (send *ri* :wait-interpolation))
	      (2
	       (setq base-to-search-pose (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *servo-to-search-cable-pose-2*))
	       (setq ret (send *robot* :rarm :inverse-kinematics base-to-search-pose))
	       (if (not ret) (ros::ros-error "IK-failed") nil)
	       (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	       (send *ri* :wait-interpolation))
	      (3
	       (setq base-to-search-pose (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *servo-to-search-cable-pose-3*))
	       (setq ret (send *robot* :rarm :inverse-kinematics base-to-search-pose))
	       (if (not ret) (ros::ros-error "IK-failed") nil)
	       (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	       (send *ri* :wait-interpolation))
	      (4
	       (ros::ros-info "Can't find cable")
	       (return-from rarm-grasp-cable nil)))))
    
    (setq base-to-search-pose (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *servo-to-cable-front-grasp-ready*))
    
    (ros::ros-info "start to grasp cable")
    (setq stamp (ros::time-now))
    (setq buf (subseq servo-name 0 (- (length servo-name) 5)))
    (setq s (subseq side 0 1))
    (setq p (subseq grasp-cable-side 0 1))
    (setq marker-name (format nil "~A~A~A~A~A" buf "cable_" s "_" p))
    (setq stamp (ros::time-now))
    (when (send *tfl* :wait-for-transform "dual_arm_base" marker-name stamp 5)
      (setq base-to-cable-marker-origin (send *tfl* :lookup-transform "dual_arm_base" marker-name stamp)))
    
    (send *robot* :rarm :move-end-pos #f(-20 0 30))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm_joint1 :joint-angle 60)
    (send *robot* :rarm :inverse-kinematics base-to-search-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (if (string= grasp-cable-side "front")
	(progn
	  (setq base-to-pre-grasp-pose-1 (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-front-to-pre-grasp-pose-1*))
	  (setq base-to-pre-grasp-pose-2 (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-front-to-pre-grasp-pose-2*))
	  (setq base-to-grasp-pose (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-front-to-grasp-pose*)))
      (progn
        (setq base-to-pre-grasp-pose-1 (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-back-to-pre-grasp-pose-1*))
	(setq base-to-pre-grasp-pose-2 (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-back-to-pre-grasp-pose-2*))
	(setq base-to-grasp-pose (send (send base-to-cable-marker-origin :copy-worldcoords) :transform *cable-back-to-grasp-pose*))))
    
    ;;pre-grasp-poses
    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)
    (send *ri* :stop-grasp :rarm :width 0.02)
    (unix:sleep 1)
    
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose-1)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose-2)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;;grasp-pose
    (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :move-end-pos #f(2 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;;grasp
    (send *ri* :start-grasp :rarm)
    (unix:sleep 2)

    (setq base-to-change-pose-1 (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *cable-front-change-pose-before-insert-1*))
    (setq base-to-change-pose-2 (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *cable-front-change-pose-before-insert-2*))
    (setq base-to-change-pose-3 (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *cable-front-change-pose-before-insert-3*))

    (send *robot* :rarm :inverse-kinematics base-to-change-pose-1)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :inverse-kinematics base-to-change-pose-2)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :inverse-kinematics base-to-change-pose-3)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    ;; (setq self-coords (send *robot* :rarm :end-coords))
    ;; (setq target-pos (send (setq self-coords (send *robot* :rarm :end-coords)) :worldpos))
    ;; (send (setq target-coords (send (send base-to-servo-marker-origin :copy-worldcoords) :transform *cable-front-change-pose-before-insert*)) :replace-pos target-pos)
    
    ;; (send *robot* :rarm :inverse-kinematics target-coords)
    ;; (send *robot* :rarm :move-end-pos #f(5 10 10))
    ;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    ;; (send *ri* :wait-interpolation)
    ))

(defun rarm-insert-cable (servo-name side)
  (let ((base-to-marker-origin nil)
	(base-to-pre-insert-pose nil)
	(base-to-insert-pose nil)
	(cable-id-list nil)
	(stamp nil)
	(ret nil))
    (setq buf (subseq servo-name 5 (length servo-name)))
    (setq base-to-marker-origin (eval (read-from-string (format nil "~A~A~A" "*base_to_" buf "*"))))
    (if (string= side "right")
	(progn
	  (setq base-to-pre-insert-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *servo-to-slide-insert-right-cable-inter-pose*))
	  (setq base-to-insert-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *servo-to-slide-insert-right-cable-pose*)))
      (progn
	(setq base-to-pre-insert-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *servo-to-slide-insert-left-cable-inter-pose*))
	(setq base-to-insert-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *servo-to-slide-insert-left-cable-pose*))))

    ;;set hand
    (send *robot* :rarm :inverse-kinematics base-to-pre-insert-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :inverse-kinematics base-to-insert-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    ;;get corret wrench and set as wrench-init


    (setq wrench-init (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
    (while (equal wrench-init *null-wrench*)
      (ros::ros-info "Couldn't get correct wrench")
      ;;TODO escape from singular pose
      (setq wrench-init (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k)))

    (setq i 0)
    (setq z-lim 4)
    (setq base-stroke #f(0.5 0 0))
    (while (< i 30)
      (setq wrench (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
      (setq wrench-diff (v- wrench wrench-init))
      (setq diff-z (elt wrench-diff 2))
      (if (> diff-z z-lim)
	  (progn
	    (ros::ros-info "detect contacting to serface of servo")
	    (return))
	(progn
	  (ros::ros-info "still hanging in the air")
	  (send *robot* :rarm :move-end-pos base-stroke)
	  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
	  (send *ri* :wait-interpolation)
	  (setq i (+ i 1))
	  (setq stroke (* (elt base-stroke 0) i))
	  (ros::ros-info (format nil "stroke is ~A" stroke)))))

    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :rarm :move-end-pos #f(0 0 -5))
    (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (setq h_delta 1.2)
    (setq i_delta 1.0)
    (setq z-lim 3)
    (setq total-insert 0)
    (setq total-insert-th 7.0)
    (setq mode 1)
    (setq count 0)
    (while t
      (setq wrench-pre (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
      (case mode
	    (1
	     (ros::ros-info "Move to +y")
	     (setq count (+ count 1)) 
	     (move-rarm-y h_delta)
	     (move-rarm-x i_delta)
	     (unix:sleep 1)
	     (setq wrench-tmp (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
	     (setq wrench-diff (v- wrench-tmp wrench-pre))
	     (setq diff-z (elt wrench-diff 2))
	     (if (< diff-z z-lim)
		 (progn
		   (ros::ros-info "Good")
		   (setq total-insert (+ total-insert i_delta))
		   (if (> h_delta 0.5)
		       (setq h_delta (- h_delta 0.1)))
		   (ros::ros-info (format nil "total_insert is ~A" total-insert))
		   (ros::ros-info (format nil "Now h_delta is ~A" h_delta)))
	       (progn
		 (ros::ros-info "Bad")
		 (move-rarm-y (- 0 h_delta))
		 (move-rarm-x (- 0 i_delta))))
	     (setq mode 2))
	    (2
	     (ros::ros-info "Move to -y")
	     (setq count (+ count 1))
	     (move-rarm-y (- 0 h_delta))
	     (move-rarm-x i_delta)
	     (unix:sleep 1)
	     (setq wrench-tmp (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
	     (setq wrench-diff (v- wrench-tmp wrench-pre))
	     (setq diff-z (elt wrench-diff 2))
	     (if (< diff-z z-lim)
		 (progn
		   (ros::ros-info "Good")
		   (setq total-insert (+ total-insert i_delta))
		   (if (> h_delta 0.5)
		       (setq h_delta (- h_delta 0.1)))
		   (ros::ros-info (format nil "total_insert is ~A" total-insert))
		   (ros::ros-info (format nil "Now h_delta is ~A" h_delta)))
	       (progn
		 (ros::ros-info "Bad")
		 (move-rarm-y h_delta)
		 (move-rarm-x (- 0 i_delta))))
	     (setq mode 3))
	    (3
	     (ros::ros-info "Move to +z")
	     (setq count (+ count 1))
	     (move-rarm-z h_delta)
	     (move-rarm-x i_delta)
	     (unix:sleep 1)
	     (setq wrench-tmp (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
	     (setq wrench-diff (v- wrench-tmp wrench-pre))
	     (setq diff-z (elt wrench-diff 2))
	     (if (< diff-z z-lim)
		 (progn
		   (ros::ros-info "Good")
		   (setq total-insert (+ total-insert i_delta))
		   (if (> h_delta 0.5)
		       (setq h_delta (- h_delta 0.1)))
		   (ros::ros-info (format nil "total_insert is ~A" total-insert))
		   (ros::ros-info (format nil "Now h_delta is ~A" h_delta)))
	       (progn
		 (ros::ros-info "Bad")
		 (move-rarm-y h_delta)
		 (move-rarm-x (- 0 i_delta))))
	     (setq mode 4))
	    (4
	     (ros::ros-info "Move to -z")
	     (setq count (+ count 1))
	     (move-rarm-z (- 0 h_delta))
	     (move-rarm-x i_delta)
	     (unix:sleep 1)
	     (setq wrench-tmp (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
	     (setq wrench-diff (v- wrench-tmp wrench-pre))
	     (setq diff-z (elt wrench-diff 2))
	     (if (< diff-z z-lim)
		 (progn
		   (ros::ros-info "Good")
		   (setq total-insert (+ total-insert i_delta))
		   (if (> h_delta 0.5)
		       (setq h_delta (- h_delta 0.1)))
		   (ros::ros-info (format nil "total_insert is ~A" total-insert))
		   (ros::ros-info (format nil "Now h_delta is ~A" h_delta)))
	       (progn
		 (ros::ros-info "Bad")
		 (move-rarm-y h_delta)
		 (move-rarm-x (- 0 i_delta))))
	     (setq mode 1)))
      (ros::ros-info (format nil "times of try is ~A" count))
      (if (or (> total-insert total-insert-th) (> count 20))
	  (progn
	    (ros::ros-info "rushing")
	    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
	    (send *robot* :rarm :move-end-pos #f(10 0 0))
	    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	    (send *ri* :wait-interpolation)
	    (send *robot* :rarm :move-end-pos #f(-10 0 0))
	    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
	    (send *ri* :wait-interpolation)
	    (return-from rarm-insert-cable t)))
      ;; (setq f hoge)
      ;; (if f
      ;; 	  (progn
      ;; 	    (ros::ros-info "success")
      ;; 	    (return from rarm-insert-cable-test t))
      ;; 	(progn
      ;; 	  (return to start point)))
      )))
		 
	    

#|
(setq wrench-init (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k)))
(send *robot* :rarm :move-end-pos base-stroke)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(setq wrench (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
(setq wrench-diff (v- wrench wrench-init))

(send *robot* :rarm :move-end-pos #f(0 0 -1))
(send *ri* :angle-vector (send *robot* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(setq wrench (send (one-shot-subscribe "/dual_panda/rarm_state_controller/franka_states" franka_msgs::FrankaState) :k_f_ext_hat_k))
(setq wrench-diff (v- wrench wrench-init))

(setq cable-copy (send base-to-cable-marker-origin :copy-worldcoords))
    (send cable-copy (send cable_origin :copy-worldcoords))
    (setq cable-rpy (elt (rpy-angle (send cable-copy :rot)) 0))
    (if (> (elt cable-rpy 0) 0.7)
	(progn
	  (rplaca)
	  (send base-to-cable-marker-origin :rpy 0.7 (elt cable-rpy 1) (elt cable-rpy 2))
|#
