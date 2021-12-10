#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/parameters.l")

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
	(print 2))
    (if (string= marker-name "lpov-rarm_elbow_p_servo")
	(print 2)))

(defun rarm-srearch-servo-tag (marker-name)
  (let ((detection-results-msg nil)
	(target-id nil)
	(id-list nil)
	(base-to-search-pose nil)
	(count 0))
    (ros::ros-info "searching tag...")
    (setq target-id (marker-name-to-id marker-name))
    (while t
      (setq detection-results-msg (one-shot-subscribe "/right_tag_detections" apriltag_ros::AprilTagDetectionArray))
      (when detection-results-msg
	(ros::ros-info "Got a message")
	(setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
	(if (id-in-list tesrget-id)
	    (progn
	      (ros::ros-info "target-marker was found")
	      (return from rarm-srearch-servo-tag t))
	  (progn
	    (setq count (+ count 1))
	    (case count
		  (1
		   (setq base-to-search-pose (send (send *base-to-rarm-elbow-p-servo* :copy-worldcoords) :transform *rarm-elbow-p-to-search-tag-pose-1*))
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (2
		   (setq base-to-search-pose (send (send *base-to-rarm-elbow-p-servo* :copy-worldcoords) :transform *rarm-elbow-p-to-search-tag-pose-2*))
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (3
		   (setq base-to-search-pose (send (send *base-to-rarm-elbow-p-servo* :copy-worldcoords) :transform *rarm-elbow-p-to-search-tag-pose-3*))
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (4
		   (ros::ros-info "Can't find tag")
		   (return from rarm-srearch-servo-tag t)))))))))
  
  

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
      (setq *base-to-rarm-elbow-p-servo* base-to-marker-origin))
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
(setq btgp (send *robot* :larm :end-coords))
(setq param (send (send (send base-to-marker-origin :inverse-transformation) :copy-worldcoords) :transform (send btgp :copy-worldcoords)))
(setq test (send (send base-to-marker-origin :copy-worldcoords) :transform param))
(setq param (send (send marker-to-base :copy-worldcoords) :transform (send btgp :copy-worldcoords)))

|#
(defun rarm-grasp-cable (servo-name)
  (let ((base-to-marker-origin nil)
	(base-to-pre-prasp-pose nil)
	(base-to-grasp-pose nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "grasp cable")
    (setq stamp (ros::time-now))
    (setq marker-name "cable_front")
    (when (send *tfl* :wait-for-transform "dual_arm_base" marker-name stamp 5)
      (setq base-to-marker-origin (send *tfl* :lookup-transform "dual_arm_base" marker-name stamp)))
    (setq base-to-pre-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *cable-front-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *cable-front-to-grasp-pose*))

    ;pre-grasp-pose
    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)
    (send *ri* :stop-grasp :rarm :width 0.02)
    (unix:sleep 1)
    
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;grasp-pose
    (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :rarm)
    (unix:sleep 2)
    (send *ri* :stop-grasp :rarm)
    (unix:sleep 2)))


(defun check-camera ()
  (let ((base-to-marker-000 nil)
	(base-to-marker-001 nil)
	(base-to-marker-002 nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "grasp cable")
    (setq stamp (ros::time-now))
    (setq marker-name "cable_front")
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_000" stamp 5)
      (setq *marker-000* (send *tfl* :lookup-transform "dual_arm_base" "test_000" stamp)))
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_000" stamp 5)
      (setq *marker-001* (send *tfl* :lookup-transform "dual_arm_base" "test_001" stamp)))
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_001" stamp 5)
      (setq *marker-002* (send *tfl* :lookup-transform "dual_arm_base" "test_002" stamp)))))
