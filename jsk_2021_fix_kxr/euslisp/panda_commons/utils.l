#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "franka_msgs")
(ros::load-ros-manifest "apriltag_ros")
(ros::load-ros-manifest "opencv_apps")
;;; check if an id exists in target-list

(defun marker-name-to-servo-id (marker-name)
  (setq marker-name (subseq marker-name 5 (length marker-name)))
  (if (string= marker-name "rarm_elbow_p_servo")
      (return-from marker-name-to-servo-id 2)))

(defun servo-name-to-cable-id-list (servo-name)
  (setq servo-name (subseq servo-name 5 (length servo-name)))
  (if (string= servo-name "rarm_elbow_p_servo")
      (return-from servo-name-to-cable-id-list (list 8 9 10 11))))

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

(defun make-timeout-servo-list (joint_states)
  (setq timeout-servo-list '())
  (setq count (- (length (send joint_states :name)) 1))
  (while (> count -1)
    (if (< (elt (send joint_states :effort) count) 0.1)
	(progn
	  (if (not (string= (elt (send joint_states :name) count) "HEAD-NECK-Y"))
	      (progn
		(setq timeout-servo-list (cons (elt (send joint_states :name) count) timeout-servo-list))
		(setq *reparing-servo-num* count)))))
    (setq count (- count 1)))
  timeout-servo-list)

(defun decide-target-servo (timeout-servo-list)
  (setq target-servo (elt timeout-servo-list 0))
  target-servo)

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
	(found-id nil)
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
	    (setq found-id body-front-id)
	    (return-from search-body "front")))
      (if (id-in-list body-back-id id-list)
	  (progn
	    (ros::ros-info "body-back was found")
	    (setq found-id body-front-id)
	    (return-from search-body "back"))))
      
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

(defun larm-search-rarm-elbow-p-servo-tag ()
  (let ((detection-results-msg nil)
	(target-id nil)
	(id-list nil)
	(base-to-search-pose nil)
	(count 0))
    (ros::ros-info "searching tag...")
    (setq marker-name "lpov-rarm_elbow_p_servo")
    (setq target-id (marker-name-to-servo-id marker-name))
    (setq start-pose (send (send *robot* :larm :end-coords) :copy-worldcoords))
    (while t
      (setq detection-results-msg (one-shot-subscribe "/left_tag_detections" apriltag_ros::AprilTagDetectionArray))
      (when detection-results-msg
	(ros::ros-info "Got a message")
	(setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
	(print id-list)
	(if (id-in-list target-id id-list)
	    (progn
	      (ros::ros-info "target-marker was found")
	      (return-from larm-search-rarm-elbow-p-servo-tag t))
	  (progn
	    (setq count (+ count 1))
	    (case count
		  (1
		   (setq base-to-search-pose (send (send start-pose :copy-worldcoords) :transform *rarm_elbow_servo_p_search-pose-1*))
		   (send *robot* :larm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (2
		   (setq base-to-search-pose (send (send start-pose :copy-worldcoords) :transform *rarm_elbow_servo_p_search-pose-2*))
		   (send *robot* :larm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (3
		   (setq base-to-search-pose (send (send start-pose :copy-worldcoords) :transform *rarm_elbow_servo_p_search-pose-3*))
		   (send *robot* :larm :inverse-kinematics base-to-search-pose)
		   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
		   (send *ri* :wait-interpolation))
		  (4
		   (ros::ros-info "Can't find tag")
		   (return-from larm-search-rarm-elbow-p-servo-tag nil)))))))))

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
    (send *robot* :larm :move-end-pos #f(-3 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :larm)
    (unix:sleep 1)
    
    (send *robot* :larm :move-end-pos #f(-3 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ))

(defun larm-release-rarm-elbow-p-servo-from-below ()
  (let ((base-to-marker-origin nil)
	(base-to-inter1-pose nil)
	(base-to-inter2-pose nil)
	(base-to-inter3-pose nil)
	(base-to-pre-grasp-pose nil)
	(base-to-grasp-pose nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "start to release rarm-elbow-p-servo from below")
    (setq stamp (ros::time-now))
    (setq marker-name "lpov-rarm_elbow_p_servo")
    (setq base-to-marker-origin *base_to_rarm_elbow_p_servo*)
    (setq base-to-inter1-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-inter1-pose*))
    (setq base-to-inter2-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-inter2-pose*))
    (setq base-to-pre-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *rarm-elbow-p-to-grasp-pose*))

    (send *ri* :stop-grasp :larm)
    (unix:sleep 1)

    (send *robot* :larm :inverse-kinematics base-to-pre-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *robot* :larm :inverse-kinematics base-to-inter2-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *robot* :larm :inverse-kinematics base-to-inter1-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)))


    
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
    (send *ri* :stop-grasp :rarm :width 0.01)
    (unix:sleep 1)
    
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose-1)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose-2)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;;grasp-pose
    ;; (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    ;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    ;; (send *ri* :wait-interpolation)
    ;; (send *robot* :rarm :move-end-pos #f(2 0 0))
    ;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    ;; (send *ri* :wait-interpolation)

    (setq goal_x 670)
    (setq goal_y 650)
    (setq robust_x 60)
    (setq min_x (- goal_x (/ robust_x 2)))
    (setq max_x (+ goal_x (/ robust_x 2)))
    (setq min_y 630)
    (setq max_y 675)
    (setq k_x 0.13)
    (setq k_y 0.116)
    (setq fx 0)
    (setq fy 0)
    (while (< fx 3)
      (setq rects (send (one-shot-subscribe "/rarm_cable_terminal_detection/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
      (when rects
	(progn
	  (setq l (length rects))
	  (setq i 0)
	  (setq max_i 0)
	  (setq size_max 0)
	  (setq size_buf 0)

	  (while (< i l)
	    (setq size_buf (* (send (send (elt rects i) :size) :width) (send (send (elt rects i) :size) :height)))
	    (if (> size_buf size_max)
		(progn
		  (setq size_max size_buf)
		  (setq max_i i)))
	    (ros::ros-info (format nil "fx:~A index:~A, size:~A" fx i size_buf))
	    (setq i (+ i 1)))
	  (setq x (send (send (elt rects max_i) :center) :x))
	  (setq y (send (send (elt rects max_i) :center) :y))
	  (ros::ros-info (format nil "x:~A, y:~A" x y))
	  (if (or (< x min_x) (> x max_x))
	      (progn
		(setq delta_x (* k_x (- x goal_x)))
		(setq fx 0))
	    (progn
	      (setq delta_x (* 0.4 (* k_x (- x goal_x))))
	    (setq fx (+ fx 1))))
	  (send *robot* :rarm :move-end-pos (float-vector 0 delta_x 0))
	  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
	  (send *ri* :wait-interpolation)
	  (unix:sleep 1))))
    
    (while (< fy 3)
      (setq rects (send (one-shot-subscribe "/rarm_cable_terminal_detection/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
      (when rects
	(progn
	  (setq l (length rects))
	  (setq i 0)
	  (setq max_i 0)
	  (setq size_max 0)
	  (setq size_buf 0)

	  (while (< i l)
	    (setq size_buf (* (send (send (elt rects i) :size) :width) (send (send (elt rects i) :size) :height)))
	    (if (> size_buf size_max)
		(progn
		  (setq size_max size_buf)
		  (setq max_i i)))
	    (ros::ros-info (format nil "fy:~A index:~A, size:~A" fy i size_buf))
	    (setq i (+ i 1)))
	  (setq x (send (send (elt rects max_i) :center) :x))
	  (setq y (send (send (elt rects max_i) :center) :y))
	  (ros::ros-info (format nil "x:~A, y:~A" x y))
	  (cond
	   ((< y min_y) (progn
			  (setq delta_y (* 0.7 (* k_y (- y goal_y))))
			  (setq fy 0)))
	    ((> y max_y) (progn
			   (setq delta_y (* 1.0 (* k_y (- y goal_y))))
			   (setq fy 0)))
	   (t (progn
		(setq delta_y 0)
		(setq fy (+ fy 1)))))
	  (send *robot* :rarm :move-end-pos (float-vector 0 0 delta_y))
	  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
	  (send *ri* :wait-interpolation)
	  (unix:sleep 1))))

    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)
    (send *ri* :stop-grasp :rarm :width 0.003)
    (unix:sleep 1)

    (send *robot* :rarm :move-end-pos (float-vector 2 2 -4))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)

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
    (setq stamp (ros::time-now))
    (when (send *tfl* :wait-for-transform "dual_arm_base" servo-name stamp 5)
      (setq base-to-marker-origin (send *tfl* :lookup-transform "dual_arm_base" servo-name stamp)))
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
    (while (< i 15)
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
    (send *robot* :rarm :move-end-pos #f(0 0 -8))
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
	    (send *ri* :stop-grasp :rarm :width 0.005)
	    (unix:sleep 1)
	    (send *robot* :rarm :move-end-pos #f(-30 0 30))
	    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	    (send *ri* :wait-interpolation)

	    (send *robot* :rarm :move-end-pos #f(-70 0 0))
	    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
	    (send *ri* :wait-interpolation)
	    (return-from rarm-insert-cable t))))   
    ))
		 
	    

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
