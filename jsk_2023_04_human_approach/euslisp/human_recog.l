#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(load "package://spoteus/spot-interface.l")
(spot-init)
(ros::roseus "human_recog")

(defun move-to-human (msg)
  (let (rect-list distance max-rect x y)
    (setq rect-list (send msg :boxes))
    (if (not (= (length rect-list) 0))
  	(progn
	  (print "data-start")
  	  (setq sorted-list (sort rect-list #'compare-distance-of-rects))
  	  (setq max-rect (car sorted-list))
	  (setq x (send (send (send max-rect :pose) :position) :x))
	  (setq y (send (send (send max-rect :pose) :position) :y))
	  (setq *angle* (rad2deg (atan y x)))
	  ;; (print *angle*)
	  (send *ri* :go-pos 0 0 *angle*)
  	  (send *ri* :go-pos 0.5 0 0)
	  (send *ri* :speak-jp "おつかれさま")
  	  (print "done")
  	  )
      )
    )
  )



(defun recognition-skeleton (msgs)
  (setq *physical-order* (list 'right shoulder->right elbow 'right elbow->right wrist 'left shoulder->left elbow 'left elbow->left wrist))
  (setq *physical-reverse-order* (list 'right eldow->right shoulder 'right wrist->right elbow 'left elbow->left shoulder 'left wrist->left elbow))
  
  (ros::ros-info "length ~A" (length (send msgs :skeletons)))
  
  (if (<= (length (send msgs :skeletons)) 0)
      (progn
	(ros::ros-info "length inside if ~A" (length (send msgs :skeletons)))
	(print "1"))
    (progn
      (setq skeleton-list (send msgs :skeletons))
      (ros::ros-error "~A" skeleton-list)
      (dolist (skeleton skeleton-list)
	(if (send skeleton :bone_names)
	  (setq bone-names-list (send skeleton :bone_names))
	  (dolist (bone-name bone-names-list)
	    (dolist (answer-bone-name *physical-order*)
	      (if (string= bone-name answer-bone-name)
		(setq start-point (send bone-name :start_point))
		(setq end-point (send bone-name :end_point))
		(setq vector (v- end-point start-point))
		(print vector)
		)
	      )
	    )
	  )
	)
      )
    )
  )
	  (if (= (send skeleton :bone_names) (or "right shoulder->right elbow" )
	    (ros::ros-info "bones ~A" (send skeleton :bones))
	  (send

	 ;; (if (>= (length (send (send msgs :skeletons) :bone_names)) 1)
	 ;;   (ros::ros-info "~A" (car (send msgs :skeletons)))
	 ;;   (ros::ros-info "~A" (last (send msgs :skeletons))) 
	 ;;   (ros::ros-info "~A" (send (car (last (send msgs :skeletons))) :bones))
      ;; (print (send (last (send msgs :skeletons)) :bones)))
  ;;(if (= (send (send msg :skeletons) :bone_names) "right shoulder->right elbow")
      ;;(progn
	;;)
    ;;)

(defun vector (a b)
  (send (send (send b :skeletons) :bones) :start_point)

(defun compare-distance-of-rects (a b)
  (>= (norm (send (send a :pose) :position)) (norm (send (send a :pose) :position)))
  )
    
(setq *human* (one-shot-subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray))
;;(setq *skeleton* (one-shot-subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray))
(ros::subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray #'recognition-skeleton)

(print *human*)
(move-to-human *human*)
;;(recognition-skeleton *skeleton*)

;;(ros::subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray #'cb)
(ros::spin)
