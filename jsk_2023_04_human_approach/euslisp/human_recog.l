#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(load "package://spoteus/spot-interface.l")
(spot-init)
(ros::roseus "human_recog")

(send *ri* :angle-vector (send *spot* :reset-pose))
(send *ri* :gripper-close)

(defun move-to-human (msg)
  (let (rect-list distance max-rect)
    (setq rect-list (send msg :boxes))
    (if (not (= (length rect-list) 0))
  	(progn
	  ;;(print "data-start")
	  (if (= (length rect-list) 1)
	      (setq max-rect (car rect-list))
	    )
  	  ;;(setq sorted-list (sort rect-list #'compare-distance-of-rects))
  	  ;;(setq max-rect (car sorted-list))
	  (setq max-rect (car rect-list))
	  (setq *x* (send (send (send max-rect :pose) :position) :x))
	  (setq *y* (send (send (send max-rect :pose) :position) :y))
	  (setq *angle* (rad2deg (atan *y* *x*)))
	  (setq *human-to-spot-vector* (float-vector (* *x* -1) (* *y* -1) 0))
	  (print *angle*)
	  (send *ri* :go-pos 0 0 *angle*)
	  (send *ri* :go-pos 1.0 0 0)
	  )
      )
    )
  )

(defun ask()
  (send *ri* :gripper-open)
  (send *ri* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 -170.0 150.0 0.0 10.0 45.0 0.0))
  (send *ri* :speak-jp "どちらに行きますか？")
  (send *ri* :angle-vector (send *spot* :reset-pose))
  (send *ri* :gripper-close)
)
 


(defun recognition-skeleton (msgs)
  (if (= *speech-recog-res* 1)
      (progn
       (send *ri* :speak-jp "とまります。")
       (move-to-human-call)
       (ask)
       (send *speech-recog-res* 0) 
       ))
  (if (> *speech-recog-res* 1)
      (progn

	(let (physical-order physical-reverse-order skeleton-list bone-names-list bones index target-bone start-point end-point)
	  (setq physical-order (list "right shoulder->right elbow" "right elbow->right wrist" "left shoulder->left elbow" "left elbow->left wrist"))
	  (setq physical-reverse-order (list "right eldow->right shoulder" "right wrist->right elbow" "left elbow->left shoulder" "left wrist->left elbow"))
    
	  (ros::ros-info "skeletons ~A" (length (send msgs :skeletons)))
  
	  (if (>= (length (send msgs :skeletons)) 1)
	      (progn
		(setq skeleton-list (send msgs :skeletons))
		;;(ros::ros-error "~A" skeleton-list)
		
		(dolist (skeleton skeleton-list)
		  (if (send skeleton :bone_names)
		      (progn
			(setq bone-names-list (send skeleton :bone_names))
			(dolist (bone-name bone-names-list)
			  (print bone-name)
			  ;;(print (send skelton :bones)
			  (dolist (target-bone-name (append physical-order physical-reverse-order))
			    (if (string= bone-name target-bone-name)
				(progn
				  (setq bones (send skeleton :bones))
				  
				  (setq index (position bone-name bone-names-list :test 'equal))
				  (setq target-bone (elt bones index))
				  
				  (cond ((find target-bone-name physical-order)
					 (progn
					   (setq start-point (send target-bone :start_point))
					   (setq end-point (send target-bone :end_point))
					   ;;(setq *purpose-vector* (v- (vector end-point) (vector start-point)))
					   ))
					((find bone-name physical-reverse-order)
					 (progn
					   (setq end-point (send target-bone :start_point))
					   (setq start-point (send target-bone :end_point))
					   ;;(setq *purpose-vector* (v- (vector start-point) (vector end-point)))
					   )))
				  ;;(ros::ros-error "purpose-vector ~A" *purpose-vector*)
				  (setq *tfl* (instance ros::transform-listener :init))
				  (send *tfl* :wait-for-transform "base_link" "hand_color_image_sensor" (ros::time 0) 1)
				  (setq tf-base-to-arm (send *tfl* :lookup-transform "base_link" "hand_color_image_sensor" (ros::time 0)))
				  ;;(setq target-vector (send tf-base-to-arm :transform-vector *purpose-vector*)) 
				  ;;(send *ri* :angle-vector (send *spot* :head :look-at target-vector))

				  (setq tf-start-point (send tf-base-to-arm :transform-vector (vector start-point)))
				  (setq tf-end-point (send tf-base-to-arm :transform-vector (vector end-point)))
				  (print tf-start-point)
				  (print tf-end-point)
				  (setq target-vector (v- tf-end-point tf-start-point))
				  (ros::ros-error "target-vector ~A" target-vector)
				  
				  ;;(send *ri* :go-pos (elt target-vector 0) (elt target-vector 1) 0)
				  (setq *target-vector-xy* (float-vector (elt target-vector 0) (elt target-vector 1) 0))
				  ;;(setq *start-to-spot-vector* (float-vector (elt tf-start-point 0) (elt tf-start-point 1) 0))
				  (send *ri* :go-pos 0 0 (degree-vector *start-to-spot-vector* *target-vector-xy*))
				  (cond ((= *speech-recog-res* 2)
					 (progn
					   (send *ri* :speak-jp "これはキッチンです。")
					   ))
					((= *speech-recog-res* 3)
					 (progn
					   (send *ri* :speak-jp "いきましょう。ついてきてください。")
					   (send *ri* :go-pos 2.0 0 0)
					   )))
				  (send *speech-recog-res* 0) 
				  )))))))))))))
    


(setq *speech-recog-res* 0)

(defun speech-recognition(msgs)
  (setq *recognition-word* (car (send msgs :transcript)))
  (print *recognition-word*)
  (setq *stop-words-list* (list "止まって" "待って" "ストップ"))
  (setq *what-is-it-list* (list "これは何ですか" "あれは何ですか" "これは何" "あれは何" "それは何"))
  (setq *where-list* (list "こっちに行きたいです ""あちらに行きたいです" "あっちに行きたいです" "そっちに行きたいです" "こっちに行きたい" "あっちに行きたい" "そっちに行きたい"))
  (cond ((find *recognition-word* *stop-words-list* :test #'equal)
	 (progn
	   (ros::ros-error "true ~A" *recognition-word*)
	   (setq *speech-recog-res* 1)
	  ))
	((find *recognition-word* *what-is-it-list* :test #'equal)
	 (progn
	   (send *ri* :speak-jp "承知しました。何をしりたいか指差しして教えてください。")
	   (setq *speech-recog-res* 2)
	   ))
	((find *recognition-word* *where-list* :test #'equal)
	 (progn
	   (send *ri* :speak-jp "承知しました。どちらに行きたいか指差しして教えてください")
	   (setq *speech-recog-res* 3)
	   )))
  )

(defun vector (a)
  (float-vector (send a :x) (send a :y) (send a :z))
  )

 
(defun compare-distance-of-rects (a b)
  (>= (norm (send (send a :pose) :position)) (norm (send (send a :pose) :position)))
  )

(defun degree-vector (a b)
  (rad2deg (acos (/ (v. a b) (* (norm a) (norm b)))))
  )



(ros::roseus-add-srvs "topic_tools")
(defun call-service-coral (req-topic)
  (ros::wait-for-service "/coral_input/image/select")
  (setq req (instance topic_tools::MuxSelectRequest :init))
  (send req :topic req-topic)
  (ros::service-call "/coral_input/image/select" req)
  )

(ros::roseus-add-srvs "coral_usb")
(defun call-service-edgetpu (req-topic) 
  (ros::wait-for-service "/edgetpu_node_manager/start")
  (setq req-1 (instance coral_usb::StartNodeRequest :init))
  (send req-1 :name req-topic)
  (ros::service-call "/edgetpu_node_manager/start" req-1)
  )

(defun move-to-human-call ()
  (call-service-coral "/dual_fisheye_to_panorama/output")
  (call-service-edgetpu "edgetpu_panorama_object_detector")
  (setq *human* (one-shot-subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray))
  (move-to-human *human*)
  )

(defun recognition-skeleton-call ()
  (ros::ros-error "recog-skeleton-call")
  ;;(call-service-coral "/spot/camera/hand_color/image")
  (call-service-coral "/camera/color/image_raw")
  (print "call-coral")
  (call-service-edgetpu "edgetpu_human_pose_estimator")
  ;;(setq *skeleton* (one-shot-subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray))
  ;;(recognition-skeleton *skeleton*)
  (ros::subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray #'recognition-skeleton)
  )


(defun main ()
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-recognition)
  (move-to-human-call)
  (print "human-call-done")
  (ask)
  (print "ask-done")
  (recognition-skeleton-call)
  (ros::spin)
  )

(main)
  







;;(ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-recognition)

;;(ros::spin)

;;(ros::subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray #'cb)
