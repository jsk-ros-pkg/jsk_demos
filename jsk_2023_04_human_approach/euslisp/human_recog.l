#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(load "package://spoteus/spot-interface.l")
;;(spot-init)
(ros::roseus "human_recog")

(defun move-to-human (msg)
  ;;(print "move-to-human-do")
  (let (rect-list distance max-rect)
    (setq rect-list (send msg :boxes))
    (if (not (= (length rect-list) 0))
  	(progn
  	  (print "data-start")
  	  (if (= (length rect-list) 1)
  	      (setq max-rect (car rect-list))
  	    )
  	  ;;(setq sorted-list (sort rect-list #'compare-distance-of-rects))
  	  ;;(setq max-rect (car sorted-list))
  	  (setq max-rect (car rect-list))
  	  (setq *x* (send (send (send max-rect :pose) :position) :x))
  	  (setq *y* (send (send (send max-rect :pose) :position) :y))
  	  (setq *angle* (rad2deg (atan *y* *x*)))
  	  (setq *human-to-spot-vector*  (float-vector (* *x* -1) (* *y* -1) 0))
  	  (print *angle*)
  	  ;;;(send *ri* :go-pos 0 0 *angle*)
  	  ;;;(send *ri* :go-pos 1.0 0 0)
  	  )
      )
    )
  )

(defun ask()
  (print "ask-do")
  ;; (send *ri* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 -170.0 150.0 0.0 10.0 45.0 0.0))
  ;; (send *ri* :gripper-open)
  ;; (send *ri* :speak-jp "どちらに行きますか？ また、何かわからないことがありますか?")
  ;; (unix::sleep 7)
  ;; (send *ri* :angle-vector (send *spot* :reset-pose))
  ;; (send *ri* :gripper-close)
)
 
(defun recognition-skeleton (msgs)
  (print "recog-skeleton-do")
  (let (physical-order physical-reverse-order skeleton-list bone-names-list bones index target-bone)
    (setq physical-order (list "right elbow->right wrist" "left elbow->left wrist"))
    (setq physical-reverse-order (list "right wrist->right elbow" "left wrist->left elbow"))
    (ros::ros-info "skeletons ~A" (length (send msgs :skeletons)))
    
    (if (>= (length (send msgs :skeletons)) 1)
  	(progn
  	  (setq skeleton-list (send msgs :skeletons))
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
  				     (setq *bone-start-point* (send target-bone :start_point))
  				     (setq *bone-end-point* (send target-bone :end_point))
  				     ))
  				  ((find bone-name physical-reverse-order)
  				   (progn
  				     (setq *bone-end-point* (send target-bone :start_point))
  				     (setq *bone-start-point* (send target-bone :end_point))
  				     )))
  			    (tf-vector)
  				  (send *ri* :go-pos 0 0 (degree-vector *human-to-spot-vector* *target-vector-xy*))
  			    (switch-mode)
  			    (setq *skeleton-recog-flag* t)
  			    (setq *speech-recog-res* 0)
  			    ))))))))))
  )

(defun switch-mode()
  (cond ((= *speech-recog-res* 2)
	 (progn
	   (send *ri* :speak-jp "これはキッチンです。")
	   (send *ri* :go-pos 0 0 (* -1 (degree-vector *human-to-spot-vector* *target-vector-xy*)))))
	((= *speech-recog-res* 3)
	 (progn
	   (send *ri* :speak-jp "いきましょう。ついてきてください。")
	   (unix::sleep 5)
	   (send *ri* :go-pos 2.0 0 0)
	   )))
  )

(defun tf-vector()
  (let (tf-base-to-arm tf-start-point tf-end-point target-vector)
    (setq *tfl* (instance ros::transform-listener :init))
    (send *tfl* :wait-for-transform "base_link" "hand_color_image_sensor" (ros::time 0) 1)
    (setq tf-base-to-arm (send *tfl* :lookup-transform "base_link" "hand_color_image_sensor" (ros::time 0))) 

    (setq tf-start-point (send tf-base-to-arm :transform-vector (vector *bone-start-point*)))
    (setq tf-end-point (send tf-base-to-arm :transform-vector (vector *bone-end-point*)))
    (print tf-start-point)
    (print tf-end-point)
    (setq target-vector (v- tf-end-point tf-start-point))
  
    (setq *target-vector-xy* (float-vector (elt target-vector 0) (elt target-vector 1) 0))
    )
  )
  

(defun speech-recognition(msgs)
  (setq *recognition-word* (car (send msgs :transcript)))
  (print *recognition-word*)
  (setq *stop-words-list* (list "止まって" "待って" "ストップ" "こっちに来て" "おいで"))
  (setq *what-is-it-list* (list "これは何ですか" "あれは何ですか" "これは何" "あれは何" "それは何"))
  (setq *where-list* (list "こっちに行きたいです ""あちらに行きたいです" "あっちに行きたいです" "そっちに行きたいです" "こっちに行きたい" "あっちに行きたい" "そっちに行きたい"))
  (setq *finish-words-list* (list "終了" "終わり" "ありがとう"))
  (cond ((find *recognition-word* *stop-words-list* :test #'equal)
	 (progn
	   (ros::ros-error "true ~A" *recognition-word*)
	   (setq *speech-recog-res* 1)
	  ))
	((find *recognition-word* *what-is-it-list* :test #'equal)
	 (progn
	   ;;(send *ri* :speak-jp "承知しました。何をしりたいか指差しして教えてください。")
	   (setq *speech-recog-res* 2)
	   (unix::sleep 10)
	   ))
	((find *recognition-word* *where-list* :test #'equal)
	 (progn
	   ;;(send *ri* :speak-jp "承知しました。どちらに行きたいか指差しして教えてください")
	   (setq *speech-recog-res* 3)
	   (unix::sleep 10)
	   ))
	((find *recognition-word* *finish-words-list* :test #'equal)
	 (progn
	   ;;(send *ri* :speak-jp "これにて終了します。お疲れ様でした。")
	   (setq *speech-recog-flag* t)
	   ))
	)
  (ros::ros-error "~A" *speech-recog-res*)
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
  ;; when input image has not come successfully, wait-for-service hang at this point.
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
  (print "call-coral-human")
  (call-service-edgetpu "edgetpu_panorama_object_detector")
  (print "call-edgetpu-human")
  (setq *human* (one-shot-subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray))
  (move-to-human *human*)
  )

(defun recognition-skeleton-call ()
  (ros::ros-error "recog-skeleton-call")
  ;;(call-service-coral "/spot/camera/hand_color/image")
  (call-service-coral "/camera/color/image_raw")
  (print "call-coral-skeleton")
  (call-service-edgetpu "edgetpu_human_pose_estimator")
  ;;(setq *skeleton* (one-shot-subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray))
  ;;(recognition-skeleton *skeleton*)
  (ros::subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray #'recognition-skeleton)
  )


(defun main()
  (initialization-and-move)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-recognition)
  (until *speech-recog-flag*
	 (cond ((= *speech-recog-res* 1)
		(progn
		  ;;(send *ri* :speak-jp "とまります。")
		  ;;(send *ri* :go-pos 0 0 0)
		  (move-to-human-call)
		  ;;(move-to-human)
		  ))
	       ((> *speech-recog-res* 1)
		(progn
		  (until *skeleton-recog-flag*
			 (recognition-skeleton-call)
			 ;;(recognition-skeleton)
			       ;;(ros::unsubscribe "/skeleton_with_depth/output/skeleton")
			 ))))
	 (ros::spin-once)
	 ;;(print "*speech-recog-res*: ~A" *speech-recog-res*) 
	 (ros::sleep))
  )

(defun initialization-and-move()
  (setq *speech-recog-flag* nil)
  (setq *skeleton-recog-flag* nil)
  (setq *speech-recog-res* 0)
  ;;(send *ri* :angle-vector (send *spot* :reset-pose))
  ;;(send *ri* :gripper-close)
  (move-to-human)
  (print "human-call-done")
  ;;(ask)
  ;;(print "ask-done")
  )

(main)




;;(ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-recognition)

;;(ros::spin)

;;(ros::subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray #'cb)
