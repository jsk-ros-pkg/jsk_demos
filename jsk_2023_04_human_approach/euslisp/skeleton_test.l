#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(load "package://spoteus/spot-interface.l")
(ros::roseus "human_recog")

(defun move-to-human (msg)
  (let (rect-list distance max-rect)
    (setq rect-list (send msg :boxes))
    (if (not (= (length rect-list) 0))
        (progn
          ;;(print "data-start")                                                                                                                                                                            
          (if (= (length rect-list) 1)
              (setq max-rect (car rect-list))
            )
          (setq sorted-list (sort rect-list #'compare-distance-of-rects))
          (setq max-rect (car sorted-list))
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


(defun recognition-skeleton (msgs)
  ;;(send *ri* :gripper-open)
  ;;(send *ri* :angle-vector (send *spot* :reset-pose))
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
			    (ros::ros-error "target-bone ~A" bone-name)

                            (setq index (position bone-name bone-names-list :test 'equal))
                            (setq target-bone (elt bones index))

                            (cond ((find target-bone-name physical-order)
                                   (progn
				     (setq start-point (send target-bone :start_point))
                                     (setq end-point (send target-bone :end_point))
                                     (setq *purpose-vector* (v- (vector end-point) (vector start-point)))
				     (print *purpose-vector*)
                                     ))
                                  ((find bone-name physical-reverse-order)
                                   (progn
                                     (setq end-point (send target-bone :start_point))
                                     (setq start-point (send target-bone :end_point))
                                     ;;(setq *purpose-vector* (v- (vector start-point) (vector end-point)))                                                                                                 
                                     ))
                                  )
                            ;;(ros::ros-error "purpose-vector ~A" *purpose-vector*)                                                                                                                         
                            ;;(setq *tfl* (instance ros::transform-listener :init))
                            ;;(send *tfl* :wait-for-transform "base_link" "hand_color_image_sensor" (ros::time 0) 1)
                            ;;(setq tf-base-to-arm (send *tfl* :lookup-transform "base_link" "hand_color_image_sensor" (ros::time 0)))
                            ;;(setq target-vector (send tf-base-to-arm :transform-vector *purpose-vector*))                                                                                                 
                            ;;(send *ri* :angle-vector (send *spot* :head :look-at target-vector))                                                                                                          

                            ;;(setq tf-start-point (send tf-base-to-arm :transform-vector (vector start-point)))
                            ;;(setq tf-end-point (send tf-base-to-arm :transform-vector (vector end-point)))
                            ;;(print tf-start-point)
                            ;;(print tf-end-point)
                            ;;(setq target-vector (v- tf-end-point tf-start-point))
                            ;;(ros::ros-error "target-vector ~A" target-vector)

                            ;;(send *ri* :go-pos (elt target-vector 0) (elt target-vector 1) 0)                                                                                                             
                            ;;(setq *target-vector-xy* (float-vector (elt target-vector 0) (elt target-vector 1) 0))
                            ;;(setq *start-to-spot-vector* (float-vector (elt tf-start-point 0) (elt tf-start-point 1) 0))                                                                                  
                            ;;(send *ri* :go-pos 0 0 (degree-vector *start-to-spot-vector* *target-vector-xy*))                                                                                             
                            )))))))))))

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



;;(call-service-coral "/spot/camera/hand_color/image")                                                                                                                                                      
(call-service-coral "/camera/color/image_raw")
(call-service-edgetpu "edgetpu_human_pose_estimator")
;;(setq *skeleton* (one-shot-subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray))                                                                                    
;;(recognition-skeleton *skeleton*)                                                                                                                                                                         
(ros::subscribe "/skeleton_with_depth/output/skeleton" jsk_recognition_msgs::HumanSkeletonArray #'recognition-skeleton)

(ros::spin)
