#!/usr/bin/env roseus

(ros::load-ros-manifest "dynamic_tf_publisher")

(ros::roseus "test_detect_laundry")

(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://pr2eus/speak.l")

(ros::rate 1)
(pr2-init)
(speak-en "sentakuki")

(while (ros::ok)
  (setq laundry-coords (check-detection :type "laundry_button" :speak-name "sentakuki" :speak nil))
  (if laundry-coords
	  (publish_tf laundry-coords "/laundry"))
  
  ;;(send *laundry* :move-to laundry-coords)
  ;;(send *pr2* :inverse-kinematics (send *laundory* :button-handle))
  (warn "laundry-coords: ~A~%" laundry-coords)
  (ros::spin-once)
  (ros::sleep))


;;; copy from jsk_smart_gui/src/get_template.l
(defun ros::coords->tf-pose-stamped0 (coords id)
  (let ((pose (ros::coords->tf-pose coords)))
    (instance geometry_msgs::PoseStamped :init
              :header (instance std_msgs::header :init
                                :frame_id id
                                :stamp (ros::time 0))
			  :pose pose)))

(defun publish_tf (pos child_frame &key (parent_frame "base_footprint"))
  (let ((settf (instance dynamic_tf_publisher::SetDynamicTFRequest :init)))
    (send settf :freq 10.0)
    (send settf :cur_tf (ros::coords->tf-transform-stamped (make-coords :pos (send pos :pos))
                               parent_frame child_frame))
    (ros::service-call "set_dynamic_tf" settf)
    ))

(defun delete_tf (frame)
  (let ((deltf (instance dynamic_tf_publisher::DeleteTFRequest :init)))
    (send deltf :header :frame_id frame)
    (ros::service-call "delete_tf" deltf)))

