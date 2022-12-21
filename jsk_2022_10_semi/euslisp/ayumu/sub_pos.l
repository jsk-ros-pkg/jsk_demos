#! usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus "walking")

(ros::ros-info "start")

(defun sub (msg)
  (setq boxes (send msg :boxes))
  (if boxes
    (progn
      ;; (ros::ros-info "subscribe msg [boxes: ~A]" boxes)
      ;; 1人目の情報を取得
      (setq pose (send (elt boxes 0) :pose))
      ;; (ros::ros-info "subscribe msg [pose: ~A]" pose)
      (setq x (send pose :position :x))
      (setq y (send pose :position :y))
      (setq z (send pose :position :z))
      (ros::ros-info "subscribe msg [x: ~A, y: ~A, z: ~A]" x y z))
    (ros::ros-info "no person")))


(ros::subscribe "/rect_array_in_panorama_to_bounding_box_array/bbox_array" jsk_recognition_msgs::BoundingBoxArray #'sub)

(ros::spin)
(exit)
