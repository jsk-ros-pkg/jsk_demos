(ros::load-ros-manifest "roseus")
(ros::roseus "fridge_marker")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "visualization_msgs")
(ros::ros-info "Publish for /objectdetection_marker_array")
(ros::advertise "/fridge_visualization_marker_array" visualization_msgs::MarkerArray 5)
(defun fridge-marker-cb (msg)
  (let* ((obj-lst (send msg :objects))
         (lifetime 3)
         mk-lst)
    (dolist (obj obj-lst)
      (let ((mt (instance visualization_msgs::Marker :init
                          :type visualization_msgs::Marker::*TEXT_VIEW_FACING*
                          :lifetime (ros::time lifetime)
                          :text "fridge(100)"
                          :pose (send obj :pose)
                          :scale (instance geometry_msgs::Vector3 :init
                                           :x 0.2 :y 0.2 :z 0.2)
                          :color (instance std_msgs::ColorRGBA :init
                                           :r 1 :g 0 :b 0 :a 1)
                          :header (instance std_msgs::header :init
                                            :stamp (send msg :header :stamp)
                                            :frame_id (send msg :header :frame_id))))
            (mo (instance visualization_msgs::Marker :init
                          :type visualization_msgs::Marker::*CUBE*
                          :lifetime (ros::time lifetime)
                          :pose (send obj :pose)
                          :scale (instance geometry_msgs::Vector3 :init
                                           :x 0.1 :y 0.1 :z 0.1)
                          :color (instance std_msgs::ColorRGBA :init
                                           :r 0 :g 1 :b 0 :a 0.8)
                          :id 1
                          :header (instance std_msgs::header :init
                                            :stamp (send msg :header :stamp)
                                            :frame_id (send msg :header :frame_id)))))
        (setq mk-lst (append mk-lst (list mt mo)))))
    (ros::publish "/fridge_visualization_marker_array" (instance visualization_msgs::MarkerArray :init :markers  mk-lst))))

(ros::subscribe "/openni/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'fridge-marker-cb))
(ros::rate 100)
;; (do-until-key
;;  (ros::spin-once)
;;  (ros::sleep)
;;  )
(ros::spin)
