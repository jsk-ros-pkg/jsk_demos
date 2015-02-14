(defun insert-primitive-marker
  (&key (shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*) (name "default_name") (description "default_description"))
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type shape-type :action jsk_rviz_plugins::TransformableMarkerOperate::*INSERT* :frame_id "odom_on_ground" :name name :description description))
    (ros::service-call "/transformable_interactive_server/request_marker_operate" req)
    ))

(defun erase-all-primitive-marker
  ()
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type jsk_rviz_plugins::TransformableMarkerOperate::*BOX* :action jsk_rviz_plugins::TransformableMarkerOperate::*ERASEALL*))
    (ros::service-call "/transformable_interactive_server/request_marker_operate" req)
    ))

(defun set-primitive-marker-dimensions
  (&key x y z radius small-radius)
  (let* ((req (instance jsk_interactive_marker::SetMarkerDimensionsRequest :init))
         (req-dim (instance jsk_interactive_marker::MarkerDimensions :init)))
    (when x (send req-dim :x x))
    (when y (send req-dim :y y))
    (when z (send req-dim :z z))
    (when radius (send req-dim :radius radius))
    (when small-radius (send req-dim :small_radius small-radius))
    (send req :dimensions req-dim)
    (ros::service-call "/transformable_interactive_server/set_dimensions" req)
    ))

(defun set-primitive-marker-pose
  (coords frame-id)
  (let* ((req (instance jsk_interactive_marker::SetTransformableMarkerPoseRequest :init))
         (req-pose-st (instance geometry_msgs::PoseStamped :init)))
    (send req-pose-st :header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id frame-id))
    (send req-pose-st :pose (ros::coords->tf-pose coords))
    (send req :pose_stamped req-pose-st)
    (ros::service-call "/transformable_interactive_server/set_pose" req)
    ))

(defun set-primitive-marker-color
  (&key (r 0) (b 0) (g 0) (a 0))
  (let* ((msg (instance std_msgs::ColorRGBA :init :r r :g g :b b :a a)))
    (ros::publish "/transformable_interactive_server/set_color" msg)))
