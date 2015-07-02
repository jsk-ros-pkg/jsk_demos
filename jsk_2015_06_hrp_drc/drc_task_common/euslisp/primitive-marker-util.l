;; send ;;;;;;;;;;;;;;;;;;;;

(defun insert-primitive-marker
  (&key (shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*) (name "default_name") (description "default_description") (resource "") (use-material t))
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type shape-type :action jsk_rviz_plugins::TransformableMarkerOperate::*INSERT* :frame_id "odom_on_ground" :name name :description description :mesh_resource resource :mesh_use_embedded_materials use-material))
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
  (&key x y z radius small-radius name)
  (let* ((req (instance jsk_interactive_marker::SetMarkerDimensionsRequest :init))
         (req-dim (instance jsk_interactive_marker::MarkerDimensions :init)))
    (when x (send req-dim :x (* 0.001 x)))
    (when y (send req-dim :y (* 0.001 y)))
    (when z (send req-dim :z (* 0.001 z)))
    (when radius (send req-dim :radius (* 0.001 radius)))
    (when small-radius (send req-dim :small_radius (* 0.001 small-radius)))
    (when name (send req :target_name name))
    (send req :dimensions req-dim)
    (ros::service-call "/transformable_interactive_server/set_dimensions" req)
    ))

(defun set-primitive-marker-pose
  (coords frame-id &key name)
  (let* ((req (instance jsk_interactive_marker::SetTransformableMarkerPoseRequest :init))
         (req-pose-st (instance geometry_msgs::PoseStamped :init)))
    (send req-pose-st :header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id frame-id))
    (send req-pose-st :pose (ros::coords->tf-pose coords))
    (when name (send req :target_name name))
    (send req :pose_stamped req-pose-st)
    (ros::service-call "/transformable_interactive_server/set_pose" req)
    ))

(defun set-primitive-marker-color
  (&key (r 0) (b 0) (g 0) (a 0))
  (let* ((msg (instance std_msgs::ColorRGBA :init :r r :g g :b b :a a)))
    (ros::publish "/transformable_interactive_server/set_color" msg)))

(defun set-primitive-marker-control-relative-pose
  (coords)
  (ros::publish "/transformable_interactive_server/set_control_relative_pose" (ros::coords->tf-pose coords)))
   
;; receive ;;;;;;;;;;;;;;;;;;;;

(defun get-primitive-marker-pose
  (&key name frame-id)
  (let* ((req (instance jsk_interactive_marker::GetTransformableMarkerPoseRequest :init))
         pose-st coords)
    (when name (send req :target_name name))
    (setq pose-st (send (ros::service-call "/transformable_interactive_server/get_pose" req) :pose_stamped))
    (setq coords (ros::tf-pose->coords (send pose-st :pose)))
    (when frame-id
      (let ((trans (send *tf* :lookup-transform frame-id
                         (send pose-st :header :frame_id) (send pose-st :header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform frame-id
                            (send pose-st :header :frame_id) (ros::time 0))))
        (setq coords (send trans :transform coords))
        ))
    coords))

(defun get-primitive-marker-dimensions
  (&key name)
  (let* ((req (instance jsk_interactive_marker::GetMarkerDimensionsRequest :init))
         dim)
    (when name (send req :target_name name))
    (setq dim (send (ros::service-call "/transformable_interactive_server/get_dimensions" req) :dimensions))
    (list (* 1000 (send dim :x)) (* 1000 (send dim :y)) (* 1000 (send dim :z))
          (* 1000 (send dim :radius)) (* 1000 (send dim :small_radius)) (send dim :type))
    ))

(defun set-focus-marker-name
  (name)
  (let* ((req (instance jsk_interactive_marker::SetTransformableMarkerFocusRequest :init)))
    (send req :target_name name) 
    (ros::service-call "/transformable_interactive_server/get_dimensions" req)
    ))

(defun get-marker-existence
  (name)
  (let* ((req (instance jsk_interactive_marker::GetTransformableMarkerExistenceRequest :init)) res)
    (send req :target_name name)
    (setq res (ros::service-call "/transformable_interactive_server/get_existence" req))
    (if res (send res :existence) nil)
    ))


;; device ;;;;;;;;;;;;;;;;;;;;

(defun apply-primitive-dimensions-to-midi-device
  (&key name)
  (let* ((dim (get-primitive-marker-dimensions :name name))
         (x (elt dim 0)) (y (elt dim 1)) (z (elt dim 2)) (r (elt dim 3)) (sr (elt dim 4)) (type (elt dim 5))
         (x-max (* 1000 1.5)) (y-max (* 1000 1.5)) (z-max (* 1000 1.5)) (r-max (* 1000 1.0)) (sr-max (* 1000 0.2))
         (x-min  (* 1000 0.01)) (y-min (* 1000 0.01)) (z-min (* 1000 0.01)) (r-min (* 1000 0.01)) (sr-min (* 1000 0.001))
         (msg (instance sensor_msgs::JoyFeedbackArray :init)) msg-array)
    (cond
     ((equal type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*)
      (push (instance sensor_msgs::JoyFeedback :init :id 0 :intensity (/ (- x x-min) (- x-max x-min))) msg-array)
      (push (instance sensor_msgs::JoyFeedback :init :id 1 :intensity (/ (- y y-min) (- y-max y-min))) msg-array)
      (push (instance sensor_msgs::JoyFeedback :init :id 2 :intensity (/ (- z z-min) (- z-max z-min))) msg-array))
     ((equal type jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER*)
      (push (instance sensor_msgs::JoyFeedback :init :id 0 :intensity (/ (- r r-min) (- r-max r-min))) msg-array)
      (push (instance sensor_msgs::JoyFeedback :init :id 1 :intensity (/ (- z z-min) (- z-max z-min))) msg-array))
     ((equal type jsk_rviz_plugins::TransformableMarkerOperate::*TORUS*)
      (push (instance sensor_msgs::JoyFeedback :init :id 0 :intensity (/ (- r r-min) (- r-max r-min))) msg-array)
      (push (instance sensor_msgs::JoyFeedback :init :id 1 :intensity (/ (- sr sr-min) (- sr-max sr-min))) msg-array)))
    (send msg :array msg-array)
    (ros::publish "/midi_config_player/set_feedback" msg)
    ))

(defun publish-coords-list (coords-list frame-id)
  (let ((pose-list-msg (instance geometry_msgs::PoseArray :init)))
    (send pose-list-msg :header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id frame-id))
    (let ((pose-list nil))
      (dotimes (i (length coords-list))
        (push (ros::coords->tf-pose (elt coords-list i)) pose-list)
        )
      (send pose-list-msg :poses pose-list) 
      )
    (ros::publish "/drill_pose_list" pose-list-msg)
    )
  )
