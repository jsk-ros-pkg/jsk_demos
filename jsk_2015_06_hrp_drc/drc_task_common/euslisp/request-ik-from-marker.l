#!/usr/bin/env roseus

(ros::roseus "request_ik_from_marker")

(load "package://jsk_ik_server/euslisp/fullbody-ik-client.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")
(ros::roseus-add-msgs "visualization_msgs")

;; initialization ;;;;;;;;;;;;;;;;;;;;;;
(defun request-ik-from-marker-init
  ()
  ;; select ocs/fc
  (setq *use-robot-interface-flag* (ros::get-param "~use_robot_interface" nil))
  ;; robot
  (setq *robot-name* (string-upcase (unix::getenv "ROBOT")))
  (generate-robot *robot-name* :generate-ri? *use-robot-interface-flag*)
  ;; ros
  (setq *ns* (ros::get-param "~transformable_interactive_server_nodename" "/transformable_interactive_server"))
  (setq *frame-id* (ros::get-param "~default_frame_id" "/odom_on_ground"))
  (setq *assoc-frame-id* (format nil "jsk_model_marker_interface/robot/~a" (send (send *robot* :larm :end-coords :parent) :name)))
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::subscribe (format nil "~a~a" *ns* "/marker_dimensions") jsk_interactive_marker::MarkerDimensions #'generate-object-with-new-dimensions-from-topic)
  (ros::advertise (format nil "~a~a" *ns* "/set_color") std_msgs::ColorRGBA 1)
  (ros::subscribe "/urdf_control_marker/pose" geometry_msgs::PoseStamped #'robot-pose-cb)
  (ros::advertise "urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)
  (ros::advertise "/jsk_model_marker_interface/robot/reset_joint_states_and_root" sensor_msgs::JointState 1)
  (ros::advertise "/midi_config_player/set_feedback" sensor_msgs::JoyFeedbackArray 1)
  (ros::advertise "/inverse_reachability_visible_map" visualization_msgs::Marker 1)
  (ros::subscribe "/solve_ik_command" std_msgs::empty #'solve-ik-command 1)
  (ros::subscribe "/save_obj_command" std_msgs::empty #'save-obj-command 1)
  (ros::subscribe "/send_angle_command" std_msgs::empty #'send-angle-command 1)
  (ros::subscribe "/reach_until_touch_command" std_msgs::empty #'reach-until-touch-command 1)
  (ros::subscribe "/object_menu_command" std_msgs::empty #'object-menu-command 1)
  (ros::subscribe "/robot_menu_command" std_msgs::empty #'robot-menu-command 1)
  (ros::advertise "/object_handle" geometry_msgs::PoseStamped 1)
  (ros::advertise "/object_approach" geometry_msgs::PoseStamped 1)
  (ros::subscribe "/object_handle_variable_pos" std_msgs::Float32 #'set-object-handle-pos-from-variable)
  (ros::subscribe "/object_handle_variable_rot" std_msgs::Float32 #'set-object-handle-rot-from-variable)
  (ros::subscribe "/object_approach_variable" std_msgs::Float32 #'set-object-approach-from-variable)
  (ros::advertise-service "/set_handle_reverse" std_srvs::Empty #'set-handle-reverse)
  (ros::advertise-service "/assoc_object_to_robot" std_srvs::Empty #'assoc-object-to-robot)
  (ros::advertise-service "/dissoc_object_to_robot" std_srvs::Empty #'dissoc-object-to-robot)
  (ros::advertise-service "/set_object_mode_next" std_srvs::Empty #'set-obj-mode-next)
  (ros::advertise-service "/set_ik_mode_next" std_srvs::Empty #'set-ik-mode-next)
  (ros::advertise-service "/get_ik_arm" drc_task_common::GetIKArm #'get-ik-arm)
  (ros::advertise-service "/get_ik_arm_pose" drc_task_common::GetIKArmPose #'get-ik-arm-pose)
  (ros::advertise (format nil "~a~a" *ns* "/set_pose") geometry_msgs::PoseStamped 1)
  (ros::advertise "obj_state_text" jsk_rviz_plugins::OverlayText 1)
  (ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-states-callback)
  ;; object
  (setq *obj-type* nil)
  (setq *obj-x* nil)
  (setq *obj-y* nil)
  (setq *obj-z* nil)
  (setq *obj-r* nil)
  (setq *obj-sr* nil)
  (setq *obj-mode-index* 0)
  (setq *obj-mode-list* (list :autoset :manuset :assoc))
  (setq *ik-mode-index* 0)
  (setq *ik-mode-list* (list :none :reach :release))
  (set-obj-mode)
  (setq *saved-obj-name* "saved_obj")
  (setq *saved-obj* (make-cascoords))
  (erase-saved-object)
  ;; command
  (setq *robot-coords* (make-coords))
  (setq *obj-coords* (make-coords))
  (setq *solve-ik-command-flag* nil)
  (setq *save-obj-command-flag* nil)
  (setq *send-angle-command-flag* nil)
  (setq *reach-until-touch-command-flag* nil)
  (setq *object-menu-command-flag* nil)
  (setq *robot-menu-command-flag* nil)
  (setq *handle-pos-variable* 0)
  (setq *handle-rot-variable* 0)
  (setq *approach-variable* 0)
  (setq *handle-reverse-flag* nil)
  (setq *assoc-obj-flag* nil)
  (setq *assoc-obj-coords* (make-coords))
  ;; ik
  (setq *ik-arm* :rarm)
  (setq *ik-result* nil)
  (init-robot-model)
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_pose"))
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_type"))
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_dimensions"))
  (generate-object-with-new-dimensions-from-service)
  (when (boundp '*irtviewer*) (send *irtviewer* :change-background #f(0.4 0.7 0.5)))
  (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (warning-message 2 "[request-ik] initialize request-ik-from-marker.~%")
  (publish-obj-mode-text)
  )

;; robot model ;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot model initialization
(defun init-robot-model
  ()
  (defvar *end-coords-without-toe*
    (mapcar
     #'(lambda (k)
         (make-cascoords
          :name
          (read-from-string (format nil "~A-end-coords-without-toe" k))
          :parent
          (send (send *robot* k :end-coords :parent) :parent)
          :coords
          (send *robot* k :end-coords :copy-worldcoords)))
     '(:rleg :lleg)))
  (defvar *end-coords-from-hand*
    (mapcar
     #'(lambda (k)
         (send *robot* k :end-coords))
     '(:rarm :larm)))
  )

;; robot model pose calback
(defun robot-pose-cb
  (msg)
  (setq *robot-coords* (ros::tf-pose-stamped->coords msg)))

;; set current position
(defun set-robot-current-pos
  ()
  (let* ((current-pose-header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id *frame-id*))
         (current-pose-msg (instance geometry_msgs::PoseStamped :init :header current-pose-header))
         current-coords
         (frame "ground") (ts (ros::time-now)))
    (unless (send *tfl* :wait-for-transform frame *frame-id* ts 1) (return-from set-robot-current-pos nil))
    (setq current-coords (send (send *tfl* :lookup-transform frame *frame-id* ts) :inverse-transformation))
    (send current-pose-msg :pose (ros::coords->tf-pose current-coords))
    (ros::publish "urdf_control_marker/set_pose" current-pose-msg)
    ))

;; set obj pose for assoc
(defun set-obj-pose-for-assoc
  ()
  (let* ((obj-header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id *frame-id*))
         obj-coords obj-msg)
    (let* ((ts (ros::time-now)))
      (unless (send *tfl* :wait-for-transform *assoc-frame-id* *frame-id* ts 1) (return-from set-obj-pose-for-assoc nil))
      (setq obj-coords
            (send (send (send *tfl* :lookup-transform *assoc-frame-id* *frame-id* ts) :inverse-transformation) :transform *assoc-obj-coords* :local)))
    (setq obj-msg (instance geometry_msgs::PoseStamped :init :header obj-header :pose (ros::coords->tf-pose obj-coords)))
    (ros::publish (format nil "~a~a" *ns* "/set_pose") obj-msg)
    ))

;; get base coords for assoc obj
(defun get-assoc-obj-coords
  (&key (robot-coords *robot-coords*))
  (let* ((ts (ros::time-now)))
    (unless (send *tfl* :wait-for-transform *assoc-frame-id* *frame-id* ts 1) (return-from get-assoc-obj-coords nil))
    (setq *assoc-obj-coords*
          (send (send (send *tfl* :lookup-transform *assoc-frame-id* *frame-id* ts) :inverse-transformation)
                :transformation (get-obj-coords :robot-coords robot-coords :relative-to-robot nil) :local))))

;; object model ;;;;;;;;;;;;;;;;;;;;;;;;;
;; get object model pose relative to robot
(defun get-obj-coords
  (&key (relative-to-robot t) (saved-object nil) (robot-coords *robot-coords*))
  (let* ((get-pose-req (instance jsk_interactive_marker::GetTransformableMarkerPoseRequest :init))
         get-pose-res
         obj-coords)
    (when saved-object
      (send get-pose-req :target_name *saved-obj-name*))
    (setq get-pose-res (ros::service-call (format nil "~a~a" *ns* "/get_pose") get-pose-req))
    (setq *obj-coords* (ros::tf-pose->coords (send (send get-pose-res :pose_stamped) :pose)))
    (setq obj-coords (send *obj-coords* :transformation robot-coords :local))
    (setq obj-coords (send obj-coords :transformation (make-coords) :local))
    (if relative-to-robot obj-coords *obj-coords*)
    )
  )

;; update object pose
(defun update-object-pose
  (&key (robot-coords *robot-coords*))
  (let ((obj-coords (send (get-obj-coords :robot-coords robot-coords) :copy-worldcoords)))
    (send *obj* :newcoords
          (send obj-coords :transform (send (send (send (find-if #'(lambda (x) (equal (send x :name) :origin)) (send *obj* :descendants)) :coords) :copy-worldcoords) :inverse-transformation) :local)) ;; :coords is correct. not :worldcoords.
    ))

;; generate object model
(defun generate-object-with-new-dimensions
  (&key type dimensions)
  (when dimensions
    (setq *obj-x* (* (send dimensions :x) 1000))
    (setq *obj-y* (* (send dimensions :y) 1000))
    (setq *obj-z* (* (send dimensions :z) 1000))
    (setq *obj-r* (* (send dimensions :radius) 1000))
    (setq *obj-sr* (* (send dimensions :small_radius) 1000)))
  (when type
    (setq *obj-type* type))
  (cond ((equal *obj-type* jsk_rviz_plugins::TransformableMarkerOperate::*BOX*)
         (setq *obj* (make-cube *obj-x* *obj-y* *obj-z*))
         (send *obj* :assoc (make-cascoords :name :origin))
         (let* ((handle-coords (make-coords :pos (float-vector 0 0 (- (/ *obj-z* 2.0)))))
                (handle-variable-coords (make-coords :pos (float-vector 0 0 (* *obj-z* *handle-pos-variable*))))
                approach-coords)
           (send handle-coords :transform handle-variable-coords :world)
           (send handle-coords :rotate (* 2 pi *handle-rot-variable*) :z :local)
           (send *obj* :assoc (make-cascoords :coords handle-coords :name :handle))
           (setq approach-coords (send handle-coords :copy-worldcoords))
           (send approach-coords :translate (float-vector (* 1000 *approach-variable*) 0 0) :local)
           (send *obj* :assoc (make-cascoords :coords approach-coords :name :approach))
           )
         (send *obj* :put :type :box)
         )
        ((equal *obj-type* jsk_rviz_plugins::TransformableMarkerOperate::*CYLINDER*)
         (setq *obj* (make-cylinder (* *obj-r* 0.5) *obj-z*))
         (send *obj* :assoc (make-cascoords :pos (float-vector 0 0 (* *obj-z* 0.5)) :name :origin))
         (let* ((handle-coords (make-coords))
                (handle-variable-coords (make-coords :pos (float-vector 0 0 (* *obj-z* *handle-pos-variable*))))
                approach-coords)
           (send handle-coords :transform handle-variable-coords :world)
           (send handle-coords :rotate (* 2 pi *handle-rot-variable*) :z :local)
           (send *obj* :assoc (make-cascoords :coords handle-coords :name :handle))
           (setq approach-coords (send handle-coords :copy-worldcoords))
           (send approach-coords :translate (float-vector (* 1000 *approach-variable*) 0 0) :local)
           (send *obj* :assoc (make-cascoords :coords approach-coords :name :approach)))
         (send *obj* :put :type :cylinder)
         )
        ((equal *obj-type* jsk_rviz_plugins::TransformableMarkerOperate::*TORUS*)
         (setq *obj* (make-ring *obj-r* *obj-sr* :segments 16))
         (send *obj* :assoc (make-cascoords :name :origin))
         (let* ((handle-coords (make-coords :pos (float-vector *obj-r* 0 0) :rpy (list -pi/2 pi/2 pi)))
                (handle-variable-coords (make-coords :rpy (list (+ (* 2 pi *handle-pos-variable*) pi/2)  0 0)))
                approach-coords)
           (send handle-coords :transform handle-variable-coords :world)
           (send handle-coords :rotate (* 2 pi *handle-rot-variable*) :z :local)
           (send *obj* :assoc (make-cascoords :coords handle-coords :name :handle))
           (setq approach-coords (send handle-coords :copy-worldcoords))
           (send approach-coords :translate (float-vector (* 1000 *approach-variable*) 0 0) :local)
           (send *obj* :assoc (make-cascoords :coords approach-coords :name :approach)))
         (send *obj* :put :type :torus)
         )
        ((equal *obj-type* jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE*) ;; hand mesh
         (setq *obj* (make-cube 100 100 100)) ;; dummy eus object instead of hand mesh
         (let* (hand-trans
                hand-origin-trans-pos)
           (cond ((or (string= *robot-name* "HRP2JSKNTS") (string= *robot-name* "HRP2JSKNT")  (string= *robot-name* "STARO"))
                  (setq hand-origin-trans-pos (float-vector 0 0 0)))
                 ((string= *robot-name* "JAXON")
                  (setq hand-origin-trans-pos (float-vector 0 0 -40))))
           (setq hand-trans
                 (send (send (send (send (send *robot* *ik-arm* :end-coords) :parent) :copy-worldcoords)
                             :translate hand-origin-trans-pos :local)
                       :transformation (send (send *robot* *ik-arm* :end-coords) :copy-worldcoords)))
           (send *obj* :assoc (make-cascoords :coords (make-coords) :name :origin))
           ;; :handle and :approach is fixed to end-coords
           (send *obj* :assoc (make-cascoords :coords hand-trans :name :handle))
           (send *obj* :assoc (make-cascoords :coords hand-trans :name :approach))
           )
         (send *obj* :put :type :hand-mesh)
         )
        )
  (when *handle-reverse-flag*
    (send (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)) :rotate pi :x))
  (when *handle-reverse-flag*
    (send (find-if #'(lambda (x) (equal (send x :name) :approach)) (send *obj* :descendants)) :rotate pi :x))
  (send *obj* :set-color #f(1 1 0))
  )
(defun generate-object-with-new-dimensions-from-topic
  (msg)
  (generate-object-with-new-dimensions :type (send msg :type) :dimensions msg)
  (when (boundp '*irtviewer*) (objects (list *obj* *robot*)))
  )
(defun generate-object-with-new-dimensions-from-service
  ()
  (let* (get-type-res get-dim-res type dimensions)
    (setq get-type-res
          (ros::service-call (format nil "~a~a" *ns* "/get_type") (instance jsk_interactive_marker::GetTypeRequest :init)))
    (setq get-dim-res
          (ros::service-call (format nil "~a~a" *ns* "/get_dimensions") (instance jsk_interactive_marker::GetMarkerDimensionsRequest :init)))
    (generate-object-with-new-dimensions :type (send get-type-res :type) :dimensions (send get-dim-res :dimensions)))
  )

;; publish handle pose
(defun publish-object-handle
  (&key frame-id)
  (let* ((handle-header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id frame-id))
         (handle-coords
          (send (send (find-if #'(lambda (x) (equal (send x :name) :origin)) (send *obj* :descendants)) :coords) :transformation  (send (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)) :coords)))
         (handle-pose (ros::coords->tf-pose handle-coords))
         (handle-msg (instance geometry_msgs::PoseStamped :init :header handle-header :pose handle-pose)))
    (ros::publish "/object_handle" handle-msg))
  )

(defun publish-object-approach
  (&key frame-id)
  (let* ((handle-header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id frame-id))
         (handle-coords
          (send (send (find-if #'(lambda (x) (equal (send x :name) :origin)) (send *obj* :descendants)) :coords) :transformation  (send (find-if #'(lambda (x) (equal (send x :name) :approach)) (send *obj* :descendants)) :coords)))
         (handle-pose (ros::coords->tf-pose handle-coords))
         (handle-msg (instance geometry_msgs::PoseStamped :init :header handle-header :pose handle-pose)))
    (ros::publish "/object_approach" handle-msg))
  )

;; handle variable
(defun set-object-handle-pos-from-variable
  (msg)
  (setq *handle-pos-variable* (send msg :data))
  )
(defun set-object-handle-rot-from-variable
  (msg)
  (setq *handle-rot-variable* (send msg :data))
  )

(defun set-object-approach-from-variable
  (msg)
  (setq *approach-variable* (send msg :data))
  )

;; handle reverse
(defun set-handle-reverse
  (req)
  (let* ((res (send req :response)))
    (setq *handle-reverse-flag* (not *handle-reverse-flag*))
    (generate-object-with-new-dimensions)
    res))
  
(defun save-obj-model-one
  (&key (robot-coords *robot-coords*))
  (when *obj-type*
    (let* (original-name
           (marker-operate-req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init))
           (set-marker-color-req (instance jsk_interactive_marker::SetTransformableMarkerColorRequest :init))
           (get-marker-focus-req (instance jsk_interactive_marker::GetTransformableMarkerFocusRequest :init))
           (set-marker-focus-req (instance jsk_interactive_marker::SetTransformableMarkerFocusRequest :init))
           )
      ;; restore focus
      (setq original-name (send (ros::service-call (format nil "~a~a" *ns* "/get_focus") get-marker-focus-req) :target_name))
      ;; erase
      (erase-saved-object)
      ;; copy and insert
      (send marker-operate-req :operate :action jsk_rviz_plugins::TransformableMarkerOperate::*COPY*)
      (send marker-operate-req :operate :name *saved-obj-name*)
      (send marker-operate-req :operate :description "")
      (ros::service-call (format nil "~a~a" *ns* "/request_marker_operate") marker-operate-req)
      ;; set color
      (send set-marker-color-req :target_name *saved-obj-name*)
      (send set-marker-color-req :color :r 0.8)
      (send set-marker-color-req :color :g 0.8)
      (send set-marker-color-req :color :b 0.8)
      (send set-marker-color-req :color :a 0.8)
      (ros::service-call (format nil "~a~a" *ns* "/set_color") set-marker-color-req)
      ;; move focus
      (send set-marker-focus-req :target_name original-name)
      (ros::service-call (format nil "~a~a" *ns* "/set_focus") set-marker-focus-req)
  ))
  ;; save object coords
  (generate-object-with-new-dimensions-from-service)
  (update-object-pose :robot-coords robot-coords)
  (setq *saved-obj* (make-cascoords :coords (send (send *obj* :worldcoords) :copy-worldcoords)))
  (send *saved-obj* :assoc
        (make-cascoords :coords (send (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)) :copy-worldcoords) :name :handle))
  (setq *save-obj-command-flag* nil)
  )

(defun erase-saved-object
  ()
  (let* ((marker-operate-req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send marker-operate-req :operate :action jsk_rviz_plugins::TransformableMarkerOperate::*ERASE*)
    (send marker-operate-req :operate :name *saved-obj-name*)
    (ros::service-call (format nil "~a~a" *ns* "/request_marker_operate") marker-operate-req)))
  
;; obj mode ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun set-obj-mode-next
  (req)
  (let* ((res (send req :response)))
    (setq *obj-mode-index* (+ *obj-mode-index* 1))
    (when (= *obj-mode-index* (length *obj-mode-list*))
      (setq *obj-mode-index* 0))
    (set-obj-mode)
    (publish-obj-mode-text)
    res))

(defun set-ik-mode-next
  (req)
  (let* ((res (send req :response)))
    (setq *ik-mode-index* (+ *ik-mode-index* 1))
    (when (= *ik-mode-index* (length *ik-mode-list*))
      (setq *ik-mode-index* 0))
    (publish-obj-mode-text)
    res))
(defun get-ik-arm
  (req)
  (let* ((res (send req :response)))
    (send res :ik_arm (if (equal *ik-arm* :rarm) ":rarm" ":larm"))
    res))
(defun get-ik-arm-pose
  (req)
  (let* ((res (send req :response)) (original-av (send *robot* :angle-vector)) ik-arm-coords)
    (setq ik-arm-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
    (send res :pose_stamped (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose (send (send (send (send *robot* :body_lk) :worldcoords) :copy-worldcoords) :transformation ik-arm-coords :local)) :header (instance std_msgs::Header :init :frame_id "BODY" :stamp (ros::time-now))))
    (send *robot* :angle-vector original-av)
    res))

(defun set-obj-mode
  ()
  (cond ((equal (elt *obj-mode-list* *obj-mode-index*) :autoset)
         (ros::service-call "enable_auto_set_mode" (instance std_srvs::EmptyRequest :init))
         (setq *assoc-obj-flag* nil)
         (ros::publish (format nil "~a~a" *ns* "/set_color") (instance std_msgs::ColorRGBA :init :r 1.0 :g 1.0 :b 0.0 :a 0.6))
         ;;(ros::service-call "call_obj_state_event" (instance drc_task_common::StringRequestRequest :init :data "AutoSet"))
         )
        ((equal (elt *obj-mode-list* *obj-mode-index*) :manuset)
         (ros::service-call "disable_auto_set_mode" (instance std_srvs::EmptyRequest :init))
         (setq *assoc-obj-flag* nil)
         (ros::publish (format nil "~a~a" *ns* "/set_color") (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 1.0 :a 0.6))
         ;;(ros::service-call "call_obj_state_event" (instance drc_task_common::StringRequestRequest :init :data "ManuSet"))
         )
        ((equal (elt *obj-mode-list* *obj-mode-index*) :assoc)
         (ros::service-call "disable_auto_set_mode" (instance std_srvs::EmptyRequest :init))
         (get-assoc-obj-coords)
         (setq *assoc-obj-flag* t)
         (ros::publish (format nil "~a~a" *ns* "/set_color") (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 1.0 :a 0.6))
         ;; (ros::service-call "call_obj_state_event" (instance drc_task_common::StringRequestRequest :init :data "Assoc"))         
         ))
  )

(defun publish-obj-mode-text
  ()
  (let* (obj-mode-text
         ik-mode-text
         exec-mode-text
         (text-msg (instance jsk_rviz_plugins::OverlayText :init)))
    (cond ((equal (elt *obj-mode-list* *obj-mode-index*) :autoset)
           (setq obj-mode-text "AutoSet"))
          ((equal (elt *obj-mode-list* *obj-mode-index*) :manuset)
           (setq obj-mode-text "ManuSet"))
          ((equal (elt *obj-mode-list* *obj-mode-index*) :assoc)
           (setq obj-mode-text "Assoc")))
    (cond ((equal (elt *ik-mode-list* *ik-mode-index*) :none)
           (setq ik-mode-text "None"))
          ((equal (elt *ik-mode-list* *ik-mode-index*) :reach)
           (setq ik-mode-text "Reach"))
          ((equal (elt *ik-mode-list* *ik-mode-index*) :release)
           (setq ik-mode-text "Release")))
    (send text-msg :text (format nil "Mode: ~a / ~a / ~a" obj-mode-text (string *ik-arm*) ik-mode-text))
    (send text-msg :top 70)
    (send text-msg :left 10)
    (send text-msg :width 750)
    (send text-msg :height 50)
    (send text-msg :bg_color :r 0.9)
    (send text-msg :bg_color :b 0.9)
    (send text-msg :bg_color :g 0.9)
    (send text-msg :bg_color :a 0.1)
    (send text-msg :fg_color :r 0.2)
    (send text-msg :fg_color :g 0.7)
    (send text-msg :fg_color :b 0.7)
    (send text-msg :fg_color :a 1)
    (send text-msg :line_width 1)
    (send text-msg :text_size 30)
    (ros::publish "obj_state_text" text-msg))
  )

;; callback ;;;;;;;;;;;;;;;;;;;;;;;;;
;; service server callback for assoc (not used now)
(defun assoc-object-to-robot
  (req)
  (let* ((res (send req :response)))
    (get-assoc-obj-coords)
    (setq *assoc-obj-flag* t)
    res))
(defun dissoc-object-to-robot
  (req)
  (let* ((res (send req :response)))
    (setq *assoc-obj-flag* nil)
    res))

;; ik
(defun solve-ik-command
  (msg)
  (setq *solve-ik-command-flag* t))
;; save-obj
(defun save-obj-command
  (msg)
  (setq *save-obj-command-flag* t))
;; angle-vector
(defun send-angle-command
  (msg)
  (setq *send-angle-command-flag* t))
(defun reach-until-touch-command
  (msg)
  (setq *reach-until-touch-command-flag* t))
;; object menu
(defun object-menu-command
  (msg)
  (setq *object-menu-command-flag* t))
;; robot menu
(defun robot-menu-command
  (msg)
  (setq *robot-menu-command-flag* t))

;; ik ;;;;;;;;;;;;;;;;;;;;;;;;;
;; ik one
(defun request-ik-from-marker-one
  (&key (initial-pose :reset-manip-pose)
        (robot-coords *robot-coords*))
  (let* (original-av original-coords interpolate-mode)
    (warning-message 2 "[request-ik] send solve-ik service.~%")
    (when (equal (elt *obj-mode-list* *obj-mode-index*) :assoc)
      (warning-message 1 "[request-ik] do not call solve-ik in assoc obj-mode.~%")
      (setq *solve-ik-command-flag* nil)
      (return-from request-ik-from-marker-one nil))
    (setq original-av (send *robot* :angle-vector))
    (setq original-coords (send (send *robot* :worldcoords) :copy-worldcoords))
    ;; regenerate object and move coords
    (when initial-pose (send *robot* initial-pose))
    (send *robot* :fix-leg-to-coords (make-coords))
    (generate-object-with-new-dimensions-from-service)
    (update-object-pose :robot-coords robot-coords)
    ;; check mode 
    (let* ((get-marker-existence-req (instance jsk_interactive_marker::GetTransformableMarkerExistenceRequest :init)))
      (send get-marker-existence-req :target_name *saved-obj-name*)
      (when (send (ros::service-call (format nil "~a~a" *ns* "/get_existence") get-marker-existence-req) :existence)
        (setq interpolate-mode :save)))
    (unless (equal (elt *ik-mode-list* *ik-mode-index*) :none)
      (setq interpolate-mode :approach))
    ;; solve ik
    (setq *ik-result* (send-ik-request :interpolate-mode interpolate-mode :rotation-axis (if (equal (send *obj* :get :type) :hand-mesh) (list t t t) (list :z t t))))
    ;(if *pub-grasp-pose-feedback-flag* (pub-grasp-pose-feedback))
    (cond (*ik-result*
           (send-joint-states-to-marker))
          (t
           (send *robot* :angle-vector original-av)
           (send *robot* :newcoords original-coords)
           ))
    (setq *solve-ik-command-flag* nil)
    *ik-result*
    ))

;; send ik service request
(defun send-ik-request
  (&key 
   (interpolate-mode)
   (target-coords (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
   (initial-coords target-coords)
   (arm *ik-arm*)
   (rotation-axis (list :z t t)))
  (cond ((equal interpolate-mode :save)
         (setq initial-coords (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *saved-obj* :descendants)))
         )
        ((equal interpolate-mode :approach)
         (setq initial-coords (find-if #'(lambda (x) (equal (send x :name) :approach)) (send *obj* :descendants)))
         (cond ((equal (elt *ik-mode-list* *ik-mode-index*) :reach)
                )
               ((equal (elt *ik-mode-list* *ik-mode-index*) :release)
                (let (tmp)
                  (setq tmp (send initial-coords :copy-worldcoords))
                  (setq initial-coords target-coords)
                  (setq target-coords tmp))
                ))))
  (let* (ik-res
         ik-angle-list
         (divide-num (if interpolate-mode 5 1)))
    ;; service call for task state manager
    (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "SolveIK"))
    ;; loop for interpolation
    (dotimes (i divide-num)
      (let* ((arm-index (if (equal arm :rarm) 0 1))
             (mt (append (list (elt *end-coords-from-hand* arm-index)) *end-coords-without-toe*))
             (tc (send-all mt :copy-worldcoords))
             (ll (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) mt))
             (translation-axis (list t t t))
             res)
        (setf (elt tc 0) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords target-coords) :copy-worldcoords))
        (cond ((equal arm :rarm)
               (setq *assoc-frame-id* (format nil "jsk_model_marker_interface/robot/~a" (send (send *robot* :rarm :end-coords :parent) :name))))
              ((equal arm :larm)
               (setq *assoc-frame-id* (format nil "jsk_model_marker_interface/robot/~a" (send (send *robot* :larm :end-coords :parent) :name)))))
        ;; service call for ik server
        (cond ((or (string= *robot-name* "HRP2JSKNTS") (string= *robot-name* "HRP2JSKNT"))
               (setq res (ik-server-call
                          :ik-server-service-name "/solve_ik"
                          :target-coords tc
                          :move-target mt
                          :link-list ll
                          :debug-view :no-message
                          :translation-axis translation-axis
                          :rotation-axis rotation-axis
                          :max (float-vector  500  500  0  20  20  10)
                          :warnp t
                          :stop 30
                          :look-at-target (copy-seq (send (elt tc 0) :worldpos))
                          )))
              ((or (string= *robot-name* "JAXON") (string= *robot-name* "STARO"))
               (setq res (ik-server-call
                          :ik-server-service-name "/solve_ik"
                          :target-coords tc
                          :move-target mt
                          :link-list ll
                          :debug-view :no-message
                          :translation-axis translation-axis
                          :rotation-axis rotation-axis
                          :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.01 0.15 0.5 0.5)
                          :additional-weight-list
                          (list (list (send *robot* :torso :waist-r :child-link) 0.1)
                                (list (send *robot* :torso :waist-y :child-link) 0.1)
                                (list (send *robot* :torso :waist-p :child-link) 0.1))
                          :max (float-vector  500  500  0  20  20  10)
                          :warnp t
                          :stop 50
                          :avoid-collision-distance 100
                          :avoid-collision-null-gain 5.0
                          :avoid-collision-joint-gain 0.8
                          :collision-avoidance-link-pair
                          (list (list (elt (send *robot* arm :links) 2) (send *robot* :link "CHEST_LINK2")))
                          :look-at-target (copy-seq (send (elt tc 0) :worldpos))
                          ))))
        (setq ik-res (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*))
        (unless ik-res (return))
        (push (send-all (send *robot* :joint-list) :joint-angle) ik-angle-list)
        )
      )
    ;; service call for task state manager
    (cond (ik-res
           (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "SuccessIK")))
          (t
           (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FailIK"))))
    (and ik-res (reverse ik-angle-list)))
  )

;; send joint angle to robot marker
(defun send-joint-states-to-marker
  ()
  (dotimes (i (length *ik-result*))
    (let* ((msg (instance sensor_msgs::JointState :init)))
      (send msg :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
      (send msg :name (send-all (send *robot* :joint-list) :name))
      (send msg :position (mapcar #'deg2rad (elt *ik-result* i)))
      (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg)
      (ros::spin-once))
    (when (= i (- (length *ik-result*) 1))
      (return))
    (unix::usleep (* 200 1000)))
  )

(defun send-joint-states-to-marker-from-angle-vector
  (&optional (av (send *robot* :angle-vector)))
  (let* ((msg (instance sensor_msgs::JointState :init)))
    (send msg :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
    (send msg :name (send-all (send *robot* :joint-list) :name))
    (send msg :position (mapcar #'deg2rad (concatenate cons av)))
    (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg)
    (ros::spin-once))
  )

;; angle-vector ;;;;;;;;;;;;;;;;;;;;;;;;;
(defun send-angle-to-real-robot-one
  ()
  (warning-message 2 "[request-ik] send angle-vector to real robot.~%")
  (let* ((title (format nil "Send angle-vector to real robot. Time : ~~6,0f [msec]~%"))
         (menu-list (cond (*ik-result* (list "cancel" "yes"))
                          (t (list "cancel" "<<IK Fail>> yes <<IK Fail>>"))))
         (yes-index 1)
         (min-int-time 1000)
         (max-int-time 20000)
         (menu-variable-initial 0.5)
         (req (instance drc_task_common::RvizMenuCallRequest :init :title title :menu_list menu-list :variable_max max-int-time :variable_min min-int-time :variable menu-variable-initial))
         (menu-variable-slider-id 7)
         (joy-feedback-msg
          (instance sensor_msgs::JoyFeedbackArray :init :array
                    (list (instance sensor_msgs::JoyFeedback :init :id menu-variable-slider-id :intensity menu-variable-initial))))
         res res-index
         int-time)
    (ros::publish "/midi_config_player/set_feedback" joy-feedback-msg)
    (setq res (ros::service-call "rviz_menu_call" req))
    (setq int-time (send res :variable_ret))
    (setq res-index (send res :index))
    (when (equal res-index yes-index)
      ;; service call for task state manager
      (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "ExecAngleVec"))
      ;; send angle-vector command
      (cond
       (*ik-result*
        (dotimes (i (length *ik-result*))
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* i))
          (cond
           ;; interpolation mode and first time
           ((and (= i 0) (> (length *ik-result*) 1))
            (cond (*use-robot-interface-flag*
                   (send *ri* :angle-vector (send *robot* :angle-vector) (float int-time)))
                  (t
                   (send-angle-vector-from-ocs (send *robot* :angle-vector) (float int-time))))
            (unix::usleep (round (* (* int-time 1000) 0.8)))
            )
           (t
            (cond (*use-robot-interface-flag*
                   (send *ri* :angle-vector (send *robot* :angle-vector) (/ (float int-time) (length *ik-result*))))
                  (t
                   (send-angle-vector-from-ocs (send *robot* :angle-vector) (/ (float int-time) (length *ik-result*)))))
            (cond 
	     ((= i (- (length *ik-result*) 1))
              (cond (*use-robot-interface-flag*
                     (send *ri* :wait-interpolation))
                    (t
                     ;;(unix::usleep (round (* (/ (float int-time) (length *ik-result*)) 1000)))
                     (wait-interpolation-from-ocs)
                     ))
              )
	     (t
              (unix::usleep (round (* (* (/ (float int-time) (length *ik-result*)) 1000) 0.8)))))
            ))
          ))
       )
      ;; service call for task state manager
      (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FinishAngleVec"))
      ))
  (setq *send-angle-command-flag* nil)
  (warning-message 2 "[request-ik] finished sending angle-vector.~%")
  )

;; reach until touch ;;;;;;;;;;;;;;;;;;;;;;;;;
(defun reach-until-touch-one
  ()
  (warning-message 2 "[request-ik] started reach-until-touch.~%")
  (let* ((title (format nil "Reach until touch. Force thre : ~~4,0f [N]~%"))
         (menu-list (list "cancel" "yes"))
         (yes-index 1)
         (min-force-thre 0)
         (max-force-thre 50)
         (menu-variable-initial 0.1)
         (req (instance drc_task_common::RvizMenuCallRequest :init :title title :menu_list menu-list :variable_max max-force-thre :variable_min min-force-thre :variable menu-variable-initial))
         (menu-variable-slider-id 7)
         (joy-feedback-msg
          (instance sensor_msgs::JoyFeedbackArray :init :array
                    (list (instance sensor_msgs::JoyFeedback :init :id menu-variable-slider-id :intensity menu-variable-initial))))
         res res-index
         force-thre)
    (ros::publish "/midi_config_player/set_feedback" joy-feedback-msg)
    (setq res (ros::service-call "rviz_menu_call" req))
    (setq force-thre (send res :variable_ret))
    (setq res-index (send res :index))
    (when (equal res-index yes-index)
      ;; service call for task state manager
      (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "ExecReachUntilTouch"))
      ;; send reach-until-touch command
      (let* (dir
             max-dist
             (original-av (send *robot* :angle-vector))
             (original-coords (send (send *robot* :worldcoords) :copy-worldcoords))
             current-eef-pose
             target-eef-pose
             )
        (update-object-pose)
        (cond (*use-robot-interface-flag*
               (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
              (t
               (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
        (send *robot* :fix-leg-to-coords (make-coords))
        (setq current-eef-pose
              (send (send *robot* *ik-arm* :end-coords) :copy-worldcoords))
        (setq target-eef-pose
              (send (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)) :copy-worldcoords))
        (setq dir (v- (send target-eef-pose :worldpos) (send current-eef-pose :worldpos)))
        (setq max-dist (norm dir))
        (warning-message 1 "dir ~a  force-thre ~a  max-dist ~a~%" dir force-thre max-dist)
        (cond (*use-robot-interface-flag*
               (send *ri* :reach-until-touch
                     *ik-arm* dir :thre force-thre :max-dist max-dist :coords-system :world)
               )
              (t
               (reach-until-touch-from-ocs
                *ik-arm* dir :thre force-thre :max-dist max-dist)
               ))
        (send *robot* :angle-vector original-av)
        (send *robot* :newcoords original-coords)
        )
      ;; service call for task state manager
      (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FinishReachUntilTouch"))
      ))
  (setq *reach-until-touch-command-flag* nil)
  (warning-message 2 "[request-ik] finished reach-until-touch.~%")
  )

;; obj menu ;;;;;;;;;;;;;;;;;;;;;;;;;
(defun object-menu-one
  ()
  (let* ((title
          (format nil "Object Menu~%"))
         (menu-list (list "cancel" "reverse"))
         (reverse-index 1)
         (req (instance drc_task_common::RvizMenuCallRequest :init :title title :menu_list menu-list))
         res res-index res-value)
    (setq res (ros::service-call "rviz_menu_call" req))
    (setq res-index (send res :index))
    (setq res-value (elt menu-list res-index))
    (cond ((= res-index reverse-index)
           (setq *handle-reverse-flag* (not *handle-reverse-flag*))
           (generate-object-with-new-dimensions)))
    (ros::spin-once)
    (setq *object-menu-command-flag* nil)
    ))

;; robot menu ;;;;;;;;;;;;;;;;;;;;;;;;;
(defun robot-menu-one
  ()
  (let* ((title
          (format nil "Robot Menu~%"))
         (menu-list (list "cancel" "switch arm" "current pos" "potentio-vector" ":reset-pose" ":reset-manip-pose" ":init-pose"))
         (switch-arm-index 1)
         (current-pos-index 2)
         (potentio-vector-index 3)
         (req (instance drc_task_common::RvizMenuCallRequest :init :title title :menu_list menu-list))
         res res-index res-value)
    (setq res (ros::service-call "rviz_menu_call" req))
    (setq res-index (send res :index))
    (setq res-value (elt menu-list res-index))
    (cond ((equal res-index switch-arm-index)
           (cond ((equal *ik-arm* :rarm) (setq *ik-arm* :larm))
                 ((equal *ik-arm* :larm) (setq *ik-arm* :rarm)))
           (publish-obj-mode-text))
          ((equal res-index current-pos-index)
           (set-robot-current-pos))
          ((equal res-index potentio-vector-index)
           (cond (*use-robot-interface-flag*
                  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
                 (t
                  (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
           (send-joint-states-to-marker-from-angle-vector)
           )
          ((find-method *robot* (read-from-string res-value))
           (eval (list 'send '*robot* '(read-from-string res-value)))
           (setq *ik-result* (list (send-all (send *robot* :joint-list) :joint-angle)))
           (send-joint-states-to-marker)))
    (ros::spin-once)
    (setq *robot-menu-command-flag* nil)
    (publish-obj-mode-text)
    ))

;; main ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(request-ik-from-marker-init)
(ros::rate 10)
(while t
  (ros::sleep)
  (ros::spin-once)
  (when (boundp '*irtviewer*) (x::window-main-one))
  ;; publish handle and approach pose
  (when (and *obj* (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
    (let* (frame-id)
      (cond ((equal (send *obj* :get :type) :box)
             (setq frame-id "box1"))
            ((equal (send *obj* :get :type) :cylinder)
             (setq frame-id "cylinder1"))
            ((equal (send *obj* :get :type) :torus)
             (setq frame-id "torus1"))
            ((equal (send *obj* :get :type) :hand-mesh)
             (setq frame-id "hand1"))
            )
      (publish-object-handle :frame-id frame-id)
      (publish-object-approach :frame-id frame-id)
      ))
  (when *solve-ik-command-flag*
    (request-ik-from-marker-one))
  (when *save-obj-command-flag*
    (save-obj-model-one))
  (when *send-angle-command-flag*
    (send-angle-to-real-robot-one))
  (when *reach-until-touch-command-flag*
    (reach-until-touch-one))
  (when *object-menu-command-flag*
    (object-menu-one))
  (when *robot-menu-command-flag*
    (robot-menu-one))
  (when *assoc-obj-flag*
    (set-obj-pose-for-assoc))
  )
