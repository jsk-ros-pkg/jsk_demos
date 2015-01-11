#!/usr/bin/env roseus

(load "package://jsk_ik_server/euslisp/fullbody-ik-client.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/drc-task-common-util.l")

(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus-add-srvs "jsk_pcl_ros")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")
(ros::roseus-add-msgs "visualization_msgs")

(ros::roseus "request_ik_from_marker")

;; initialization ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-ik-from-marker-init
  ()
  ;; ros
  (setq *ns* (ros::get-param "~transformable_interactive_server_nodename" "/transformable_interactive_server"))
  (setq *frame-id* (ros::get-param "~default_frame_id" "/odom_on_ground"))
  (setq *assoc-frame-id* "jsk_model_marker_interface/hrp2/RARM_LINK6")
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::subscribe (format nil "~a~a" *ns* "/marker_dimensions") jsk_interactive_marker::MarkerDimensions #'generate-object-with-new-dimensions-from-topic)
  (ros::advertise (format nil "~a~a" *ns* "/set_color") std_msgs::ColorRGBA 1)
  (ros::subscribe "/urdf_control_marker/pose" geometry_msgs::PoseStamped #'robot-pose-cb)
  (ros::advertise "urdf_control_marker/set_pose" geometry_msgs::PoseStamped 1)
  (ros::advertise "/jsk_model_marker_interface/hrp2/reset_joint_states_and_root" sensor_msgs::JointState 1)
  (ros::advertise "/midi_config_player/set_feedback" sensor_msgs::JoyFeedbackArray 1)
  (ros::advertise "/inverse_reachablity_visible_map" visualization_msgs::Marker 1)
  (ros::subscribe "/solve_ik_command" std_msgs::empty #'solve-ik-command 1)
  (ros::subscribe "/save_obj_command" std_msgs::empty #'save-obj-command 1)
  (ros::subscribe "/send_angle_command" std_msgs::empty #'send-angle-command 1)
  (ros::subscribe "/object_menu_command" std_msgs::empty #'object-menu-command 1)
  (ros::subscribe "/robot_menu_command" std_msgs::empty #'robot-menu-command 1)
  (ros::subscribe "/grasp_pose" geometry_msgs::PoseStamped #'solve-ik-from-grasp-pose-command 1)
  (ros::subscribe "/grasp_pose_z_free" geometry_msgs::PoseStamped #'solve-ik-from-grasp-pose-command-z-free 1)
  (ros::subscribe "/push_pose" geometry_msgs::PoseStamped #'solve-ik-from-push-pose-command 1)
  (ros::subscribe "/move_pose" geometry_msgs::PoseStamped #'solve-ik-from-move-pose-command 1)
  (ros::subscribe "/move_by_axial_restraint_pose" geometry_msgs::PoseArray #'solve-ik-from-move-by-axial-pose-command 1)
  (ros::subscribe "/grasp_dual_pose" geometry_msgs::PoseArray #'solve-ik-from-grasp-dual-pose-command 1)
  (ros::subscribe "/grasp_dual_pose_z_free" geometry_msgs::PoseArray #'solve-ik-from-grasp-dual-pose-command-z-free 1)
  (ros::subscribe "/move_dual_pose" geometry_msgs::PoseArray #'solve-ik-from-move-dual-pose-command 1)
  (ros::advertise "/object_handle" geometry_msgs::PoseStamped 1)
  (ros::advertise "/object_approach" geometry_msgs::PoseStamped 1)
  (ros::advertise "/grasp_pose_feedback" geometry_msgs::PoseStamped 1)
  (ros::advertise "/grasp_pose_dual_feedback" geometry_msgs::PoseArray 1)
  (ros::subscribe "/object_handle_variable_pos" std_msgs::Float32 #'set-object-handle-pos-from-variable)
  (ros::subscribe "/object_handle_variable_rot" std_msgs::Float32 #'set-object-handle-rot-from-variable)
  (ros::subscribe "/object_approach_variable" std_msgs::Float32 #'set-object-approach-from-variable)
  (ros::advertise-service "/set_handle_reverse" std_srvs::Empty #'set-handle-reverse)
  (ros::advertise-service "/set_exec_mode" std_srvs::Empty #'set-exec-mode-cb)
  (ros::advertise-service "/assoc_object_to_robot" std_srvs::Empty #'assoc-object-to-robot)
  (ros::advertise-service "/dissoc_object_to_robot" std_srvs::Empty #'dissoc-object-to-robot)
  (ros::advertise-service "/set_object_mode_next" std_srvs::Empty #'set-obj-mode-next)
  (ros::advertise-service "/set_ik_mode_next" std_srvs::Empty #'set-ik-mode-next)
  (ros::advertise (format nil "~a~a" *ns* "/set_pose") geometry_msgs::PoseStamped 1)
  (ros::advertise "obj_state_text" jsk_rviz_plugins::OverlayText 1)
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
  (setq *draw-viewer-flag* (ros::get-param "~draw_viewer" t))
  (setq *robot-coords* (make-coords))
  (setq *obj-coords* (make-coords))
  (setq *solve-ik-command-flag* nil)
  (setq *solve-ik-from-pose-command-flag* nil)
  (setq *solve-ik-from-dual-pose-command-flag* nil)
  (setq *save-obj-command-flag* nil)
  (setq *send-angle-command-flag* nil)
  (setq *object-menu-command-flag* nil)
  (setq *robot-menu-command-flag* nil)
  (setq *handle-pos-variable* 0)
  (setq *handle-rot-variable* 0)
  (setq *approach-variable* 0)
  (setq *handle-reverse-flag* nil)
  (setq *assoc-obj-flag* nil)
  (setq *assoc-obj-coords* (make-coords))
  (setq *target-pose-coords* (make-coords))
  (setq *initial-pose-coords* (make-coords))
  (setq *move-target-from-hand* (make-coords))
  (setq *target-pose-coords-for-second-arm* (make-coords))
  (setq *initial-pose-coords-for-second-arm* (make-coords))
  (setq *move-target-from-second-hand* (make-coords))
 
  ;; ik
  (setq *ik-arm* :rarm)
  (setq *ik-result* nil)
  (setq *rotation-axis* (list t t t))
  (setq *rotation-axis-default* (list :z t t))
  (setq *pub-grasp-pose-feedback-flag* nil)
  (setq *pub-grasp-dual-pose-feedback-flag* nil)
  ;; todo
  (setq *ik-pose-list* nil)
  ;;executive map
  (setq *map-step-dist* (float-vector 200 200 100))
  (setq *map-max-point* (list 400 800 10))
  (setq *map-min-point* (list -300 -600 0))
  (setq *inverse-reachablity-map-mode* nil)
  (init-robot-model)
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_pose"))
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_type"))
  (ros::wait-for-service (format nil "~a~a" *ns* "/get_dimensions"))
  (generate-object-with-new-dimensions-from-service)
  (when *draw-viewer-flag* (objects (list *obj* *robot*)))
  (when (boundp '*irtviewer*) (send *irtviewer* :change-background #f(0.4 0.7 0.5)))
  (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

  (warning-message 2 "[request-ik] initialize request-ik-from-marker.~%")
  )

;; robot & obj model ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot model ;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot model initialization
(defun init-robot-model
  ()
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
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
  (send *obj* :newcoords
        (send (send (get-obj-coords :robot-coords robot-coords) :copy-worldcoords) :transform (send (send (find-if #'(lambda (x) (equal (send x :name) :origin)) (send *obj* :descendants)) :copy-worldcoords) :inverse-transformation) :local))
  )

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
  (when *draw-viewer-flag* (objects (list *obj* *robot*)))
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
(defun set-exec-mode-cb
  (req) 
  (let* ((res (send req :response)))
    (set-exec-mode)
    res)
  )

(defun set-exec-mode
  ()
  (if *inverse-reachablity-map-mode*
      (progn
	(setq *inverse-reachablity-coords-list* nil)
	(setq *inverse-reachablity-map-mode* nil)
	(ros::ros-info "exec-map disable"))
    (progn
      (setq *inverse-reachablity-map-mode* t)
      (inverse-reachablity-map-init)
      (ros::ros-info "exec-map driven")
      )
    )
    (show-inverse-reachablity-map)
  )
  
  
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

;; service server callback for assoc (not used now)
(defun assoc-object-to-robot
  (req )
  (let* ((res (send req :response)))
    (get-assoc-obj-coords)
    (setq *assoc-obj-flag* t)
    res))
(defun dissoc-object-to-robot
  (req)
  (let* ((res (send req :response)))
    (setq *assoc-obj-flag* nil)
    res))

;; request ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; callback ;;;;;;;;;;;;;;;;;;;;;;;;;
;; ik

(defun pub-grasp-pose-feedback
  ()
  (ros::ros-info "pub-grasp-pose-feedback")
  (if *ik-result*
      (progn
        (let 
            ((original-av (send *robot* :angle-vector)) hand-temp-coords)
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* (- (length *ik-result*) 1)))
          (setq hand-temp-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
          (if (equal *ik-arm* :rarm)
              (progn (send hand-temp-coords :rotate (* (/ 1.0 3) pi) :z :local) (send hand-temp-coords :translate (float-vector -30 -50 0) :local))
            (progn (send hand-temp-coords :rotate (* (/ -1.0 3) pi) :z :local)  (send hand-temp-coords :translate (float-vector -30 50 0) :local))
            )
          ;(send hand-temp-coords :transform *robot-coords*)
          (setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform (send hand-temp-coords :copy-worldcoords)))
          (ros::publish "/grasp_pose_feedback" (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose hand-temp-coords) :header (instance std_msgs::Header :init :frame_id *frame-id*
                                                                                                                                                          :stamp (ros::time-now))))
          (ros::ros-info "grasp feedback pose pubed")
          (send *robot* :angle-vector original-av)
          )
        )
    )
  ;(setq *pub-grasp-pose-feedback-flag* (not *pub-grasp-pose-feedback-flag*))
  )
(defun pub-grasp-dual-pose-feedback
  ()
  (ros::ros-info "pub-grasp-pose-feedback")
  (if *ik-result*
      (progn
        (let 
            ((original-av (send *robot* :angle-vector)) hand-temp-coords second-hand-temp-coords)
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* (- (length *ik-result*) 1)))
          (setq hand-temp-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
          (setq second-hand-temp-coords (send (send *robot* (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) :end-coords :worldcoords) :copy-worldcoords))
          (if (equal *ik-arm* :rarm)
              (progn (send hand-temp-coords :rotate (* (/ 1.0 3) pi) :z :local) (send hand-temp-coords :translate (float-vector -30 -50 0) :local)(send second-hand-temp-coords :rotate (* (/ -1.0 3) pi) :z :local)(send second-hand-temp-coords :translate (float-vector -30 50 0) :local))
            (progn (send hand-temp-coords :rotate (* (/ -1.0 3) pi) :z :local)  (send hand-temp-coords :translate (float-vector -30 50 0) :local)(send second-hand-temp-coords :rotate (* (/ 1.0 3) pi) :z :local) (send second-hand-temp-coords :translate (float-vector -30 -50 0) :local))
            )
          ;(send hand-temp-coords :transform *robot-coords*)
          (setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform hand-temp-coords))
          (setq second-hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform second-hand-temp-coords))
          (ros::publish "/grasp_pose_dual_feedback" (instance geometry_msgs::PoseArray :init :poses (list (ros::coords->tf-pose hand-temp-coords) (ros::coords->tf-pose second-hand-temp-coords)) :header (instance std_msgs::Header :init :frame_id *frame-id*
                                                                                                                                                                                                                   :stamp (ros::time-now))))
          (ros::ros-info "grasp dual_feedback pose pubed")
          (send *robot* :angle-vector original-av)
          )
        )
  ;(setq *pub-grasp-pose-feedback-flag* (not *pub-grasp-pose-feedback-flag*))
    )
  )
(defun solve-ik-command
  (msg)
  (setq *solve-ik-command-flag* t))

(defun solve-ik-from-grasp-pose-command 
  (msg)
  (solve-ik-from-pose-common msg)
  (if (equal *ik-arm* :rarm)
      (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -140 -80 0) :local))
    (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -140 80 0) :local))
    )
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list t t t))
  (setq *pub-grasp-pose-feedback-flag* nil)
  (setq *solve-ik-command-flag* t)
  )
(defun solve-ik-from-grasp-pose-command-z-free
  (msg)
  (solve-ik-from-grasp-pose-command msg)
  (setq *pub-grasp-pose-feedback-flag* t)
  (setq *rotation-axis* (list :z t t))
  )
(defun solve-ik-from-move-pose-command 
  (msg)
  (get-current-pose-common)
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list t t t))
  )

(defun solve-ik-from-push-pose-command 
  (msg)
  (solve-ik-from-pose-common msg)
  (send *target-pose-coords* :translate (float-vector -10 0 0) :local) ;; length of finger
  (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -60 0 0) :local))
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list t t t));;
  )
(defun solve-ik-from-move-by-axial-pose-command
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-move-by-axial-pose-command nil))
    (let* ((transform (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts)) hand-temp-coords)
      (setq *target-pose-coords*
            (send (send transform :copy-worldcoords) :transform (ros::tf-pose->coords (elt (send msg :poses) 1))))
      (setq hand-temp-coords
            (send (send transform :copy-worldcoords) :transform (ros::tf-pose->coords (elt (send msg :poses) 0))))
	
      (if (equal *ik-arm* :rarm)
          (progn (send hand-temp-coords :translate (float-vector 30 50 0)
                       :local) (send hand-temp-coords :rotate (* (/ -1.0 3) pi) :z :local))
	(progn (send hand-temp-coords :translate (float-vector 30 -50 0) :local) (send hand-temp-coords :rotate (* (/ 1.0 3) pi) :z :local))
	)
      (setq *move-target-from-hand* (send (send (send *target-pose-coords* :copy-worldcoords) :transformation hand-temp-coords :local) :transformation (make-coords) :local)) 
      (setq *solve-ik-from-pose-command-flag* t)
      (get-current-pose-common)
      (send *initial-pose-coords* :transform *move-target-from-hand*)
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *rotation-axis* (list :x t t))
      ))
  )
(defun get-current-pose-common
  ()
  (let ((original-av (send *robot* :angle-vector)))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    ;(setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform (send hand-temp-coords :copy-worldcoords)))
    ;(setq *initial-pose-coords* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords)))
    (setq *initial-pose-coords* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords)))
    (setq *initial-pose-coords-for-second-arm* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) :end-coords :worldcoords) :copy-worldcoords)))
    (send *robot* :angle-vector original-av)
    )
  )
(defun solve-ik-from-pose-common
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-pose-common nil))
    (setq *target-pose-coords*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords(send msg :pose)
                                                                                                                          )
                )
          )
    (if (equal *ik-arm* :rarm)
        (setq *move-target-from-hand* (send (send (make-coords) :rotate (* (/ 1.0 3) pi) :z :local) :translate (float-vector -30 -50 0))) ;;for grasp
      (setq *move-target-from-hand* (send (send (make-coords) :rotate (* (/ -1.0 3) pi) :z :local) :translate (float-vector -30 50 0))) ;;for grasp     
      ;;(progn (send *target-pose-coords* :translate (float-vector 30 60 0) :local) (send *target-pose-coords* :rotate (* (/ -1.0 4) pi) :z :local))
      ;;(progn (send *target-pose-coords* :translate (float-vector 30 -60 0) :local) (send *target-pose-coords* :rotate (* (/ 1.0 4) pi) :z :local))
      )
    )
  )
(defun solve-ik-from-grasp-dual-pose-command
  (msg)
  (solve-ik-from-dual-pose-common msg)
  (if (equal *ik-arm* :rarm)
      (progn 
        (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -140 -80 0) :local))
        (setq *initial-pose-coords-for-second-arm* (send (send *target-pose-coords-for-second-arm* :copy-worldcoords) :translate (float-vector -140 80 0) :local))
        )
    (progn
      (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -140 80 0) :local))
      (setq *initial-pose-coords-for-second-arm* (send (send *target-pose-coords-for-second-arm* :copy-worldcoords) :translate (float-vector -140 -80 0) :local))
      )
    )
  (setq *rotation-axis* (list t t t t))
  (setq *solve-ik-from-dual-pose-command-flag* t)
  )
(defun solve-ik-from-grasp-dual-pose-command-z-free
  (msg)
  (ros::ros-info "dual z:free driven")
  (solve-ik-from-grasp-dual-pose-command msg)
  (setq *rotation-axis* (list :z :z t t))
  (setq *pub-grasp-dual-pose-feedback-flag* t)
  )
(defun solve-ik-from-move-dual-pose-command
  (msg)
  (solve-ik-from-dual-pose-common msg)
  (get-current-pose-common)
  (setq *rotation-axis* (list t t t t))
  (setq *solve-ik-from-dual-pose-command-flag* t)
  )
(defun solve-ik-from-dual-pose-common
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-dual-pose-common nil))
    (setq *target-pose-coords*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords (elt (send msg :poses) 0)))
          )
    (setq *target-pose-coords-for-second-arm*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords (elt (send msg :poses) 1)))
          )
    (if (equal *ik-arm* :rarm)
        (progn (setq *move-target-from-hand* (send (send (make-coords) :rotate (* (/ 1.0 3) pi) :z :local) :translate (float-vector -30 -50 0))) (setq *move-target-from-second-hand* (send (send (make-coords) :rotate (* (/ -1.0 3) pi) :z :local) :translate (float-vector -30 50 0)))) ;;for grasp
      (progn (setq *move-target-from-hand* (send (send (make-coords) :rotate (* (/ -1.0 3) pi) :z :local) :translate (float-vector -30 50 0))) (setq *move-target-from-second-hand* (send (send (make-coords) :rotate (* (/ 1.0 3) pi) :z :local) :translate (float-vector -30 -50 0)))) ;;for grasp
      )
    )
  )

(defun save-obj-command
  (msg)
  (setq *save-obj-command-flag* t))
;; angle-vector
(defun send-angle-command
  (msg)
  (setq *send-angle-command-flag* t))
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
  (&key (initial-pose :reset-manip-pose) (robot-coords *robot-coords*))
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
    (when *draw-viewer-flag* (objects (list *obj* *robot*)))
    ;; check mode 
    (let* ((get-marker-existence-req (instance jsk_interactive_marker::GetTransformableMarkerExistenceRequest :init)))
      (send get-marker-existence-req :target_name *saved-obj-name*)
      (when (send (ros::service-call (format nil "~a~a" *ns* "/get_existence") get-marker-existence-req) :existence)
        (setq interpolate-mode :save)))
    (unless (equal (elt *ik-mode-list* *ik-mode-index*) :none)
      (setq interpolate-mode :approach))
    ;; solve ik
    (setq *ik-result* (send-ik-request :interpolate-mode interpolate-mode))
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
(defun request-ik-from-pose-one
  (&key (initial-pose :reset-manip-pose) (robot-coords (send *robot-coords* :copy-worldcoords)) (target-pose-coords (send *target-pose-coords* :copy-worldcoords)) (initial-pose-coords (send *initial-pose-coords* :copy-worldcoords)) target-pose-coords-for-second-arm initial-pose-coords-for-second-arm move-target-from-second-hand)
  (let* (original-av original-coords)
    (warning-message 2 "[request-ik] send solve-ik service.~%")
    (setq original-av (send *robot* :angle-vector))
    (setq original-coords (send (send *robot* :worldcoords) :copy-worldcoords))
    ;; regenerate move coords
    (when initial-pose (send *robot* initial-pose))
    (send *robot* :fix-leg-to-coords (make-coords))
    ;; check mode 
    ;; solve ik
    (setq target-pose-coords (send (send target-pose-coords :transformation robot-coords :local) :transformation (make-coords) :local))
    (setq initial-pose-coords (send (send initial-pose-coords :transformation robot-coords :local) :transformation (make-coords) :local))
    (if target-pose-coords-for-second-arm (setq target-pose-coords-for-second-arm (send (send target-pose-coords-for-second-arm :transformation robot-coords :local) :transformation (make-coords) :local)))
    (if initial-pose-coords-for-second-arm (setq initial-pose-coords-for-second-arm (send (send initial-pose-coords-for-second-arm :transformation robot-coords :local) :transformation (make-coords) :local)))


    (when *draw-viewer-flag* (objects (list *robot* target-pose-coords initial-pose-coords)))
    (setq *ik-result* (send-ik-request :target-coords target-pose-coords :initial-coords initial-pose-coords :rotation-axis *rotation-axis* :interpolate-mode :from-pose-one :move-target-from-hand *move-target-from-hand*
                                       :target-coords-for-second-arm target-pose-coords-for-second-arm :initial-coords-for-second-arm initial-pose-coords-for-second-arm :move-target-from-second-hand move-target-from-second-hand))
    (if *pub-grasp-pose-feedback-flag* (pub-grasp-pose-feedback))
    (if *pub-grasp-dual-pose-feedback-flag* (pub-grasp-dual-pose-feedback))
    (cond (*ik-result*
           (send-joint-states-to-marker))
          (t
           (send *robot* :angle-vector original-av)
           (send *robot* :newcoords original-coords)
           )
          )
    (setq *solve-ik-from-pose-command-flag* nil)
    (setq *solve-ik-from-dual-pose-command-flag* nil)
    *ik-result*
    ))

;; set start pose
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

;; send ik service request
(defun send-ik-request
  (&key 
   (interpolate-mode)
   (target-coords (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
   (initial-coords target-coords)
   (arm *ik-arm*)
   (move-target-from-hand (make-coords))
   (rotation-axis (list :z t t))
   (target-coords-for-second-arm nil)
   (initial-coords-for-second-arm target-coords-for-second-arm)
   (move-target-from-second-hand (make-coords))
   )
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
  (let* (ik-res ik-angle-list (divide-num (if interpolate-mode 5 1)))
    ;; service call for task state manager
    (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "SolveIK"))
    ;; loop for interpolation
    (dotimes (i divide-num)
      (if (not target-coords-for-second-arm)
          (let* ((arm-index (if (equal arm :rarm) 0 1))
                 (mt (append (list (elt *end-coords-from-hand* arm-index)) *end-coords-without-toe*))
                 (tc (send-all mt :copy-worldcoords))
                 (ll (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) mt))
                 (translation-axis (list t t t))
                 (end-coords-2 (make-cascoords :name :end-coords-2 :coords (send (elt *end-coords-from-hand* arm-index) :copy-worldcoords)))
                 res)

            (setf (elt tc 0) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords target-coords) :copy-worldcoords))
            (send end-coords-2 :transform move-target-from-hand :local)
            (send (send (elt *end-coords-from-hand* arm-index) :parent) :assoc end-coords-2)
            (setf (elt mt 0) end-coords-2)	      
            (cond ((equal arm :rarm)
                   (setq *assoc-frame-id* "jsk_model_marker_interface/hrp2/RARM_LINK6"))
              ((equal arm :larm)
               (setq *assoc-frame-id* "jsk_model_marker_interface/hrp2/LARM_LINK6")))
            ;; service call for ik server
            (setq res (ik-server-call
                       :target-coords tc
                       :move-target mt
                       :link-list ll
                       :debug-view :no-message
                       :translation-axis translation-axis
                       :rotation-axis rotation-axis
                       :max (float-vector  500  500  0  20  20  10)
                       :warnp t
                       :stop 60
                       ))
            
            (setq ik-res (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*))
            (send (send (elt *end-coords-from-hand* arm-index) :parent) :dissoc end-coords-2)
            (unless ik-res (return))
            (push (send-all (send *robot* :joint-list) :joint-angle) ik-angle-list)
            )
        (let* ((arm-index (if (equal arm :rarm) 0 1))
               (second-arm-index (if (equal arm :rarm) 1 0))
               (mt (append (list (elt *end-coords-from-hand* arm-index) (elt *end-coords-from-hand* second-arm-index)) *end-coords-without-toe*))
               (tc (send-all mt :copy-worldcoords))
               (ll (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) mt))
               (translation-axis (list t t t t))
               (end-coords-2 (make-cascoords :name :end-coords-2 :coords (send (elt *end-coords-from-hand* arm-index) :copy-worldcoords)))
               (end-coords-3 (make-cascoords :name :end-coords-3 :coords (send (elt *end-coords-from-hand* second-arm-index) :copy-worldcoords)))
               res)
          
          (setf (elt tc 0) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords target-coords) :copy-worldcoords))
          (setf (elt tc 1) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords-for-second-arm target-coords-for-second-arm) :copy-worldcoords))
          (send end-coords-2 :transform move-target-from-hand :local)
          (send (send (elt *end-coords-from-hand* arm-index) :parent) :assoc end-coords-2)
          (setf (elt mt 0) end-coords-2)
          (send (send (elt *end-coords-from-hand* second-arm-index) :parent) :assoc end-coords-3)
          (setf (elt mt 1) end-coords-3)
          (send end-coords-3 :transform move-target-from-second-hand :local)
          (cond ((equal arm :rarm)
                 (setq *assoc-frame-id* "jsk_model_marker_interface/hrp2/RARM_LINK6"))
                ((equal arm :larm)
                 (setq *assoc-frame-id* "jsk_model_marker_interface/hrp2/LARM_LINK6")))
          ;; service call for ik server
          (setq res (ik-server-call
                     :target-coords tc
                     :move-target mt
                     :link-list ll
                     :debug-view :no-message
                     :translation-axis translation-axis
                     :rotation-axis rotation-axis
                     :max (float-vector  500  500  0  20  20  10)
                     :warnp t
                     :stop 45
                     ))
          
          (setq ik-res (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*))
          (send (send (elt *end-coords-from-hand* arm-index) :parent) :dissoc end-coords-2)
          (send (send (elt *end-coords-from-hand* second-arm-index) :parent) :dissoc end-coords-3)
          (unless ik-res (return))
          (push (send-all (send *robot* :joint-list) :joint-angle) ik-angle-list)
          )
        
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
      (ros::publish "/jsk_model_marker_interface/hrp2/reset_joint_states_and_root" msg)
      (ros::spin-once))
    (when (= i (- (length *ik-result*) 1))
      (return))
    (unix::usleep (* 200 1000)))
  )

;; pose, angle-vector ;;;;;;;;;;;;;;;;;;;;;;;;;
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
            (send *ri* :angle-vector (send *robot* :angle-vector) (float int-time))
            (unix::usleep (round (* (* int-time 1000) 0.8)))
            )
           (t
            (send *ri* :angle-vector (send *robot* :angle-vector) (/ (float int-time) (length *ik-result*)))
            (cond ((= i (- (length *ik-result*) 1))
                   (send *ri* :wait-interpolation))
                  (t
                   (unix::usleep (round (* (* (/ (float int-time) (length *ik-result*)) 1000) 0.8)))))
            )
           )
          ))
       (t
        (send *ri* :angle-vector (send *robot* :angle-vector) int-time)
        (send *ri* :wait-interpolation)))
      ;; service call for task state manager
      (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FinishAngleVec"))
      (ros::ros-info (format nil "request angle: ~A" (send *robot* :angle-vector)))
      (ros::ros-info (format nil "true angle: ~A" (send *ri* :state :potentio-vector)))
      ))
  (setq *send-angle-command-flag* nil)
  (warning-message 2 "[request-ik] finished sending angle-vector.~%")
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
    (setq *object-menu-command-flag* nil)
    ))

;; robot menu ;;;;;;;;;;;;;;;;;;;;;;;;;
(defun robot-menu-one
  ()
  (let* ((title
          (format nil "Robot Menu~%"))
         (menu-list (list "cancel" "switch arm" "current pos" "inverse-reachablity-map-mode" ":reset-pose" ":reset-manip-pose" ":init-pose" ":reach-drill-walk-pose" ":reach-drill-pre-pose" ":pierce-drill-pre-pose"))
         (switch-arm-index 1) 
         (current-pos-index 2)
	 (inverse-reachablity-map-mode-index 3)
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
	  ((equal res-index inverse-reachablity-map-mode-index)
	   (set-exec-mode))
          ((find-method *robot* (read-from-string res-value))
           (eval (list 'send '*robot* '(read-from-string res-value)))
           (setq *ik-result* (list (send-all (send *robot* :joint-list) :joint-angle)))
           (send-joint-states-to-marker)))
    (setq *robot-menu-command-flag* nil)
    ))
(defun make-xy-coords (x y)
  (send (send *robot-coords* :copy-worldcoords) :translate (float-vector x y 0) :local)
  ;(make-coords :pos (float-vector x y 0))
  )
(defun inverse-reachablity-map-init ()
  (let* (map-list-x map-list-y)
      (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) (elt *map-step-dist* 0))))
	(dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) (elt *map-step-dist* 1))))
	  (push (+ (* (* 1.0 ix) (elt *map-step-dist* 0)) (elt *map-min-point* 0))  map-list-x)
	  (push (+ (* (* 1.0 iy) (elt *map-step-dist* 1)) (elt *map-min-point* 1))  map-list-y)
	  ))
      (setq *inverse-reachablity-coords-list* (mapcar #'make-xy-coords map-list-x map-list-y))
      )
  )
(defun show-inverse-reachablity-map ()
  (let* ((msg (instance visualization_msgs::marker :init))
         (pose-msg (instance geometry_msgs::pose :init))
         points-list-msg colors-list-msg
         )
    (send msg :header (instance std_msgs::header :init
                                :stamp (ros::time-now) :frame_id *frame-id*))
    (send msg :lifetime (ros::time 0.0))
    (send msg :frame_locked t)
    (send msg :type 6) ;; cube_list
    (send msg :action 0)
    (send pose-msg :orientation :w 1.0)
    (send msg :pose pose-msg)
    (send msg :scale (instance geometry_msgs::vector3 :init
                               :x (* 0.5 (* 0.001 (elt *map-step-dist* 0)))
                               :y (* 0.5 (* 0.001 (elt *map-step-dist* 1)))
                               :z (* 0.5 (* 0.001 (elt *map-step-dist* 2)))
))
    (dotimes (i (length *inverse-reachablity-coords-list*))
      (push (instance geometry_msgs::point :init
                      :x (* 0.001 (elt (send (elt *inverse-reachablity-coords-list* i) :pos) 0))
                      :y (* 0.001 (elt (send (elt *inverse-reachablity-coords-list* i) :pos) 1))
                      :z (* 0.001 (elt (send (elt *inverse-reachablity-coords-list* i) :pos) 2)))
            points-list-msg)
      (push (vector->rgba (float-vector 1 0 1)) colors-list-msg)
      )
    (send msg :points points-list-msg)
    (send msg :colors colors-list-msg)
    (ros::publish "/inverse_reachablity_visible_map" msg)
   )
  )
   
   
   

  

;; main ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(request-ik-from-marker-init)

(while t
  (ros::spin-once)
  (x::window-main-one)
  (when (and *obj* (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
    (let* (frame-id)
      (cond ((equal (send *obj* :get :type) :box)
             (setq frame-id "box1"))
            ((equal (send *obj* :get :type) :cylinder)
             (setq frame-id "cylinder1"))
            ((equal (send *obj* :get :type) :torus)
             (setq frame-id "torus1")))
      (publish-object-handle :frame-id frame-id)
      (publish-object-approach :frame-id frame-id)
      ))
  (when *solve-ik-command-flag*
    (cond
     (*inverse-reachablity-map-mode* 
      (let* ((inverse-reachablity-coords-list-temp nil))
	(dotimes (i (length *inverse-reachablity-coords-list*))
	  (if (request-ik-from-marker-one :robot-coords (elt *inverse-reachablity-coords-list* i))
	      (push (elt *inverse-reachablity-coords-list* i) inverse-reachablity-coords-list-temp)
	    )
	  )
	(setq *inverse-reachablity-coords-list* inverse-reachablity-coords-list-temp)
	(show-inverse-reachablity-map)
	))
     (t
      (request-ik-from-marker-one))       
     )
    )
  (when *solve-ik-from-pose-command-flag*
    (cond
     (*inverse-reachablity-map-mode* 
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *pub-grasp-dual-pose-feedback-flag* nil)
      (let* ((inverse-reachablity-coords-list-temp nil))
	(dotimes (i (length *inverse-reachablity-coords-list*))
	  (if (request-ik-from-pose-one :robot-coords (elt *inverse-reachablity-coords-list* i))
	      (push (elt *inverse-reachablity-coords-list* i) inverse-reachablity-coords-list-temp)
	    )
	  )
	(setq *inverse-reachablity-coords-list* inverse-reachablity-coords-list-temp)
	(show-inverse-reachablity-map)
	))
     (t
      (request-ik-from-pose-one)) 
     )
    (setq *pub-grasp-pose-feedback-flag* nil)
    (setq *pub-grasp-dual-pose-feedback-flag* nil)
    )
  (when *solve-ik-from-dual-pose-command-flag*
    (cond
     (*inverse-reachablity-map-mode*
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *pub-grasp-dual-pose-feedback-flag* nil)
      (let* ((inverse-reachablity-coords-list-temp nil))
	(dotimes (i (length *inverse-reachablity-coords-list*))
	  (if (request-ik-from-pose-one :target-pose-coords-for-second-arm *target-pose-coords-for-second-arm* :initial-pose-coords-for-second-arm *initial-pose-coords-for-second-arm* :move-target-from-second-hand *move-target-from-second-hand* :robot-coords (elt *inverse-reachablity-coords-list* i))
	      (push (elt *inverse-reachablity-coords-list* i) inverse-reachablity-coords-list-temp)
	    )
	  )
	(setq *inverse-reachablity-coords-list* inverse-reachablity-coords-list-temp)
	(show-inverse-reachablity-map)
	))
     (t
        (request-ik-from-pose-one :target-pose-coords-for-second-arm *target-pose-coords-for-second-arm* :initial-pose-coords-for-second-arm *initial-pose-coords-for-second-arm* :move-target-from-second-hand *move-target-from-second-hand*))
     )
    (setq *pub-grasp-pose-feedback-flag* nil)
    (setq *pub-grasp-dual-pose-feedback-flag* nil)
    )
  (when *save-obj-command-flag*
    (save-obj-model-one))
  (when *send-angle-command-flag*
    (send-angle-to-real-robot-one))
  (when *object-menu-command-flag*
    (object-menu-one))
  (when *robot-menu-command-flag*
    (robot-menu-one))
  (when *assoc-obj-flag*
    (set-obj-pose-for-assoc))
  )
