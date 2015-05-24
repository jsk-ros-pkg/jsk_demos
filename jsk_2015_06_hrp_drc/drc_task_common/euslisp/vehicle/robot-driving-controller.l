#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(ros::roseus-add-srvs "std_srvs")

;; for drc_task_common
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "drc_task_common")

(defclass robot-driving-controller
  :super propertied-object
  :slots (motion
          ros-rate
          accel-origin brake-origin
          step-accel-command step-brake-command
          release-accel-command release-brake-command
          state-list mode-list
          polygon-stamped-msg
          torus-finder-estimated-robot-coords
          handle-pose-estimated-robot-coords
          handle-pose-estimate-flag
          handle-pose-old-robot-coords
          torus-finder-estimation-flag
          old-estimated-handle-angle
          old-accel-command
          old-brake-command
          tf-listener
          handling-end-coords-list
          record-handling-end-coords-flag
          max-turn-handle-omega
          ) 
  )

(defmethod robot-driving-controller
  (:init
   (&key ((:real tmp-real) nil)
         ((:motion tmp-motion) nil)
         ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "robot_driving_controller")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance robot-driving-motion :init :real nil)) ;; real is set as tmp-real after initialization (motion is assumed to be pass with :real nil. if not, initialization process would be executed twice
     )
   (setq *motion* tmp-motion)
   (setq motion *motion*)
   
   ;; initialize instance valiables
   (setq ros-rate tmp-ros-rate
         state-list (list :initialize :release :approach :running)
         mode-list (list :stop :operation :recognition)
         torus-finder-estimated-robot-coords nil
         handle-pose-estimated-robot-coords nil
         handle-pose-old-robot-coords nil
         old-estimated-handle-angle 0.0
         old-accel-command nil
         old-brake-command nil
         handle-pose-estimate-flag nil
         torus-finder-estimation-flag nil
         handling-end-coords-list nil
         record-handling-end-coords-flag nil
         )
   ;; set defalut value for pedaling (for staro)
   (setq accel-origin 14
         step-accel-command 14 ;; 28
         release-accel-command 0 ;; 14
         brake-origin 5
         step-brake-command 15 ;; 20
         release-brake-command 0 ;; 5
         max-turn-handle-omega 90
         )

   ;; initialize joy controller values if exists
   (when (ros::wait-for-service "/drive/operation/initialize" 1)
     (let ((req (instance std_srvs::EmptyRequest :init)))
       (ros::service-call "/drive/operation/initialize" req t)
       )
     )

   ;; handle-angle topics
   (ros::advertise "/drive/controller/goal_handle_angle" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/estimated_handle_angle" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/steering_diff_angle_vector" std_msgs::Float32 1 t)

   ;; pedal angle topics
   (ros::advertise "/drive/controller/step" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/min_step" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/max_step" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/step_on_flag" std_msgs::Bool 1 t)

   ;; neck command topics
   (ros::advertise "/drive/controller/neck_y_angle" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/neck_p_angle" std_msgs::Float32 1 t)

   ;; for steering visualization
   (ros::advertise "/drive/controller/currest_steering" jsk_recognition_msgs::TorusArray 1)
   (ros::advertise "/drive/controller/handling_trajectory" geometry_msgs::PolygonStamped 1)

   ;; for steering estimation
   (ros::advertise "/drive/controller/torus_finder_input" geometry_msgs::PolygonStamped 1)
   (ros::subscribe "/drive/torus_finder/output/pose"
                   geometry_msgs::PoseStamped
                   #'send self :torus-finder-callback)

   ;; for crank recognition
   (setq tf-listener (instance ros::transform-listener :init))
   (ros::subscribe "/handle_pose"
                   geometry_msgs::PoseStamped
                   #'send self :handle-pose-callback)

   ;; for controller mode
   (ros::advertise "/drive/controller/handle_mode" std_msgs::String 1)
   (ros::advertise "/drive/controller/accel_mode" std_msgs::String 1)
   (ros::advertise "/drive/controller/brake_mode" std_msgs::String 1)
   (ros::advertise "/drive/controller/neck_mode" std_msgs::String 1)

   ;; for vehicle ui
   (ros::advertise-service "/drive/controller/initialize" std_srvs::Empty #'send self :initialize-callback)
   (ros::advertise-service "/drive/controller/approach_handle" std_srvs::Empty #'send self :approach-handle-callback)
   (ros::advertise-service "/drive/controller/grasp" std_srvs::Empty #'send self :grasp-callback)
   (ros::advertise-service "/drive/controller/release" std_srvs::Empty #'send self :release-callback)
   (ros::advertise-service "/drive/controller/correct" std_srvs::Empty #'send self :correct-callback)
   (ros::advertise-service "/drive/controller/resume" std_srvs::Empty #'send self :resume-callback)
   (ros::advertise-service "/drive/controller/approach_accel" std_srvs::Empty #'send self :approach-accel-callback)
   (ros::advertise-service "/drive/controller/reach_leg" std_srvs::Empty #'send self :reach-leg-callback)
   (ros::advertise-service "/drive/controller/reach_arm" std_srvs::Empty #'send self :reach-arm-callback)
   (ros::advertise-service "/drive/controller/egress" std_srvs::Empty #'send self :egress-callback)
   (ros::advertise-service "/drive/controller/set_max_step" drc_task_common::SetValue #'send self :set-max-step-callback)
   (ros::advertise-service "/drive/controller/set_min_step" drc_task_common::SetValue #'send self :set-min-step-callback)
   (ros::advertise-service "/drive/controller/set_handle_mode" drc_task_common::StringRequest #'send self :set-handle-mode-callback)
   (ros::advertise-service "/drive/controller/set_accel_mode" drc_task_common::StringRequest #'send self :set-accel-mode-callback)
   (ros::advertise-service "/drive/controller/set_neck_mode" drc_task_common::StringRequest #'send self :set-neck-mode-callback)
   (ros::advertise-service "/drive/controller/overwrite_handle_angle" drc_task_common::SetValue #'send self :overwrite-handle-angle-callback)

   ;; initialize plist
   ;; after advertise publisher and before subscribe controller input
   (send self :controller-state :handle :initialize)
   (send self :controller-state :accel :initialize)
   (send self :controller-state :brake :initialize)
   (send self :controller-state :neck :initialize)
   (send self :controller-mode :handle :stop)
   (send self :controller-mode :accel :stop)
   (send self :controller-mode :brake :stop)
   (send self :controller-mode :neck :stop)
   
   ;; subscriber for operator command
   (ros::subscribe "/drive/operation/accel_cmd"
                   std_msgs::Float64
                   #'send self :accel-callback)
   (ros::subscribe "/drive/operation/brake_cmd"
                   std_msgs::Float64
                   #'send self :brake-callback)
   (ros::subscribe "/drive/operation/handle_cmd"
                   std_msgs::Float64
                   #'send self :handle-callback)
   (ros::subscribe "/drive/operation/neck_y_cmd"
                   std_msgs::Float64
                   #'send self :neck-y-callback)
   (ros::subscribe "/drive/operation/neck_p_cmd"
                   std_msgs::Float64
                   #'send self :neck-p-callback)
   
   ;; set real flag without moving model
   (send motion :real tmp-real :use-real-robot-angle nil)
   )

  ;; vehicle ui callback
  (:initialize-callback
   (req)
   (let ((res (send req :response)))
     (send self :initialize :sync nil)
     res
     )
   )
  (:approach-handle-callback
   (req)
   (let ((res (send req :response)) use-controller)
     (send self :approach-handle) ;; sync-controller is checked in default parameter
     res
     )
   )
  (:grasp-callback
   (req)
   (let ((res (send req :response)))
     (send self :grasp)
     res
     )
   )
  (:release-callback
   (req)
   (let ((res (send req :response)))
     (send self :release)
     res
     )
   )
  (:correct-callback
   (req)
   (let ((res (send req :response)))
     (send self :correct-handle-pose :yes t :look-at-handle nil)
     res
     )
   )
  (:resume-callback
   (req)
   (let ((res (send req :response)))
     (send self :resume-handle-pose)
     res
     )
   )
  (:approach-accel-callback
   (req)
   (let ((res (send req :response)))
     (send self :approach-accel) ;; sync-controller is checked in default parameter
     res
     )
   )
  (:reach-leg-callback
   (req)
   (let ((res (send req :response)))
     (send self :approach-floor :reach-until-touch t)
     res
     )
   )
  (:reach-arm-callback
   (req)
   (let ((res (send req :response)))
     (send self :approach-fist :reach-until-touch t)
     res
     )
   )
  (:egress-callback
   (req)
   (let ((res (send req :response)))
     (send self :go-to-egress)
     res
     )
   ) 
    
  (:set-max-step-callback
   (req)
   (let ((res (send req :response)))
     (send res :set_value (send self :step-accel-command (send req :value)))
     (warn "set step-accel-command to ~A~%" (send self :step-accel-command))
     res
     )
   )
  (:set-min-step-callback
   (req)
   (let ((res (send req :response)))
     (send res :set_value (send self :release-accel-command (send req :value)))
     (warn "set release-accel-command to ~A~%" (send self :release-accel-command))
     res
     )
   )

  (:set-handle-mode-callback
   (req)
   (let ((res (send req :response)))
     (send self :controller-mode :handle (read-from-string (format nil ":~A" (string-downcase (send req :data)))))
     (warn "set handle-mode to ~A~%" (send self :controller-mode :handle))
     res
     )
   )
  
  (:set-accel-mode-callback
   (req)
   (let ((res (send req :response)))
     (send self :controller-mode :accel (read-from-string (format nil ":~A" (string-downcase (send req :data)))))
     (warn "set accel-mode to ~A~%" (send self :controller-mode :accel))
     res
     )
   )

  (:set-neck-mode-callback
   (req)
   (let ((res (send req :response)))
     (send self :controller-mode :neck (read-from-string (format nil ":~A" (string-downcase (send req :data)))))
     (warn "set neck-mode to ~A~%" (send self :controller-mode :neck))
     res
     )
   )

  (:overwrite-handle-angle-callback
   (req)
   (let ((res (send req :response)) (pub-msg (instance std_msgs::Float32 :init)))
     (send pub-msg :data (send res :set_value (send motion :overwrite-handle-angle (send req :value))))
     (setq old-estimated-handle-angle (send req :value))
     (send self :estimate-current-handle-angle) ;; update estimated handle angle
     (warn "overwrite handle angle to ~A~%" (send motion :car :handle-joint :joint-angle))
     (ros::publish "/drive/controller/goal_handle_angle" pub-msg)
     (send self :set-synchronize-flag :handle)
     res
     )
   )
  
  ;; interface command
  (:initialize
   (&rest args &key (sync nil) (time 10000) (real (send motion :real)) (fist-offset #f(0 0 25)) (floor-offset #f(0 0 0)))
   (let ((pub-handle-goal (instance std_msgs::Float32 :init))) ;; sync is executed according to controller-mode, not real flag
     (when sync
       (send motion :sync-all-controller) ;; execute sync-controller forcely in all limbs
       )
     (send motion :real nil :use-real-robot-angle nil) ;; temporary disable (send *ri* :angle-vector)
     (warn "init-pose~%")
     (send motion :ride :init-pose t)
     (send motion :drive-init-pose-crank :limb :larm :move-arm nil) ;; todo: fix and remove
     (warn "approach-fist~%")
     (send self :approach-fist :offset fist-offset :reach-until-touch nil)
     (warn "approach-accel~%")
     (send self :approach-accel) ;; sync is executed when (equal (send self :controller-state :accel) :running) automatically
     (warn "approach-floor~%")
     (send self :approach-floor :offset floor-offset :reach-until-touch nil)
     (warn "approach-handle~%")
     (send self :approach-handle) ;; sync is executed when (equal (send self :controller-state :handle) :running) automatically
     (send self :publish-body-relative-steering-coords)
     ;; move head front
     (warn "init-head~%")
     (send self :controller-state :neck :running) ;; activate neck
     (send self :neck-cmd 0.0 :neck-p :time 2000)
     (send self :neck-cmd 0.0 :neck-y :time 2000)
     (warn "send-initial-pose~%")
     (send motion :real real :use-real-robot-angle nil) ;; set real as argument
     (send motion :model2real :time time :wait-interpolation t) ;; send *ri* :angle-vector if real is t (default :real is set above)
     (warn "init-hand~%")
     (send motion :open-hand (send motion :grasp-arm))
     (send motion :close-hand (get-opposite-arm (send motion :grasp-arm)))
     ;; reset goal handle angle
     (send pub-handle-goal :data (send motion :car :handle-joint :joint-angle))
     (ros::publish "/drive/controller/goal_handle_angle" pub-handle-goal)
     )
   ;; initialize joy_vehicle if exists
   (send self :set-synchronize-flag :all)
   ;; init operation mode
   (send self :controller-mode :handle :stop)
   (send self :controller-mode :accel :stop)
   (send self :controller-mode :neck :stop)
   nil
   )
  (:grasp
   (&rest args)
   (let ((grasp-result nil)
         (pub-handle-goal (instance std_msgs::Float32 :init))
         )
     (unless (or (equal (send self :controller-state :handle) :approach) (equal (send self :controller-state :handle) :release))
       (warn "handle is not approached~%")
       (return-from :grasp nil) ;; prevent to crash
       )
     (setq grasp-result (send* motion :grasp-handle args))
     (when grasp-result
       ;; update goal_handle_angle to current handle
       (send pub-handle-goal :data (send motion :car :handle-joint :joint-angle))
       (ros::publish "/drive/controller/goal_handle_angle" pub-handle-goal)
       ;; handle-angle is estimated to be right when grasped
       (setq old-estimated-handle-angle (send motion :car :handle-joint :joint-angle))
       ;; set handle status
       (send self :controller-state :handle :running)
       )
     grasp-result
     )
   )
  (:release
   (&rest args)
   (let (release-result)
     (setq release-result (send* motion :release-handle args))
     (when release-result
       (send self :controller-state :handle :release)
       )
     release-result
     )
   )
  (:correct-handle-pose
   (&key (yes nil) (grasp t) (time 2500) (look-at-handle t))
   (let (ret-result approach-result grasp-result)
     (setq handle-pose-estimate-flag t)
     (setq ret-result (send self :execute-handle-pose-compensation :yes yes :look-at-handle look-at-handle))
     ;; grasp in new handle coords
     (when (and grasp ret-result) ;; regrasp only when recognition is succeeded
       (warn "regrasp~%")
       (setq approach-result (send self :approach-handle :tm time))
       (if approach-result
           (progn (setq grasp-result (send self :grasp)) ;; grasp only when approach is succeeded
                  (unless  grasp-result
                    (warn "grasp failed~%")
                    (send self :resume-handle-pose)
                    )
                  )
         (progn (warn "approach failed~%")
                (send self :resume-handle-pose)
                )
         )
       (send motion :model2real :time time :wait-interpolation t)
       )
     (setq handle-pose-estimate-flag nil)
     (append ret-result (list approach-result grasp-result)) ;; return all results
     )
   )
  (:resume-handle-pose
   ()
   (if handle-pose-old-robot-coords
       (progn
         (send motion :robot :newcoords handle-pose-old-robot-coords) ;; revert compensation result
         (send self :publish-body-relative-steering-coords) ;; update steering coords in rviz
         )
     (warn "previous handle pose does not exists~%")
     )
   )
  (:turn ;; estimated as :relative nil
   (angle &key (mode :sequence) (weight 1.0) (rad nil))
   (let ((pub-msg (instance std_msgs::Float32 :init))
         handle-deg remain-angle)
     ;; if radius, transform to deg
     (if rad
         (setq handle-deg (/ (rad2deg angle) weight))
       (setq handle-deg (/ angle weight))
       )

     ;; turn can be used only when grasped
     (unless (equal (send self :controller-state :handle) :running)
       (warn "Robot do not grasp handle~%")
       (return-from :turn nil) ;; return nil because incorrect use of function
       )

     (warn "turn ~A[deg]~%" handle-deg)

     ;; turn-handle using motion
     (setq remain-angle
           (case mode
             (:once (send motion :turn-handle-once handle-deg :relative nil :use-controller t))
             (:sequence (send motion :turn-handle handle-deg :relative nil :use-controller t))
             (:velocity (send motion :turn-handle-velocity handle-deg max-turn-handle-omega :rate ros-rate :relative nil :use-controller t))
             (t (send motion :turn-handle handle-deg :relative nil :use-controller t))
             )
           )
     ;; publish target angle
     (send pub-msg :data handle-deg)
     (ros::publish "/drive/controller/goal_handle_angle" pub-msg)
     remain-angle
     )
   )
  (:approach-handle
   (&rest args &key (sync-controller (equal (send self :controller-state :handle) :running)) &allow-other-keys)
   (let (approach-result)
     (setq approach-result
           (send* motion :approach-handle :sync-controller sync-controller args)
           )
     (send self :controller-state :handle :approach)
     approach-result
     )
   )
  (:approach-accel
   (&rest args &key (limb :rleg) (sync-controller (equal (send self :controller-state :accel) :running)) &allow-other-keys)
   (send* motion :approach-accel-pedal :limb limb :sync-controller sync-controller args)
   (send self :accel-origin (send motion :robot limb :ankle-p :joint-angle))
   (send self :controller-state :accel :approach) ;; enbale accel
   )
  (:approach-brake
   (&rest args)
   (send* motion :approach-brake-pedal args)
   (send self :controller-state :brake :approach) ;; enbale brake
   )
  (:approach-floor
   (&rest args &key (start-impedance t) (set-ref-force nil) &allow-other-keys)
   (send* motion :approach-floor
          :start-impedance start-impedance :set-ref-force set-ref-force
          args) ;; approach to floor
   )
  (:approach-fist
   (&rest args &key (start-impedance t) (set-ref-force nil) (reach-until-touch nil) &allow-other-keys)
   (send* motion :approach-fist
          :start-impedance start-impedance :set-ref-force set-ref-force
          :reach-until-touch reach-until-touch
          args) ;; approach fist to seat
   )
  (:step-accel ;; mainly for command-line
   (command &rest args)
   (unless (or (equal (send self :controller-state :accel) :approach) (equal (send self :controller-state :accel) :running))
     (warn "accel is not approached~%")
     (return-from :step-accel nil)
     )
   (send* motion :step-accel-pedal command args) ;; step accel
   (send self :controller-state :accel :running)
   )
  (:detatch-accel
   ()
   (send motion :detatch-accel-pedal :use-controller t) ;; detatch accel for emergency
   (send self :controller-state :accel :release)
   )
  (:step-brake ;; mainly for command-line
   (command &rest args)
   (unless (or (equal (send self :controller-state :brake) :approach) (equal (send self :controller-state :brake) :running))
     (warn "brake is not approached~%")
     (return-from :step-brake nil)
     )
   (send* motion :step-brake-pedal command args) ;; step brake
   (send self :controller-state :brake :running)
   )
  (:support-by-leg
   (&rest args &key (offset #f(0 0 0)) (start-impedance t) (set-ref-force t) &allow-other-keys)
   (send* motion :approach-floor
          :offset offset :start-impedance start-impedance :set-ref-force set-ref-force
          args) ;; approach to floor
   )
  (:go-to-egress
   (&rest args) ;; todo: consider controller-state
   ;; stop accel
   (send self :accel-cmd 0.0)
   (send self :controller-mode :accel :stop)
   ;; stop handling
   (send self :controller-mode :handle :stop)
   ;; sync
   (when (and sync (not (send *ri* :simulation-modep)))
     (let ((sync-list nil))
       (when (equal (send self :controller-state :handle) :running)
         (push (send motion :grasp-arm) sync-list)
         )
       (when (equal (send self :controller-state :accel) :running)
         (push :rleg sync-list)
         )
       (when (equal (send self :controller-state :brake) :running)
         (push :lleg sync-list)
         )
       (send motion :sync-all-controller sync-list)
       )
     )
   ;; execute egress
   (send* motion :go-to-egress args)
   )

  ;; internal functions
  (:accel-cmd ;; convert command to motion and send to step-accel
   (cmd)
   (let ((cmd-motion (send self :get-pedal-motion-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A[deg]~%" cmd cmd-motion)
     (send self :step-accel (+ accel-origin cmd-motion) :relative nil :use-controller t)
     (send self :publish-current-step cmd-motion)
     (send self :publish-step-on-flag cmd)
     cmd-motion ;; return real command to robot
     )
   )
  (:brake-cmd ;; convert command to motion and send to step-brake
   (cmd)
   (let ((cmd-motion (send self :get-pedal-motion-from-cmd cmd release-brake-command step-brake-command)))
     (warn "brake ~A[cmd] -> ~A[deg]~%" cmd cmd-motion)
     (send self :step-brake (+ brake-origin cmd-motion) :relative nil :use-controller t)
     cmd-motion ;; return real command to robot
     )
   )
  (:neck-cmd
   (angle axis &key (time (* (+ (/ 1.0 ros-rate) 0.1) 1000))) ;; default time is rate-sec + offset
   (when (equal (send self :controller-state :neck) :running) ;; neck-state will be set as :running in :initialize function (neck should not move before :initialize function)
     (let ((pub-msg (instance std_msgs::Float32 :init))
           (topic-name (format nil "/drive/controller/neck_~A_angle" (string-trim "neck-" (string-downcase axis))))
           current-angle
           )
       (setq current-angle (send motion :look-around angle axis :relative nil :time time))
       (send pub-msg :data current-angle)
       (ros::publish topic-name pub-msg)
       )
     )
   )
  
  (:publish-current-step
   (step)
   (let ((pub-step (instance std_msgs::Float32 :init)))
     (send pub-step :data step)
     (ros::publish "/drive/controller/step" pub-step)
     )
   )
  (:publish-step-on-flag
   (cmd)
   (let ((pub-flag (instance std_msgs::Bool :init)))
     (send pub-flag :data (> cmd 0.5))
     (ros::publish "/drive/controller/step_on_flag" pub-flag)
     )
   )

  ;; handle controller callback
  (:handle-callback
   (msg)
   (when (send self :is-waiting-sync :handle)
     (warn "Sync ~A is executing. return.~%" :handle)
     (return-from :handle-callback nil)
     )
   (let* ((data (send msg :data)) ;; data is estimated as radius
          (threshold 1.0) ;; [deg]
          )
     (unless (equal (send self :controller-mode :handle) :operation)
       (warn "handle mode is ~A~%" (send self :controller-mode :handle))
       (return-from :handle-callback nil) ;; do not turn handle
       )
     (when (> (abs (- (rad2deg data) (send motion :car :handle-joint :joint-angle))) threshold)
       (send self :turn data :rad t :mode :velocity)
       )
     )
   )
  (:accel-callback
   (msg)
   (when (send self :is-waiting-sync :accel)
     (warn "Sync ~A is executing. return.~%" :accel)
     (return-from :accel-callback nil)
     )
   (let ((threshold 0.05) ;; 0-1 range command
         (accel-command (send msg :data))
         )
     (unless (equal (send self :controller-mode :accel) :operation)
       (warn "accel mode is ~A~%" (send self :controller-mode :accel))
       (setq accel-command 0.0) ;; forcely stop accel
       (return-from :accel-callback nil) ;; do not accel
       )
     ;; send accel command
     (when (or (not old-accel-command)
               (> (abs (- accel-command old-accel-command)) threshold)
               )
       (send self :accel-cmd accel-command)
       (setq old-accel-command accel-command)
       )
     )
   )
  (:brake-callback
   (msg)
   (let ((threshold 0.05) ;; 0-1 range command
         (brake-command (send msg :data))
         )
     (when (or (not old-brake-command)
               (> (abs (- brake-command old-brake-command)) threshold)
               )
       (send self :brake-cmd brake-command)
       (send self :accel-cmd 0.0)
       (setq old-brake-command brake-command)
       )
     )
   )
  
  (:get-pedal-motion-from-cmd
   (data min-pedal-command max-pedal-command) ;; data will be 0.0 from 1.0
   (+ min-pedal-command (* data (- max-pedal-command min-pedal-command)))
   )
  
  (:neck-y-callback
   (msg)
   (when (send self :is-waiting-sync :neck)
     (warn "Sync ~A is executing. return.~%" :neck)
     (return-from :neck-y-callback nil)
     )
   (unless (equal (send self :controller-mode :neck) :operation)
     (warn "neck mode is ~A~%" (send self :controller-mode :neck))
     (return-from :neck-y-callback nil) ;; do not turn handle
     )
   (let ((neck-command (send msg :data)))
     (send self :neck-cmd neck-command :neck-y)
     )
   )
  (:neck-p-callback
   (msg)
   (when (send self :is-waiting-sync :neck)
     (warn "Sync ~A is executing. return.~%" :neck)
     (return-from :neck-p-callback nil)
     )
   (unless (equal (send self :controller-mode :neck) :operation)
     (warn "neck mode is ~A~%" (send self :controller-mode :neck))
     (return-from :neck-p-callback nil) ;; do not turn handle
     )
   (let ((neck-command (send msg :data)))
     (send self :neck-cmd neck-command :neck-p)
     )
   )

  ;; handling visualization
  (:publish-body-relative-steering-coords
   ()
   (let* ((body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
          (steering-coords (send (send motion :car :vehicle-handle :steering-center) :copy-worldcoords))
          (torus (instance jsk_recognition_msgs::Torus :init))
          (pose (instance geometry_msgs::Pose :init))
          (current-ros-time (ros::time-now))
          torus-array-msg header
          body-relative-steering-coords
          body-relative-steering-coords-position
          body-relative-steering-coords-quaternion
          )
     ;; translate: worldcoords of center -> body related coords of steering-center
     (setq body-relative-steering-coords (send body-coords :transformation steering-coords))
     (setq body-relative-steering-coords-position (scale 0.001 (send body-relative-steering-coords :pos))) ;; eus[mm]->ros[m]
     (setq body-relative-steering-coords-quaternion (matrix2quaternion (send body-relative-steering-coords :rot)))
     ;; make torus
     (send torus :large_radius (* 0.001 (/ 380 2)))
     (send torus :small_radius 0.02)
     (send pose :position
           (instance geometry_msgs::Point :init
                     :x (elt body-relative-steering-coords-position 0)
                     :y (elt body-relative-steering-coords-position 1)
                     :z (elt body-relative-steering-coords-position 2)
                     )
           )
     (send pose :orientation
           (instance geometry_msgs::Quaternion :init
                     :x (elt body-relative-steering-coords-quaternion 1)
                     :y (elt body-relative-steering-coords-quaternion 2)
                     :z (elt body-relative-steering-coords-quaternion 3)
                     :w (elt body-relative-steering-coords-quaternion 0)
                     )
           )
     (send torus :header (instance std_msgs::Header :init :stamp current-ros-time :frame_id "BODY"))
     (send torus :pose pose)
     ;; publish
     (setq torus-array-msg (instance jsk_recognition_msgs::TorusArray :init))
     (send torus-array-msg :toruses (list torus))
     (send torus-array-msg :header
           (instance std_msgs::Header :init :stamp current-ros-time :frame_id "BODY"))
     (ros::publish "/drive/controller/currest_steering" torus-array-msg)
     )
   )
  (:record-handling-end-coords
   (&key (reset-buffer nil))
   (when (and record-handling-end-coords-flag (equal (send self :controller-state :handle) :running))
     (let* ((model-angle-vector (send motion :robot :angle-vector)))
       (when reset-buffer
         (setq handling-end-coords-list nil)
         )
       (send motion :robot :angle-vector (send *ri* :state :angle-vector))
       (push (send (send motion :robot (send motion :grasp-arm) :end-coords) :copy-worldcoords) handling-end-coords-list)
       (send motion :robot :angle-vector model-angle-vector) ;; resume
       )
     )
   )
  (:publish-steering-trajectory
   (&key (project-to-steering nil) (pub-topic nil))
   (unless (stringp pub-topic)
     (setq pub-topic "/drive/controller/handling_trajectory")
     )
   (let* ((body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
          (steering-coords (send motion :car :vehicle-handle :steering-center))
          (polygon-stamped-msg (instance geometry_msgs::PolygonStamped :init))
          body-relative-ec ec-relative-steering-coords tmp-ec tmp-pos tmp-point32 tmp-point32-list
          )
     ;; translate end-coords: worldcoords of grasp-point -> body related coords of steering-point
     (dolist (ec (reverse handling-end-coords-list))
       (setq tmp-ec (send ec :copy-worldcoords))
       (setq ec-relative-steering-coords (send tmp-ec :transformation steering-coords :local))
       (when project-to-steering
         (send tmp-ec :translate (float-vector 0 0 (elt (send ec-relative-steering-coords :pos) 2)) :local) ;; project end-coords(estimated as grasp-point) to same plane as steering
         )
       (setq body-relative-ec (send body-coords :transformation tmp-ec :local)) ;; make relative coords from body-link to end-coords
       (setq tmp-pos (scale 0.001 (send body-relative-ec :pos)))
       (push (instance geometry_msgs::Point32 :init
                       :x (elt tmp-pos 0)
                       :y (elt tmp-pos 1)
                       :z (elt tmp-pos 2))
             tmp-point32-list
             )
       )
     ;; publish
     (send polygon-stamped-msg :polygon :points tmp-point32-list)
     (send polygon-stamped-msg :header :stamp (ros::time-now))
     (send polygon-stamped-msg :header :frame_id "BODY")
     (ros::publish pub-topic polygon-stamped-msg)
     )
   )

  ;; steering estimation by torus-finder
  (:estimate-steering-by-torus-finder
   (&key (wait-sec 30.0))
   (let (wait-start)
     (send self :publish-steering-trajectory :project-to-steering t :pub-topic "/drive/controller/torus_finder_input")
     (setq torus-finder-estimated-robot-coords nil)
     (setq wait-start (ros::time-now))
     (while (and (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) wait-sec)
                 (not torus-finder-estimated-robot-coords))
       (ros::spin-once)
       (ros::sleep)
       )
     (unless torus-finder-estimated-robot-coords
       (return-from :estimate-steering-by-torus-finder nil) ;; failed estimation
       )
     (send self :apply-steering-estimation torus-finder-estimated-robot-coords)
     (send self :publish-body-relative-steering-coords) ;; update steering coords in rviz
     t
     )
   )
  (:torus-finder-callback
   (msg)
   (when torus-finder-estimation-flag
     (warn "Update handle coordinates according to torus_finder~%")
     (let ((pose (send msg :pose))
           body-relative-steering-coords)
       ;; (warn "old: ~A~%" (send motion :car :vehicle-handle :copy-worldcoords))
       (setq body-relative-steering-coords (make-coords
                                            :pos (scale 1000 (float-vector (send pose :position :x) (send pose :position :y) (send pose :position :z))) ;; ros[m] -> eus[mm]
                                            :rot (quaternion2matrix (float-vector (send pose :orientation :w) (send pose :orientation :x)
                                                                                  (send pose :orientation :y) (send pose :orientation :z)))
                                            )
             )
       (setq torus-finder-estimated-robot-coords
             (send self :calculate-new-robot-coords-from-body-relative-steering-coords body-relative-steering-coords)
             )
       )
     )
   )
  (:apply-steering-estimation
   (estimated-robot-coords &key (yes nil))
   (let ((old-robot-coords (send motion :robot :copy-worldcoords))
          )
     (send motion :robot :newcoords estimated-robot-coords)
     (send *viewer* :draw-objects)
     (warn "apply new robot coords~%")
     (unless (or yes (y-or-n-p)) ;; y-or-n-p is not evaluated when yes is t
       (warn "revert old robot coords~%")
       (send motion :robot :newcoords old-robot-coords) ;; revert
       )
     (send motion :robot :copy-worldcoords)
     )
   )
  (:calculate-new-robot-coords-from-body-relative-steering-coords
   (body-relative-steering-coords)
   (let ((robot-coords (send motion :robot :copy-worldcoords))
         (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
         (steering-coords (send motion :car :vehicle-handle :steering-center))
         real-steering-worldcoords steering-relative-body-coords real-body-worldcoords
         body-relative-robot-coords
         )
     (warn "original robot-coords: ~A~%" robot-coords)
     (warn "original body-coords: ~A~%" body-coords)
     (warn "body2steering: ~A~%" body-relative-steering-coords)
     (setq steering-relative-body-coords (send body-relative-steering-coords :inverse-transformation))
     (setq real-body-worldcoords (send steering-coords :transform steering-relative-body-coords :local))
     (warn "new body-coords: ~A~%" real-body-worldcoords)
     (setq body-relative-robot-coords (send body-coords :transformation robot-coords))
     (setq robot-coords (send real-body-worldcoords :transform body-relative-robot-coords))
     (warn "new robot-coords: ~A~%" robot-coords)
     robot-coords
     )
   )

  ;; crank recognition
  (:handle-pose-callback
   (msg)
   (when handle-pose-estimate-flag ;; only when handle-pose-estimate-flag is enabled
     (let ((trans (send tf-listener :lookup-transform "BODY" 
                        (send msg :header :frame_id)
                        (send msg :header :stamp))) ;; cascaded-coords: body->camera
           body-relative-handle-pose
           )
       (unless trans
         (warn "Failed to solve tf~%")
         (return-from :handle-pose-callback nil)
         )
       (warn "Body2Camera: ~A~%" trans)
       (setq body-relative-handle-pose
             (send trans :transform (ros::tf-pose-stamped->coords msg))) ;; camera->handle_pose
       (warn "Camera2Handle: ~A~%" (ros::tf-pose-stamped->coords msg))
       (warn "Body2Handle: ~A~%" body-relative-handle-pose)
       (setq handle-pose-estimated-robot-coords
             (send self :calculate-new-robot-coords-from-body-relative-steering-coords body-relative-handle-pose)
             )
       )
     )
   )
  (:execute-handle-pose-compensation
   (&rest args &key (tm 2500) (limb (send motion :grasp-arm))
          (position-threshold 1000) (yes nil) (min-wait-sec 5.0) (max-wait-sec 30.0)
          (release-offset #f(-100 0 0)) (shoulder-y-deg 0.0) (look-at-handle t)
         )
   ;; wait handling   
   (let (wait-start
         old-head-angle-vector
         old-robot-coords old-body-coords distance
         release-result grasp-result approach-result
         compensation-result
         )
     (warn "stop neck control~%")
     (send self :controller-mode :neck :stop)
     (warn "release accel~%")
     (send self :accel-cmd 0.0) ;; force stop
     (send self :controller-mode :accel :stop)
     (warn "wait interpolation~%")
     (setq wait-start (ros::time-now))
     (while (and (or (send *ri* :interpolatingp (read-from-string (format nil ":~A-controller" (string-downcase limb))))
                     (send *ri* :interpolatingp))
                 (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) min-wait-sec))
       (ros::spin-once)
       (ros::sleep)
       )
     (when (or (send *ri* :interpolatingp (read-from-string (format nil ":~A-controller" (string-downcase limb))))
               (send *ri* :interpolatingp))
       (warn "interpolation do not finish~%")
       (return-from :execute-handle-pose-compensation nil)
       )
     (warn "release handle~%")
     (if (not (equal (send self :controller-state :handle) :release)) ;; prevent double release
         (setq release-result (send self :release :offset release-offset))
       (setq release-result t)
       )
     ;; (setq release-result (send self :release :offset release-offset))
     (when release-result
       ;; temporary ;;
       (send (send motion :robot) (send motion :grasp-arm) :shoulder-y :joint-angle shoulder-y-deg)
       (send motion :model2real :time tm :wait-interpolation t) ;; execute shoulder-y avoidance
       ;; temporary ;;
       (when look-at-handle
         (setq old-head-angle-vector (send motion :robot :head :angle-vector))
         (send (send motion :robot) :look-at-target (send motion :car :vehicle-handle :steering-center)) ;; look at handle
         (send motion :model2real :time tm :wait-interpolation t) ;; execute look-at
         )
       (warn "wait recognition~%")
       (setq wait-start (ros::time-now))
       (setq handle-pose-estimated-robot-coords nil) ;; reset recognition result
       (while (or (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) min-wait-sec)
                  (and (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) max-wait-sec)
                       (not handle-pose-estimated-robot-coords))
                  )
         (ros::spin-once)
         (ros::sleep)
         )
       (warn "compensate robot pose~%")
       (setq old-robot-coords (send motion :robot :copy-worldcoords))
       (setq old-body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
       (if handle-pose-estimated-robot-coords
           (progn (send self :apply-steering-estimation handle-pose-estimated-robot-coords :yes yes)
                  (setq compensation-result t)
                  )
         (progn (warn "cannot find handle_pose~%")
                (setq compensation-result nil)
                )
         )
       (setq distance (norm (v- (send (send (car (send (send motion :robot) :links)) :copy-worldcoords) :pos) (send old-body-coords :pos))))
       (when (> distance position-threshold)
         (warn "new body coords is too far: ~A~%" distance)
         (setq compensation-result nil)
         (send motion :robot :newcoords old-robot-coords) ;; revert
         )
       (when look-at-handle
         (send (send motion :robot) :head :angle-vector old-head-angle-vector) ;; look at front
         (send motion :model2real :time tm :wait-interpolation t) ;; resume head
         )
       (setq handle-pose-old-robot-coords old-robot-coords) ;; save old-robot-coords to revert
       )
     (send self :publish-body-relative-steering-coords) ;; update steering coords in rviz
     (and release-result compensation-result) ;; return result of all compensation process
     )
   )

  ;; handle angle estimation
  (:estimate-current-handle-angle
   (&rest args &key (offset #f(0 0 0)) (offset-wrt :local)) ;; offset is from end-coords to grasp-point  
   (let ((current-angle old-estimated-handle-angle)
         (half-range 180)
         (pub-msg (instance std_msgs::Float32 :init))
         current-end-coords steering-center-coords-at-zero-deg
         steering-center-relative-end-coords relative-current-pos-from-steerint-center estimated-angle
         steering-center-relative-grasp-point relative-grasp-point-pos-from-steerint-center estimated-angle-offset
         tmp-angle-vector
         )
     ;; preserve current angle-vector
     (setq tmp-angle-vector (send (send motion :robot) :angle-vector))
     ;; get current end-coords and resume angle-vector
     (send (send motion :robot) :angle-vector (send *ri* :state :potentio-vector))
     (setq current-end-coords (send (send motion :robot) (send motion :grasp-arm) :end-coords :copy-worldcoords))
     (send current-end-coords :translate offset offset-wrt)
     (send (send motion :robot) :angle-vector tmp-angle-vector)
     ;; fix steering-center coords to 0[deg], handle-joint is counter-clockwise
     (setq steering-center-coords-at-zero-deg (send (send (send motion :car :vehicle-handle :steering-center) :copy-worldcoords)
                                                    :rotate (deg2rad (- (send motion :car :handle-joint :joint-angle))) :z :local))
     ;; get relative-coords from steering-center
     (setq steering-center-relative-end-coords
           (send (send steering-center-coords-at-zero-deg :copy-worldcoords) :transformation current-end-coords :local))
     ;; get relative angle from polar-angle
     (setq relative-current-pos-from-steerint-center (send steering-center-relative-end-coords :pos))
     (setq estimated-angle (rad2deg (atan (elt relative-current-pos-from-steerint-center 1) (elt relative-current-pos-from-steerint-center 0))))
     ;; calculate offset angle from relative-coords of grasp-point
     (setq steering-center-relative-grasp-point
           (send (send (send motion :car :vehicle-handle :steering-center) :copy-worldcoords) :transformation (send (send motion :car (send motion :grasp-point)) :copy-worldcoords) :local))
     (setq relative-grasp-point-pos-from-steerint-center (send steering-center-relative-grasp-point :pos))
     (setq estimated-angle-offset (rad2deg (atan (elt relative-grasp-point-pos-from-steerint-center 1) (elt relative-grasp-point-pos-from-steerint-center 0))))
     (setq estimated-angle (+ estimated-angle estimated-angle-offset)) ;; current estimated angle is 0-360
     (let ((counter 0)) ;; slide range of estimated angle according to old-estimated-handle-angle
       (while (not (and (<= (- old-estimated-handle-angle half-range) estimated-angle)
                        (<= estimated-angle (+ old-estimated-handle-angle half-range))
                        (<= (send motion :car :handle-joint :min-angle) estimated-angle)
                        (<= estimated-angle (send motion :car :handle-joint :max-angle))
                        )
                   )
         (when (< estimated-angle (- old-estimated-handle-angle half-range))
           (setq estimated-angle (+ estimated-angle (* half-range 2)))
           )
         (when (> estimated-angle (+ old-estimated-handle-angle half-range))
           (setq estimated-angle (- estimated-angle (* half-range 2)))
           )
         (when (> counter 10)
           (warn "cannnot estimate~%")
           (return-from :estimate-current-handle-angle nil)
           )
         (setq counter (+ counter 1))
         )
       )
     (setq old-estimated-handle-angle estimated-angle)
     ;; publish estimated angle
     (send pub-msg :data estimated-angle)
     (ros::publish "/drive/controller/estimated_handle_angle" pub-msg)
     estimated-angle
     )
   )

  ;; state management
  (:controller-state
   (target &optional (new-state nil))
   ;; :initialize -> :approach <-> :running (grasp/step) 
   ;;                          <-> :release (release/detouch)
   (let ((target-state (read-from-string (format nil ":~A-state" (string-downcase target))))) ;; plist assumes :<target>-state
     ;; update to new state
     (when (member new-state state-list) ;; update only when different value applied
       (send self :put target-state new-state)
       (send self :set-synchronize-flag target) ;; synchronize command when mode updated
       )
     (send self :get target-state)
     )
   )  
  
  ;; mode management
  (:controller-mode
   (target &optional (new-mode nil))
   ;; :stop, :operate, :recognize
   (let ((target-mode (read-from-string (format nil ":~A-mode" (string-downcase target))))
         (pub-mode (instance std_msgs::String :init))
         ) ;; plist assumes :<target>-mode
     (when (member new-mode mode-list)
       (send self :set-synchronize-flag target) ;; synchronize command when mode updated
       (send self :put target-mode new-mode)
       (send pub-mode :data (string-downcase (send self :get target-mode)))
       (eval (list 'ros::publish (format nil "/drive/controller/~A_mode" (string-downcase target)) pub-mode))
       )
     (send self :get target-mode)
     )
   )

  (:set-synchronize-flag
   (target)
   (let ((target-flag (read-from-string (format nil ":~A-sync" (string-downcase target)))))
     (send self :put target-flag t)
     )
   )

  (:is-waiting-sync
   (target)
   (or (send self :get (read-from-string (format nil ":~A-sync" (string-downcase target))))
       (send self :get :all-sync)
       )
   )

  (:execute-operation-synchronize
   ()
   (let ((sync-target-list (list :all :handle :accel :neck)) ;; :brake is out of sync
         target-flag
         )
     (dolist (target sync-target-list)
       (setq target-flag (read-from-string (format nil ":~A-sync" (string-downcase target))))
       (when (and (send self :get target-flag)
                  (ros::wait-for-service "/drive/operation/synchronize" 1))
         (warn "call sync ~A~%" (string-downcase target))
         (let ((req (instance drc_task_common::StringRequestRequest :init)))
           (send req :data (string-downcase target))
           (ros::service-call "/drive/operation/synchronize" req t)
           (unix::usleep 500000) ;; have to wait after service call because service call immediately back from executive but real server is running
           )
         (send self :put target-flag nil)
         )
       )
     )
   )

  ;; calculate diff
  (:calc-error-of-grasp-arm
   (&rest args &key (mode :angle-vector) &allow-other-keys)
   (let (diff-av
         (pub-diff (instance std_msgs::Float32 :init))
         )
     (setq diff-av (send* motion :calc-error-of-grasp-arm :mode mode args))
     (send pub-diff :data (norm diff-av))
     (ros::publish "/drive/controller/steering_diff_angle_vector" pub-diff)
     diff-av
     )
   )
  
  ;; accessor
  (:motion () motion)

  ;; setter
  (:accel-origin
   (&optional (angle nil))
   (when angle
     (setq accel-origin angle)
     )
   accel-origin
   )
  (:step-accel-command
   (&optional (angle nil))
   (let ((pub-msg (instance std_msgs::Float32 :init)))
     (when angle
       (setq step-accel-command angle)
       )
     (send pub-msg :data step-accel-command)
     (ros::publish "/drive/controller/max_step" pub-msg)
     step-accel-command
     )
   )
  (:release-accel-command
   (&optional (angle nil))
   (let ((pub-msg (instance std_msgs::Float32 :init)))
     (when angle
       (setq release-accel-command angle)
       )
     (send pub-msg :data release-accel-command)
     (ros::publish "/drive/controller/min_step" pub-msg)
     release-accel-command
     )
   )
  (:brake-origin (&optional (angle nil)) (when angle (setq brake-origin angle)) brake-origin)
  (:step-brake-command (&optional (angle nil)) (when angle (setq step-brake-command angle)) step-brake-command)
  (:release-brake-command (&optional (angle nil)) (when angle (setq release-brake-command angle)) release-brake-command)
  (:max-turn-handle-omega (&optional (val nil)) (unless val (setq max-turn-handle-omega val)) max-turn-handle-omega)
  (:record-handling-end-coords-flag
   (&optional (val 0))
   (unless (equal val 0)
     (setq record-handling-end-coords-flag val)
     (setq handling-end-coords-list nil)
     )
   record-handling-end-coords-flag
   )

  ;; main loop
  (:execute
   (&key (publish-steering-trajectory nil))
   (do-until-key
    (when (equal (send self :controller-state :handle) :running) ;; estimate handle angle only when handling
      (send self :estimate-current-handle-angle)
      )
    (send self :calc-error-of-grasp-arm :display-result nil)
    (send self :record-handling-end-coords)
    (send self :execute-operation-synchronize) ;; call service in ocs in service-callback causes hungup in executive
    (when publish-steering-trajectory
      (send self :publish-steering-trajectory)
      )
    (ros::spin-once)
    (ros::sleep)
    )
   )
  )

(defun robot-vehicle-main
  (&key (robot-name "robot") (real t) (postfix nil))
  (let (controller-name)
    (setq controller-name (format nil "~A-driving-controller" (string-downcase robot-name)))
    (when postfix
      (setq controller-name (format nil "~A-~A" controller-name (string-downcase postfix)))
      )
    (warn "make controller: ~A~%" controller-name)
    (setq *controller* (eval (list 'instance (read-from-string controller-name) :init :real real)))
    (warn "ready to execute~%")
    ;; (send *controller* :initialize) ;; initialize can be called from ui
    (send *controller* :execute)
    )
  )
