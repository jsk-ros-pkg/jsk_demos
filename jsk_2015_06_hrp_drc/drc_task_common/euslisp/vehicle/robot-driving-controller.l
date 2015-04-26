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
          step-accel-command step-brake-command
          release-accel-command release-brake-command
          is-handle-reached-to-target
          grasped-flag accel-flag
          max-turn-angle max-turn-angle-diff-deg
          old-handle-deg
          polygon-stamped-msg
          torus-finder-estimated-robot-coords
          handle-pose-estimated-robot-coords
          handle-pose-estimate-flag
          old-estimated-handle-angle
          old-accel-command
          old-brake-command
          tf-listener
          handling-end-coords-list
          record-handling-end-coords-flag
          ) 
  )

(defmethod robot-driving-controller
  (:init
   (&key ((:real tmp-real) nil)
         ((:motion tmp-motion) nil)
         ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         (approach-handle nil) (approach-pedal nil)
         ((:max-turn-angle tmp-max-turn-angle) 45.0)
         ((:max-turn-angle-diff tmp-max-turn-angle-diff) 15.0)
         )
   (when roseus-init
     (ros::roseus "robot_driving_controller")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance robot-driving-motion :init :real tmp-real))
     )
   (setq *motion* tmp-motion)
   (setq motion *motion*)
   (when approach-handle
     (send motion :approach-handle)
     )
   (when approach-pedal
     (send motion :approach-accel-pedal)
     (send motion :approach-brake-pedal)
     )
   ;; initialize instance valiables
   (setq ros-rate tmp-ros-rate
         grasped-flag nil
         accel-flag nil
         max-turn-angle tmp-max-turn-angle
         max-turn-angle-diff-deg tmp-max-turn-angle-diff
         torus-finder-estimated-robot-coords nil
         handle-pose-estimated-robot-coords nil
         old-estimated-handle-angle 0.0
         old-handle-deg 0.0
         old-accel-command nil
         old-brake-command nil
         handle-pose-estimate-flag nil
         handling-end-coords-list nil
         record-handling-end-coords-flag nil
         )
   ;; set defalut value for pedaling
   (setq step-accel-command 28
         release-accel-command 14
         step-brake-command 20
         release-brake-command 5
         )
   ;; subscriber for operator command
   (ros::subscribe "/drive/operation/accel_cmd"
                   std_msgs::Float64
                   #'send self :accel-callback)
   (ros::subscribe "/drive/operation/brake_cmd"
                   std_msgs::Float64
                   #'send self :brake-callback)
   (ros::subscribe "/drive/operation/grasp_cmd"
                   std_msgs::String
                   #'send self :grasp-callback)
   (ros::subscribe "/drive/operation/handle_cmd"
                   std_msgs::Float64
                   #'send self :handle-callback)
   (ros::advertise "/drive/controller/pedal_state" std_msgs::Bool 1 t)

   ;; handle-angle commands
   (ros::advertise "/drive/controller/goal_handle_angle" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/estimated_handle_angle" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/step" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/min_step" std_msgs::Float32 1 t)
   (ros::advertise "/drive/controller/max_step" std_msgs::Float32 1 t)

   ;; for steering visualization
   (ros::advertise "/drive/controller/currest_steering" jsk_recognition_msgs::TorusArray 1)
   (ros::advertise "/drive/controller/handling_trajectory" geometry_msgs::PolygonStamped 1)

   ;; for steering estimation
   (ros::subscribe "/drive/torus_finder/output/pose"
                   geometry_msgs::PoseStamped
                   #'send self :torus-finder-callback)
   (ros::advertise "/drive/torus_finder/input/polygon" geometry_msgs::PolygonStamped 1)
   
   ;; for crank recognition
   (setq tf-listener (instance ros::transform-listener :init))
   (ros::subscribe "/handle_pose"
                   geometry_msgs::PoseStamped
                   #'send self :handle-pose-callback)

   ;; for vehicle ui
   (ros::advertise-service "/drive/controller/initialize" std_srvs::Empty #'send self :initialize-callback)
   (ros::advertise-service "/drive/controller/grasp" std_srvs::Empty #'send self :grasp-callback)
   (ros::advertise-service "/drive/controller/release" std_srvs::Empty #'send self :release-callback)
   (ros::advertise-service "/drive/controller/correct" std_srvs::Empty #'send self :correct-callback)
   (ros::advertise-service "/drive/controller/set_max_step" drc_task_common::SetValue #'send self :set-max-step-callback)
   (ros::advertise-service "/drive/controller/set_min_step" drc_task_common::SetValue #'send self :set-min-step-callback)
   )

  ;; vehicle ui callback
  (:initialize-callback
   (req)
   (let ((res (send req :response)))
     (send self :initialize :sync t)
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
     (send self :correct)
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

  ;; interface command
  (:initialize
   (&key (sync nil))
   (when (and sync (not (send *ri* :simulation-modep)))
     (send motion :sync-all-controller)
     )
   (send motion :ride :init-pose t)
   (send motion :drive-init-pose-crank :limb :larm :move-arm nil) ;; todo: fix and remove
   (send self :approach-accel)
   ;; (send self :approach-floor)
   (send self :support-by-leg :start-ref-force nil)
   (send self :approach-handle)
   nil
   )
  (:grasp
   (&rest args)
   (let (grasp-result)
     (setq grasp-result (send* motion :grasp-handle args))
     (when grasp-result
       (setq grasped-flag t)
       ;; handle-angle is estimated to be right when grasped
       (setq old-estimated-handle-angle (send motion :car :handle-joint :joint-angle))
       )
     grasp-result
     )
   )
  (:release
   (&rest args)
   (let (release-result)
     (setq release-result (send* motion :release-handle args))
     (when release-result
       (setq grasped-flag nil)
       )
     release-result
     )
   )
  (:correct
   ()
   (let (ret-result)
     (setq handle-pose-estimate-flag t)
     (setq ret-result (send self :execute-handle-pose-compensation))
     (setq handle-pose-estimate-flag nil)
     ret-result
     )
   )
  (:turn
   (angle &key (mode :sequence) (weight 1.0) (rad nil) (supress-diff-max nil))
   (let ((pub-msg (instance std_msgs::Float32 :init))
         handle-deg real-handle-deg remain-angle)
     ;; if radius, transform to deg
     (if rad
         (setq handle-deg (/ (rad2deg angle) weight))
       (setq handle-deg (/ angle weight))
       )

     ;; turn can be used only when grasped
     (unless grasped-flag
       (warn "Robot do not grasp handle~%")
       (return-from :turn nil) ;; return nil because incorrect use of function
       )
     
     ;; supress in min-max angle
     (if (> handle-deg max-turn-angle)
         (setq handle-deg max-turn-angle)
       (if (< handle-deg (- max-turn-angle))
         (setq handle-deg (- max-turn-angle))
         )
       )
     ;; supress difference max
     (let ((turn-angle-diff-deg (- handle-deg old-handle-deg)))
       (if (and (> (abs turn-angle-diff-deg) max-turn-angle-diff-deg)
                supress-diff-max)
           (progn (setq real-handle-deg
                        (if (> turn-angle-diff-deg 0)
                            (+ old-handle-deg max-turn-angle-diff-deg)
                          (- old-handle-deg max-turn-angle-diff-deg)
                          )
                        )
                  (warn "max-diff-angle limitation ~A [deg] was enabled:~% original: ~A~% real: ~A~% "
                        max-turn-angle-diff-deg handle-deg real-handle-deg)
                  )
         (setq real-handle-deg handle-deg)
         )
       )
     (warn "turn ~A[deg]~%" real-handle-deg)

     ;; turn-handle using motion
     (setq remain-angle
           (case mode
             (:once (send motion :turn-handle-once real-handle-deg :relative nil :use-controller t))
             (:sequence (send motion :turn-handle real-handle-deg :relative nil :use-controller t))
             (t (send motion :turn-handle real-handle-deg :relative nil :use-controller t))
             )
           )
       
     (setq old-handle-deg real-handle-deg)
     ;; publish target angle
     (send pub-msg :data real-handle-deg)
     (ros::publish "/drive/controller/goal_handle_angle" pub-msg)
     remain-angle
     )
   )
  (:approach-handle
   (&rest args &key (sync-controller grasped-flag) &allow-other-keys)
   (let (approach-result)
     (setq approach-result
           (send* motion :approach-handle :sync-controller sync-controller args)
           )
     (setq grasped-flag nil)
     approach-result
     )
   )
  (:approach-accel
   ()
   (send motion :approach-accel-pedal)
   (setq accel-flag t) ;; enbale accel
   )
  (:approach-brake
   ()
   (send motion :approach-brake-pedal)
   )
  (:approach-floor
   (&rest args &key (offset #f(0 0 50)) (start-impedance t) (start-ref-force nil) &allow-other-keys)
   (send* motion :approach-floor :offset offset
          :start-impedance start-impedance :stop-ref-force start-ref-force
          args) ;; approach to floor
   )
  (:step-accel
   (command)
   (send motion :step-accel-pedal command :relative nil :use-controller t) ;; step accel
   )
  (:detatch-accel
   ()
   (send motion :detatch-accel-pedal :use-controller t) ;; detatch accel for emergency
   )
  (:step-brake
   (command)
   (send motion :step-brake-pedal command :relative nil :use-controller t) ;; step brake
   )
  (:support-by-leg
   (&rest args &key (offset #f(0 0 0)) (start-impedance t) (start-ref-force t) &allow-other-keys)
   (send* motion :approach-floor
          :offset offset :start-impedance start-impedance :stop-ref-force start-ref-force
          args) ;; approach to floor
   )

  ;; internal functions
  (:accel-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-motion-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     (send self :step-accel cmd-angle)
     cmd-angle ;; return real command to robot
     )
   )
  (:brake-cmd
   (cmd)
   (let ((cmd-angle (send self :get-pedal-motion-from-cmd cmd release-brake-command step-brake-command)))
     (warn "brake ~A[cmd] -> ~A [deg]~%" cmd cmd-angle)
     (send self :step-brake cmd-angle)
     cmd-angle ;; return real command to robot
     )
   )

  ;; handle controller callback
  (:handle-callback
   (msg)
   (let* ((data (send msg :data)) ;; data is estimated as radius
          (threshold 1.0) ;; [deg]
          )
     (when (> (abs (- (rad2deg data) (send motion :car :handle-joint :joint-angle))) threshold)
       (send self :turn data :rad t :mode :sequence)
       )
     )
   )
  (:accel-callback
   (msg)
   (let ((pub-step (instance std_msgs::Float32 :init))
         (pub-pedal-state (instance std_msgs::Bool :init))
         (threshold 0.05) ;; 0-1 range command
         (accel-command (send msg :data))
         )
     ;; check accel flag
     (unless accel-flag
       (warn "accel-cmd is disabled~%")
       (setq accel-command 0.0) ;; forcely stop accel
       )
     ;; send accel command
     (when (or (not old-accel-command)
               (> (abs (- accel-command old-accel-command)) threshold)
               )
       (send pub-step :data (send self :accel-cmd accel-command))
       (ros::publish "/drive/controller/step" pub-step)
       (setq old-accel-command accel-command)
       )
     (send pub-pedal-state :data accel-flag)
     (ros::publish "/drive/controller/pedal_state" pub-pedal-state)
     )
   )
  (:brake-callback
   (msg)
   (let ((pub-step (instance std_msgs::Float32 :init))
         (threshold 0.05) ;; 0-1 range command
         (brake-command (send msg :data))
         )
     (when (or (not old-brake-command)
               (> (abs (- brake-command old-brake-command)) threshold)
               )
       (send self :brake-cmd brake-command)
       (send pub-step :data (send self :accel-cmd 0.0)) ;; release accel
       (ros::publish "/drive/controller/step" pub-step)
       (setq old-brake-command brake-command)
       )
     )
   )
  (:get-pedal-motion-from-cmd
   (data min-pedal-command max-pedal-command) ;; data will be 0.0 from 1.0
   (+ min-pedal-command (* data (- max-pedal-command min-pedal-command)))
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
   (when (and record-handling-end-coords-flag grasped-flag)
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
   (&key (project-to-steering nil))
   (let* ((tmp-robot (copy-object (send motion :robot)))
          (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
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
     (warn "publish data~%")
     (send polygon-stamped-msg :polygon :points tmp-point32-list)
     (send polygon-stamped-msg :header :stamp (ros::time-now))
     (send polygon-stamped-msg :header :frame_id "BODY")
     (ros::publish "/drive/controller/handling_trajectory" polygon-stamped-msg)
     )
   )

  ;; steering estimation
  (:estimate-steering-by-torus-filter
   (&key (limb :larm))
   (let* ((tmp-robot (copy-object (send motion :robot)))
          (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
          (steering-coords (send motion :car :vehicle-handle :steering-center))
          header polygon
          body-relative-ec ec-relative-steering-coords tmp-ec tmp-pos tmp-point32 tmp-point32-list
          end-coords-list)
     ;; collect end-coords of real robot
     (setq polygon-stamped-msg (instance geometry_msgs::PolygonStamped :init))
     (setq header (send polygon-stamped-msg :header))
     (setq polygon (send polygon-stamped-msg :polygon))
     (warn "start recording~%")
     (do-until-key
       (send tmp-robot :angle-vector (send *ri* :state :angle-vector))
       (push (send (send tmp-robot limb :end-coords) :copy-worldcoords) end-coords-list)
       )
     (warn "finish recording~%")
     ;; translate end-coords: worldcoords of grasp-point -> body related coords of steering-point
     (dolist (ec (reverse end-coords-list))
       (setq tmp-ec (send ec :copy-worldcoords))
       (setq ec-relative-steering-coords (send tmp-ec :transformation steering-coords :local))
       (send tmp-ec :translate (float-vector 0 0 (elt (send ec-relative-steering-coords :pos) 2)) :local) ;; project end-coords(estimated as grasp-point) to same plane as steering
       (setq body-relative-ec (send body-coords :transformation tmp-ec :local)) ;; make relative coords from body-link to end-coords
       (setq tmp-pos (send body-relative-ec :pos))
       ;; (setq tmp-pos (send ec :pos))
       (setq tmp-point32 (instance geometry_msgs::Point32 :init))
       (setq tmp-pos (scale 0.001 tmp-pos))
       (send tmp-point32 :x (elt tmp-pos 0))
       (send tmp-point32 :y (elt tmp-pos 1))
       (send tmp-point32 :z (elt tmp-pos 2))
       (push tmp-point32 tmp-point32-list)
       )
     ;; publish
     (warn "publish data~%")
     (send polygon :points tmp-point32-list)
     (send header :stamp (ros::time-now))
     (send header :frame_id "BODY")
     (ros::publish "/drive/torus_finder/input/polygon" polygon-stamped-msg)
     (ros::spin-once)
     (ros::sleep)
     )
   )  
  (:torus-finder-callback
   (msg)
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
  (:republish-steering-trajectory
   ()
   (ros::publish "/drive/torus_finder/input/polygon" polygon-stamped-msg)
   )
  (:apply-steering-estimation
   (estimated-robot-coords &key (yes nil))
   (let ((old-robot-coords (send motion :robot :copy-worldcoords))
          )
     (send motion :robot :newcoords estimated-robot-coords)
     (send *viewer* :draw-objects)
     (unless (or yes (y-or-n-p)) ;; y-or-n-p is not evaluated when yes is t
       (send motion :robot :newcoords old-robot-coords) ;; revert
       )
     (send motion :robot :copy-worldcoords)
     )
   )
  (:calculate-new-robot-coords-from-body-relative-steering-coords
   (body-relative-steering-coords)
   (let ((tmp-robot (copy-object (send motion :robot)))
         (body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
         (steering-coords (send motion :car :vehicle-handle :steering-center))
         real-steering-worldcoords steering-relative-body-coords real-body-worldcoords estimated-robot-coords)
     (warn "old: ~A~%" (send tmp-robot :copy-worldcoords))     
     (warn "trans: ~A~%" body-relative-steering-coords)
     (setq steering-relative-body-coords (send body-relative-steering-coords :inverse-transformation))
     (setq real-body-worldcoords (send steering-coords :transform steering-relative-body-coords :local))
     (warn "new body: ~A~%" real-body-worldcoords)
     (send tmp-robot :transform (send body-coords :transformation real-body-worldcoords :world) :world)
     (setq estimated-robot-coords (send tmp-robot :copy-worldcoords))
     (warn "new robot: ~A~%" estimated-robot-coords)
     estimated-robot-coords
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
       (setq body-relative-handle-pose
             (send trans :transform (ros::tf-pose-stamped->coords msg))) ;; camera->handle_pose
       (setq handle-pose-estimated-robot-coords
             (send self :calculate-new-robot-coords-from-body-relative-steering-coords body-relative-handle-pose)
             )
       )
     )
   )
  (:execute-handle-pose-compensation
   (&key (tm 2500) (wait-sec 30.0) (limb (send motion :grasp-arm))
         (position-threshold 1000)
         )
   ;; wait handling   
   (let (wait-start old-accel-flag
         old-head-angle-vector
         old-robot-coords old-body-coords distance
         release-result grasp-result approach-result
         compensation-result
         )
     (warn "wait interpolation~%")
     (setq wait-start (ros::time-now))
     (while (and (or (send *ri* :interpolatingp (read-from-string (format nil ":~A-controller" (string-downcase limb))))
                     (send *ri* :interpolatingp))
                 (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) wait-sec))
       (ros::spin-once)
       (ros::sleep)
       )
     (when (or (send *ri* :interpolatingp (read-from-string (format nil ":~A-controller" (string-downcase limb))))
               (send *ri* :interpolatingp))
       (warn "interpolation do not finish~%")
       (return-from :execute-handle-pose-compensation nil)
       )
     (warn "release handle~%")
     (setq old-accel-flag accel-flag)
     (setq accel-flag nil) ;; disable accel
     (send self :accel-cmd 0.0) ;; force stop
     (if grasped-flag
         (setq release-result (send self :release))
       (setq release-result t)
       )
     (when release-result
       ;; temporary ;;
       (send (send motion :robot) (send motion :grasp-arm) :shoulder-y :joint-angle 0)
       ;; temporary ;;
       (setq old-head-angle-vector (send motion :robot :head :angle-vector))
       (send (send motion :robot) :look-at-target (send motion :car :vehicle-handle :steering-center)) ;; look at handle
       (when (send motion :real)
         (send *ri* :angle-vector (send motion :robot :angle-vector) tm)
         (send *ri* :wait-interpolation)
         )
       (warn "wait recognition~%")
       (setq wait-start (ros::time-now))
       (setq handle-pose-estimated-robot-coords nil) ;; reset recognition result
       (while (and (< (- (send (ros::time-now) :to-sec) (send wait-start :to-sec)) wait-sec)
                   (not handle-pose-estimated-robot-coords))
         (ros::spin-once)
         (ros::sleep)
         )
       (warn "compensate robot pose~%")
       (setq old-robot-coords (send motion :robot :copy-worldcoords))
       (setq old-body-coords (send (car (send (send motion :robot) :links)) :copy-worldcoords))
       (if handle-pose-estimated-robot-coords
           (progn (send self :apply-steering-estimation handle-pose-estimated-robot-coords :yes t)
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
       ;; resume
       (warn "regrasp~%")
       (setq approach-result (send self :approach-handle :tm tm))
       (if approach-result
           (progn (setq grasp-result (send self :grasp)) ;; grasp only when approach is succeeded
                  (unless  grasp-result
                    (warn "grasp failed~%")
                    (send motion :robot :newcoords old-robot-coords) ;; revert
                    )
                  )
         (progn (warn "approach failed~%")
                (send motion :robot :newcoords old-robot-coords) ;; revert
                )
         )
       (send (send motion :robot) :head :angle-vector old-head-angle-vector) ;; look at front
       (when (send motion :real)
         (send *ri* :angle-vector (send (send motion :robot) :angle-vector) tm)
         (send *ri* :wait-interpolation)
         )
       )
     (setq accel-flag old-accel-flag)
     (and release-result compensation-result approach-result grasp-result) ;; return result of all compensation process
     )
   )

  ;; handle angle estimation
  (:estimate-current-handle-angle
   ()
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
     (let ((counter 0))
       (while (not (and (<= (- old-estimated-handle-angle half-range) estimated-angle) (<= estimated-angle (+ old-estimated-handle-angle half-range))))
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
  ;; accessor
  (:motion () motion)

  ;; setter
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
  
  (:step-brake-command (&optional (angle nil)) (when angle (setq step-brake-command angle)) step-brake-command)
  (:release-brake-command (&optional (angle nil)) (when angle (setq release-brake-command angle)) release-brake-command)
  (:max-turn-angle (&optional (val nil)) (when val (setq max-turn-angle val)) max-turn-angle)
  (:max-turn-angle-diff (&optional (val nil)) (when val (setq max-turn-angle-diff-deg val)) max-turn-angle-diff-deg)
  (:grasped-flag (&optional (val 0)) (unless (equal val 0) (setq grasped-flag val)) grasped-flag)
  (:accel-flag (&optional (val 0)) (unless (equal val 0) (setq accel-flag val)) accel-flag)

  ;; main loop
  (:execute
   ()
   (let ((count 0))
     (do-until-key
      (send self :estimate-current-handle-angle)
      (send self :record-handling-end-coords)
      (ros::spin-once)
      (ros::sleep)
      )
     )
   )
  )
