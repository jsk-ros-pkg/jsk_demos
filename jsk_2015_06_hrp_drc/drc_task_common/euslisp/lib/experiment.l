;; for experiment.l
(load "package://drc_task_common/euslisp/lib/valve.l")
(load "package://drc_task_common/euslisp/lib/door.l")

(defun init (&key (simulation nil))
  ;; only support jaxon
  (load-robot-model :jaxon)
  (unless (boundp '*ri*)
    (jaxon_red-init))
  (ros::rate 10)
  (setq *tf-listener* (instance ros::transform-listener :init 30))
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))

  (when (and
         simulation
         (= (send (send *ri* :get-auto-balancer-param) :controller_mode) 1))
    ;; if auto-balancer is running, stop auto-balancer
    (if (y-or-n-p ";; Ready for stopping auto-balancer?")
        (progn
          (send *robot* :reset-pose)
          (send *robot* :head :neck-p :joint-angle 20)
          (send *robot* :fix-leg-to-coords (make-coords))
          (send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
          (if (< (norm (v- (send *ri* :state :potentio-vector)
                           (send *robot* :angle-vector)))
                 40.0)
              (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
            (progn
              (if (y-or-n-p ";; Move to reset-pose in 8 seconds ?")
                  (send *ri* :angle-vector (send *robot* :angle-vector) 8000)
                (return))))
          (send *ri* :wait-interpolation)
          (send *ri* :stop-auto-balancer)
          )
      (progn
        (warn ";; Retry after stopping auto-balancer~%")
        (return))))

  ;; check hand controller
  (send *ri* :start-grasp)
  (send *ri* :stop-grasp)

  (when (send (send *ri* :get-emergency-stopper-param) :is_stop_mode)
    (if (y-or-n-p ";; Ready for emergency release motion ?")
        (send *ri* :emergency-release-motion)
      (return)))

  ;; start auto-balancer
  (send *ri* :start-auto-balancer :limbs (list :rleg :lleg))
  )

(defun wait-for-recognition ()
    (setq *perception-succeeded* nil)
    (while (not (and *perception-succeeded*
                     (< (send (ros::time- *perception-start-time* *perception-succeeded-time*) :to-sec) 1.0)))
      (when *perception-succeeded*
        (ros::ros-info "time diff: ~A" (send (ros::time- *perception-succeeded-time*  *perception-start-time*) :to-sec)))
      (ros::spin-once)
      (ros::sleep)
      ))

(defun run-valve (dx collision-num trajectory-num speed-factor)
  ;; dx := mm
  ;; steps
  ;; 0. setup ros stuff
  ;; 1. set octree resolution
  ;; 2. clear laser cache
  ;; 3. wait for recognition
  ;; 4. plan motion
  ;; 5. exec motion
  
  ;; 0. setup ros stuff
  ;; reset robot
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  ;; 4. plan motion
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  (bench
   (if *avs*
       ;; 5. exec motion
       (progn
         (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
           (ros::ros-info "execution: ~A" tm))
         (send *ri* :wait-interpolation))
       ;;nil
     (ros::ros-warn "Failed to solve ik")))
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  )

(defun run-valve-walk-with-one-perception (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              ;;:ik-type :use-torso
                              :ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver1* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  (if *avs*
      (progn
        (ros::ros-info "go-pos-arg: ~A" (send *avs* :go-pos-arg))
        (send* *ri* :go-pos (send *avs* :go-pos-arg))
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  )


(defun run-valve-walk-with-one-perception* (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              ;;:ik-type :use-torso
                              :ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (bench
   (ros::ros-info "planning motion")    ;solve ik to normalize time
   (setq *avs* (send *ik-solver1* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  
  (setq *stand-coords* (valve-stand-coords *valve*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  (bench
   (ros::ros-info "planning motion2")
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  (if *avs*
      (progn
        (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
        (send* *ri* :go-pos (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  )


(defun run-valve-walk-with-two-perception* (dx dx2 collision-num trajectory-num collision-num2 trajectory-num2 speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              ;;:ik-type :use-torso
                              :ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (bench
   (ros::ros-info "planning motion")    ;solve ik to normalize time
   (setq *avs* (send *ik-solver1* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  
  (setq *stand-coords* (valve-stand-coords *valve*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  ;;(ros::ros-info "go-pos-arg: ~A" (send *avs* :go-pos-arg))
  (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  ;; move head to valve
  (send *robot* :head :look-at (send *valve* :worldpos))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send* *ri* :go-pos (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx2))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (bench
   (ros::ros-info "planning motion")    ;solve ik to normalize time
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num2
                     :stop 50
                     :collision-num collision-num2 :debug-view nil))
   (ros::ros-info "done"))
  (if *avs*
      (progn
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))
  
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  
  )

(defun run-valve-walk-with-tracking* (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              ;;:ik-type :use-torso
                              :ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  (setq *perception-start-time* (ros::time-now))
  (wait-for-recognition)
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (setq *stand-coords* (valve-stand-coords *valve*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  ;;(ros::ros-info "go-pos-arg: ~A" (send *avs* :go-pos-arg))
  (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  (send* *ri* :go-pos-no-wait (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  (while (> (length (send *ri* :get-remaining-foot-step-sequence)) 2)
   (ros::sleep)
   (ros::spin-once))
  (bench
   (ros::ros-info "planning motion")    ;solve ik to normalize time
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  (ros::ros-info "wait go-pos")
  (while (not (= (length (send *ri* :get-remaining-foot-step-sequence)) 0))
   (ros::sleep))
  (if *avs*
      (progn
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))
  
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  )

(defun run-door (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *traj* *door* *recog-result*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (send *ri* :start-impedance :arms :m-p 10 :d-p 800 :k-p 300 :m-r 2.0 :d-r 80 :k-r 40)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/valve_detection/icp_registration/output_pose")
  (ros::subscribe "/door_detection/door_handle"
                  geometry_msgs::PoseStamped
                  #'test-door-recognition-callback)
  ;; 1. set octree resolution
  
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/door_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-succeeded-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
    (bench
   (if *avs*
       ;; 5. exec motion
       (progn
         (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
           (ros::ros-info "execution: ~A" tm))
         (send *ri* :wait-interpolation))
       ;;nil
     (ros::ros-warn "Failed to solve ik")))
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :rarm :move-end-pos (float-vector 500 0 0))
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )

(defun run-door-walk-with-two-perception* (dx dx2 collision-num trajectory-num collision-num2 trajectory-num2 speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *traj* *door* *recog-result*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (send *ri* :start-impedance :arms :m-p 10 :d-p 800 :k-p 300 :m-r 2.0 :d-r 80 :k-r 40)
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :fullbody-with-standcoords
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/valve_detection/icp_registration/output_pose")
  (ros::subscribe "/door_detection/door_handle"
                  geometry_msgs::PoseStamped
                  #'test-door-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/door_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  
  (setq *stand-coords* (door-stand-coords *door*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  
  (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  (send* *ri* :go-pos (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/door_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx2))

  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (bench
   (ros::ros-info "planning motion2")
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num2
                     :stop 50
                     :collision-num collision-num2 :debug-view nil))
   (ros::ros-info "done")
   )
     (if *avs*
      (progn
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))

    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :rarm :move-end-pos (float-vector 500 0 0))
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )

(defun run-door-walk-with-tracking (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *traj* *door* *recog-result*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (send *ri* :start-impedance :arms :m-p 10 :d-p 800 :k-p 300 :m-r 2.0 :d-r 80 :k-r 40)
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :fullbody-with-standcoords
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/valve_detection/icp_registration/output_pose")
  (ros::subscribe "/door_detection/door_handle"
                  geometry_msgs::PoseStamped
                  #'test-door-recognition-callback)
  (setq *perception-start-time* (ros::time-now))
  (wait-for-recognition)
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (setq *stand-coords* (door-stand-coords *door*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
  (send *ri* :go-pos-no-wait
         (elt (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*) 0)
         (elt (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*) 1)
         (elt (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*) 2))
  (while (> (length (send *ri* :get-remaining-foot-step-sequence)) 2)
   (ros::sleep)
   (ros::spin-once))
  
  (bench
   (ros::ros-info "planning motion2")
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done")
   )
  (ros::ros-info "wait go-pos")
  (while (not (= (length (send *ri* :get-remaining-foot-step-sequence)) 0))
   (ros::sleep))
     (if *avs*
      (progn
        (bench
         ;; 5. exec motion
         (progn
           (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
             (ros::ros-info "execution: ~A" tm))
           (send *ri* :wait-interpolation))))
       ;;nil
    (ros::ros-warn "Failed to solve ik"))

    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :rarm :move-end-pos (float-vector 500 0 0))
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )

(defun run-door-walk-with-one-perception* (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *traj* *door* *recog-result*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (send *ri* :start-impedance :arms :m-p 10 :d-p 800 :k-p 300 :m-r 2.0 :d-r 80 :k-r 40)
  (setq *ik-solver1* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :fullbody-with-standcoords
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (setq *ik-solver2* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/valve_detection/icp_registration/output_pose")
  (ros::subscribe "/door_detection/door_handle"
                  geometry_msgs::PoseStamped
                  #'test-door-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/door_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (setq *before-robot-coords* (send *robot* :foot-midcoords))
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver1* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  
  (setq *stand-coords* (door-stand-coords *door*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords *stand-coords*)
  (bench
   (ros::ros-info "planning motion2")
   (setq *avs* (send *ik-solver2* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done")
   )
  
   (if *avs*
       ;; 5. exec motion
       (progn
         (ros::ros-info "go-pos-arg: ~A" (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
         (send* *ri* :go-pos (go-pos-arg-from-two-coords *before-robot-coords* *stand-coords*))
         (bench
         (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
           (ros::ros-info "execution: ~A" tm))
         (send *ri* :wait-interpolation)))
       ;;nil
     (ros::ros-warn "Failed to solve ik"))
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :rarm :move-end-pos (float-vector 500 0 0))
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )


(defun test-valve-perception-with-clear-cache ()
    (bench
   (setq *perception-start-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  )

(defun move-door-to-valve ()
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.15))
  (send *ri* :go-pos -0.5 -0.2 90)
  (send *ri* :go-pos 0.8 -0.6 0)
  (send *ri* :go-pos 0.5 0 -90)
  (let ((org-stride-parameter (send (send *ri* :get-gait-generator-param) :stride_parameter)))
    (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.1 20.0 0.15))
    (send *ri* :go-pos 0.0 0.0 -30)
    (send *ri* :set-gait-generator-param :stride-parameter org-stride-parameter))
  ;;(send *ri* :go-pos 0.0 0.5 -30)
  )

(defun move-valve-to-door ()
  (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.05 10.0 0.15))
  (send *ri* :go-pos -0.3 -0.3 0)
  (send *ri* :go-pos 0.0 -0.3 -45)
  (let ((org-stride-parameter (send (send *ri* :get-gait-generator-param) :stride_parameter)))
    (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.1 20.0 0.15))
    (send *ri* :go-pos 0.0 -0.4 0)
    (send *ri* :set-gait-generator-param :stride-parameter org-stride-parameter))
  (send *ri* :go-pos 1.5 0 0)
  (let ((org-stride-parameter (send (send *ri* :get-gait-generator-param) :stride_parameter)))
    (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.1 20.0 0.15))
    (send *ri* :go-pos 0 0 135)
    (send *ri* :set-gait-generator-param :stride-parameter org-stride-parameter))
  (send *ri* :go-pos 0 -0.3 0)
  )
;; (load "experiment.l")
;; (send *ri* :start-auto-balancer :limbs '(:rleg :lleg))
;; (send *ri* :emergency-walking-stop)
;; (run-door-walk-with-two-perception* 10 10 2 10 2 10 2)
;; (run-valve-walk-with-two-perception* 20 15 2 10 2 10 2)
(defun demo-valve-only ()
  (run-valve-walk-with-two-perception* 20 15 2 10 2 10 1.5))
(defun demo-door-only ()
  (run-door-walk-with-two-perception* 10 10 2 10 2 10 2))
(defun do-demo ()
  (demo-door-only)
  (move-door-to-valve)
  (demo-valve-only)
  (move-valve-to-door)
  )
