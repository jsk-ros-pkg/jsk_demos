(ros::roseus-add-msgs "move_base_msgs")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; (defvar *pr2*)
;; (defvar *ri*)


(defun call-service (service req)
  (if (ros::wait-for-service service 0)
      (ros::service-call service req)
    nil))

(defun make-scene-by-tf (tfl)
  "Load building scene by existing TF frame"
  (let (frames scene)
    (unix::sleep 1) ;; wait for TF messages

    (setq frames (send tfl :get-frame-strings))
    (cond ((find "eng2" frames :test #'string=)
           (setq scene (make-eng2-scene)))
          ((find "eng8" frames :test #'string=)
           (setq scene (make-eng8-scene)))
          (t
           (ros::ros-fatal
            "There is no building frame_id in TF tree, so exiting...")
           (exit))
          )
    scene))

(defun update-scene-by-tf (scene tfl &optional (tm (ros::time 0)) (timeout 0.1))
  (let* ((objects (send scene :objects))
         (fixed-frame (send scene :name)) ;; ok?
         (updated nil)
         trans)
    (dolist (obj objects)
      (when (and (stringp (send obj :name))
                 (send tfl :wait-for-transform fixed-frame (send obj :name)
                       tm timeout))
        (setq trans
               (send tfl :lookup-transform fixed-frame (send obj :name) tm))
        (send obj :move-to trans scene)
        (push obj updated)
        ))
    updated))

(defun update-robot-position ()
  (ros::spin-once)
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun coords->movebaseactiongoal (coords)
  (let ((goal (instance move_base_msgs::movebaseactiongoal :init)))
    (send goal :goal :target_pose :pose (ros::coords->tf-pose coords))
    (send goal :goal :target_pose :header :frame_id (send coords :name))
    goal))

(defun transform-pose-to-target-frame (coords frame-id tfl
                                       &optional (tm (ros::time 0))
                                                 (timeout 1.0))
  (send tfl :wait-for-transform frame-id (send coords :name) tm timeout)
  (let ((tra (send tfl :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun check-current-floor (scene tfl &optional pose) ;; :name == frame_id
  (if pose
      (setq pose (transform-pose-to-target-frame pose (send scene :name) tfl))
    (setq pose (send tfl :lookup-transform
                     (send scene :name) "base_footprint" (ros::time 0))))
  (send scene :current-floor pose))

(defun tolerance-loose (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.20)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.175)  ;; 10deg
  t)

(defun tolerance-medium (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.10)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.087)  ;; 5deg
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding" :double 0.05)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding" :double 0.05)
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun tolerance-strict (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.10)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.087)  ;; 5deg
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding" :double 0.03)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding" :double 0.03)
  (use-tilt-laser-obstacle-cloud nil)
  t)

(defun store-params (&rest args)
  (defparameter *xy-goal-tolerance*
    (ros::get-param "/move_base_node/DWAPlannerROS/xy_goal_tolerance"))
  (defparameter *yaw-goal-tolerance*
    (ros::get-param "/move_base_node/DWAPlannerROS/yaw_goal_tolerance"))
  (defparameter *global-footprint-padding*
    (ros::get-param "/move_base_node/global_costmap/footprint_padding"))
  (defparameter *local-footprint-padding*
    (ros::get-param "/move_base_node/local_costmap/footprint_padding"))
  t)

(defun restore-params (&rest args)
  (when (boundp '*xy-goal-tolerance*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/DWAPlannerROS" "xy_goal_tolerance"
     :double *xy-goal-tolerance*))
  (when (boundp '*yaw-goal-tolerance*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance"
     :double *yaw-goal-tolerance*))
  (when (boundp '*global-footprint-padding*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/global_costmap" "footprint_padding"
     :double *global-footprint-padding*))
  (when (boundp '*local-footprint-padding*)
    (ros::set-dynamic-reconfigure-param
     "/move_base_node/local_costmap" "footprint_padding"
     :double *local-footprint-padding*))
  t)
