(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")


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
  (unless (and (boundp '*pr2*) (boundp '*ri*))
    (ros::ros-error "*pr2* or *ri* is not bound.")
    (return-from update-robot-position nil))
  (ros::spin-once)
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun tolerance-loose (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.20)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.175)  ;; 10deg
  t)

(defun tolerance-normal (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.15)
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
