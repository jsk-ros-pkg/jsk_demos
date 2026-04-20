;; (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_robot_startup")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "fetch_auto_dock_msgs")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* nil)

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*
(objects (list *fetch* *scene*))
(send *ri* :objects (send *scene* :objects))

(let ((robot-name (ros::get-param "/robot/name")))
  (defparameter *dock-spot*
    (cond
      ((equal robot-name "fetch15")
       "/eng2/7f/room73B2-fetch-dock-front")
      ((equal robot-name "fetch1075")
       "/eng2/7f/room73B2-fetch-dock2-front")
      (t nil))))

(defun inflation-loose ()
  ;; Increase the global inflation to generate a path
  ;; that is less sensitive to differences between the map shape and the actual object placement.
  (ros::set-dynamic-reconfigure-param
    "/move_base/global_costmap/inflater" "inflation_radius" :double 0.5)
  ;; Decrease the local inflation to generate a path
  ;; that allows the robot to pass close to the object.
  (ros::set-dynamic-reconfigure-param
    "/move_base/local_costmap/inflater" "inflation_radius" :double 0.15)
  t)

(defun view-fetch ()
  (send *irtviewer* :look-all
        (send (geo::make-bounding-box
               (flatten (send-all (send *fetch* :bodies) :vertices))) :grow 2.0)))

(defun simple-dock (&key (use-pose t))
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))
  (let ((dock-action-goal (instance fetch_auto_dock_msgs::DockActionGoal :init)))
    (when use-pose
      (let* ((timestamp (ros::time-now))
             (cret (get-spot-coords *dock-spot*))
             (frame-to-dock (car cret))
             (frame-id (cdr cret))
             (lret (send *tfl* :wait-for-transform "base_link" frame-id timestamp 5))
             (base-to-frame (send *tfl* :lookup-transform "base_link" frame-id timestamp))
             (goal-pose (ros::coords->tf-pose (send frame-to-dock :transform base-to-frame :world)))
             (pose-msg (instance geometry_msgs::PoseStamped :init)))
        (send pose-msg :header :stamp timestamp)
        (send pose-msg :header :frame_id "base_link")
        (send pose-msg :pose goal-pose)
        (send dock-action-goal :goal :dock_pose pose-msg)))
    (send *dock-action* :send-goal dock-action-goal)
    (unless (send *dock-action* :wait-for-result :timeout 60)
      (send *dock-action* :cancel-all-goals)
      (ros::ros-error "No result returned from /dock action server")
      (return-from simple-dock nil))
    (send (send *dock-action* :get-result) :docked)))


(defun dock ()
  ;; look down
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 800 0 500))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (simple-dock))

(defun undock (&key (rotate-in-place nil))
  (unless *undock-action*
    (setq *undock-action*
          (instance ros::simple-action-client :init
                    "/undock" fetch_auto_dock_msgs::UndockAction)))
  (unless (send *undock-action* :wait-for-server 5)
    (ros::ros-error "/undock action server is not started")
    (return-from undock nil))
  (send *undock-action* :send-goal
        (instance fetch_auto_dock_msgs::UndockActionGoal :init
                  :goal (instance fetch_auto_dock_msgs::UndockGoal :rotate_in_place rotate-in-place)))
  (unless (send *undock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /undock action server")
    (return-from undock nil))
  (send (send *undock-action* :get-result) :undocked))

(defun move-dock-front ()
  (setq *dock-front*
        (send (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :copy-worldcoords))
  (send *fetch* :move-to *dock-front* :world)
  (send *ri* :move-to *dock-front*))

(defun move-73b2-door-front ()
  (setq *73b2-door-front*
        (send (send *scene* :spot "door-spot") :copy-worldcoords))
  (send *fetch* :move-to *73b2-door-front* :world)
  (send *ri* :move-to *73b2-door-front*))

(defun get-spot-coords (name)
  (let ((spot-coords nil))
    (setq spot-coords (send (send *scene* :spot name) :copy-worldcoords))
    spot-coords))

(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (clear-costmap t))
  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  ;;(if clear-costmap (send *ri* :clear-costmap))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose ret))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *ri* :move-to goal-pose)))

(defun move-to-spot-front ()
  (send *ri* :go-pos-unsafe 0 0 -90))

(defun move-to-trashcan-front ()
  (inflation-loose)
  (let ((offset #f(-500 -500 0)))
    (setq success
          (go-to-spot "/eng2/7f/room73B2-microwave-front"
                      :relative-pos offset
                      :relative-rot 0))
    (send *ri* :go-pos-unsafe 0.2 0 0)
    (send *ri* :go-pos-unsafe 0 0 -90)
    (send *ri* :go-pos-unsafe 0.1 0 0)
    ))

(defun move-to-hrp2-trashcan-front ()
  (let ((offset #f(800 -1200 0)))
    (setq success
          (go-to-spot "/eng2/7f/room73B2-beside-chair"
                      :relative-pos offset
                      :relative-rot pi))))

(defun move-to-belka-front ()
  (let ((offset #f(0 -800 0)) (offset-other #f(2000 -800 0)))
    ;; (send *ri* :go-pos-unsafe -0.3 0 0)
    ;; (send *ri* :go-pos-unsafe 0 0 90)
    ;; (send *ri* :go-pos-unsafe 1.9 0 0)
    (inflation-loose)
    (setq success
          (go-to-spot "/eng2/7f/room73B2-door-front"
                      :relative-pos offset-other
                      :relative-rot 0))
    (setq success
          (go-to-spot "/eng2/7f/room73B2-sink-front0"
                      :relative-pos offset
                      :relative-rot pi))))

(defun move-to-trash-pre-front ()
  (inflation-loose)
  (setq success (go-to-spot "/eng2/7f/room73B2-door-front"
                            :relative-pos #f(2200 -200 0)
                            :relative-rot -pi/2))
  (send *ri* :go-pos-unsafe 0.1 0 0)
  )

(defun move-to-73b2-outside ()
  (inflation-loose)
  (let ((offset #f(400 -800 0)))
    (setq success
          (go-to-spot "/eng2/7f/room73A4-front"
                      :relative-pos offset
                      :relative-rot -pi/2))))


;; (setq offset #f(-450 -400 0))
;;(setq success (go-to-spot "/eng2/7f/room73B2-microwave-front" :relative-pos offset :relative-rot -pi/2))
;; memo of fetch lift-pose
;; #<cascaded-coords #X557cd2b8c030 :rarm-end-coords  6299.923 5800.117 1350.932 / -0.001 -0.0 1.571>
;; #f(20.0 -26.2704 -31.6325 -186.682 42.9422 -149.082 73.9761 87.1705 0.0 0.0)
