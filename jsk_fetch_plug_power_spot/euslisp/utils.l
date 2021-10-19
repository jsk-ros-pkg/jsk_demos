;;; fetch libraries
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_fetch_startup/euslisp/navigation-utils.l")

;;; parameters
(require "package://jsk_fetch_plug_power_spot/euslisp/parameters.l")

;;; ROS messages
(ros::load-ros-manifest "apriltag_ros")


(defun approach-to-spot (&key (n-trial 3) (offset #f(0 0 0)))
  (ros::ros-info "Approaching to spot-jsk")
  (let ((success nil))
    (dotimes (i n-trial)
      (setq success (go-to-spot "/eng2/7f/room73B2-spot-plugging-place" :undock-rotate t))
      (when success (return)))
  success))


;;; make fetch to go back to dock
(defun go-back-to-dock ()
  (ros::ros-info "Going back to dock")
  (let ((success nil))
    ;;; rotate to the dock
    (send *ri* :go-pos-unsafe 0 0 135)
    ;;; go back to dock
    (setq success (auto-dock))
    success))


;;; check if an id exists in target-list
(defun id-in-list (id target-list)
  (dolist (x target-list)
    (if (eq id x)
      (return-from id-in-list t)
      nil))
  nil)


;;; move camera to markers
(defun move-head-to-markers (&key (timeout 120))
  (let ((start-time (ros::time-now))
        (detection-results-msg nil)
        (id-list nil))
    (ros::ros-info "Moving head to markers")
    (setq start-time (ros::time-now))
    (while t
      (unix::sleep 1)
      (ros::ros-info "Retrieving tag message from \"/tag_detections\"...")
      (setq detection-results-msg (one-shot-subscribe "/tag_detections" apriltag_ros::AprilTagDetectionArray)) ;; TODO: add timeout
      (when detection-results-msg
        (ros::ros-info "Got an message")
        (setq id-list (mapcar #'(lambda (x) (elt (send x :id) 0)) (send detection-results-msg :detections)))
        (if (and (id-in-list 0 id-list)
                 (id-in-list 1 id-list)
                 (id-in-list 2 id-list))
            (progn
              (ros::ros-info "all markers are found")
              (return-from move-head-to-markers t))
            (progn
              (ros::ros-info "Keep searching markers")
              nil))
        )
      ;;; move head randomly
      (let ((tilt-angle (+ 0 (* (- 70 20) (/ (random 100) 100.0))))
            (pan-angle (+ -30 (* (- 30 -30) (/ (random 100) 100.0)))))
        (ros::ros-info "tilt-angle: ~A, pan-angle: ~A" tilt-angle pan-angle)
        (send *fetch* :head_tilt_joint :joint-angle tilt-angle)
        (send *fetch* :head_pan_joint :joint-angle pan-angle)
        )
      (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      ;;; check timeout
      (if (ros::time> (ros::time- (ros::time-now) start-time) (ros::time timeout))
          (progn
            (ros::ros-error "Time out exceeded.")
            (return-from move-head-to-markers nil))
          nil))))


(defun grasp-plug ()
  (let ((base-to-plug-origin nil)
        (base-to-pre-grasp-pose nil)
        (base-to-grasp-pose nil)
        (ret nil))
    ;; grasp a plug
    (ros::ros-info "Grasp a plug")
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    ;;; get plug frame
    (setq base-to-plug-origin (send *tfl* :lookup-transform "base_link" *plug-frame* (ros::time 0)))
    (if (eq base-to-plug-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from grasp-plug nil)) nil)
    ;;; move head to the plug marker
    (send *fetch* :head :look-at (send base-to-plug-origin :pos))
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; get plug frame again and calc target pose
    (setq base-to-plug-origin (send *tfl* :lookup-transform "base_link" *plug-frame* (ros::time 0)))
    (if (eq base-to-plug-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from grasp-plug nil)) nil)
    (setq base-to-pre-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-grasp-pose*))
    ;;; move eef to pre grasp pose and open hand
    (ros::ros-debug "Moving hand to pre grasp pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from grasp-plug nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :effort 100)
    ;;; move eef to grasp pose
    (ros::ros-debug "Moving hand to grasp pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from grasp-plug nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; grasp a plug
    (ros::ros-debug "grasping a plug")
    (send *ri* :start-grasp :effort 100)
    ;;;; move eef to pre grasp pose
    (ros::ros-debug "Moving hand to the pre grasp pose")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from grasp-plug nil)) nil)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation))
  t)


(defun recover-to-resetpose ()
  (let ()
    ;;; release hand
    (ros::ros-debug "release")
    (send *ri* :stop-grasp :effort 100)
    ;;; move to pre reset pose
    ;(ros::ros-debug "pre reset pose")
    ;(send *fetch* :angle-vector *pre-reset-pose*)
    ;(send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    ;;; reset-pose
    (ros::ros-debug "reset pose")
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    t))


(defun plug-to-holder ()
  (let ((base-to-holder-origin nil)
        (base-to-pre-insert-pose nil)
        (base-to-insert-pose nil)
        (base-to-plug-origin nil)
        (base-to-pre-grasp-pose nil)
        (ret nil))
    ;; insert plug to holder
    (ros::ros-info "Insert plug to holder")
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    ;; get holder frame
    (setq base-to-holder-origin (send *tfl* :lookup-transform "base_link" *holder-frame* (ros::time 0)))
    (if (eq base-to-holder-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-holder nil)) nil)
    ;;; move head to the holder marker
    (send *fetch* :head :look-at (send base-to-holder-origin :pos))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; get holder frame again and calc target pose
    (setq base-to-holder-origin (send *tfl* :lookup-transform "base_link" *holder-frame* (ros::time 0)))
    (if (eq base-to-holder-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-holder nil)) nil)
    (setq base-to-pre-insert-pose (send (send base-to-holder-origin :copy-worldcoords) :transform *holder-to-pre-insert-pose*))
    (setq base-to-insert-pose (send (send base-to-holder-origin :copy-worldcoords) :transform *holder-to-insert-pose*))
    ;;; move eef to pre insert pose
    (ros::ros-debug "Move eef to pre insert pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-insert-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-holder nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;;; move eef to insert pose
    (ros::ros-debug "Move eef to insert pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-insert-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-holder nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; release plug
    (ros::ros-debug "release")
    (send *ri* :stop-grasp :effort 100)
    ;;; get plug frame again and calc target pose
    (setq base-to-plug-origin (send *tfl* :lookup-transform "base_link" *plug-frame* (ros::time 0)))
    (if (eq base-to-plug-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-holder nil)) nil)
    (setq base-to-pre-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-pre-grasp-pose*))
    ;;;; move eef to pre grasp pose
    (ros::ros-debug "Moving hand to the pre grasp pose")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-holder nil)) nil)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation))
  t)


(defun plug-to-socket ()
  (let ((base-to-socket-origin nil)
        (base-to-pre-insert-pose nil)
        (base-to-insert-pose nil)
        (base-to-too-plugged-pose nil)
        (base-to-plugged-pose nil)
        (base-to-plug-origin nil)
        (base-to-pre-grasp-pose nil)
        (base-to-grasp-pose nil)
        (temp-target-coord nil)
        (ret nil))
    ;; insert plug to socket
    (ros::ros-info "Insert plug to socket")
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    ;; get socket frame
    (setq base-to-socket-origin (send *tfl* :lookup-transform "base_link" *socket-frame* (ros::time 0)))
    (if (eq base-to-socket-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-socket nil)) nil)
    ;;; move head to the socket marker
    (send *fetch* :head :look-at (send base-to-socket-origin :pos))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;;
    (setq base-to-socket-origin (send *tfl* :lookup-transform "base_link" *socket-frame* (ros::time 0)))
    (if (eq base-to-socket-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-socket nil)) nil)
    (setq base-to-pre-insert-pose (send (send base-to-socket-origin :copy-worldcoords) :transform *socket-to-pre-insert-pose*))
    (setq base-to-insert-pose (send (send base-to-socket-origin :copy-worldcoords) :transform *socket-to-insert-pose*))
    (setq base-to-too-plugged-pose (send (send base-to-socket-origin :copy-worldcoords) :transform *socket-to-too-plugged-pose*))
    (setq base-to-plugged-pose (send (send base-to-socket-origin :copy-worldcoords) :transform *socket-to-plugged-pose*))
    ;;; move eef to pre insert pose
    (ros::ros-debug "Move eef to pre insert pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-insert-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;;; move eef to insert pose
    (ros::ros-debug "Move eef to insert pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-insert-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; move eef to too plugged pose
    (ros::ros-debug "Move eef to too plugged pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-too-plugged-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; move eef to plugged pose
    (ros::ros-debug "Move eef to plugged pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-plugged-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; release plug
    (ros::ros-info "release")
    (send *ri* :stop-grasp :effort 100)
    ;;; move eef to pre insert pose
    (ros::ros-debug "Move eef to pre insert pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-insert-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    ;; regrasp
    (ros::ros-info "Regrasping")
    ;;;
    (setq base-to-plug-origin (send *tfl* :lookup-transform "base_link" *plug-frame* (ros::time 0)))
    (if (eq base-to-plug-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-holder nil)) nil)
    (setq base-to-pre-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-grasp-pose*))
    ;;; move eef to pre grasp pose
    (ros::ros-debug "Move eef to pre grasp pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;;; move eef to grasp pose
    (ros::ros-debug "Move eef to grasp pose...")
    (setq ret (send *fetch* :inverse-kinematics base-to-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; regrasp
    (ros::ros-debug "regrasp")
    (send *ri* :start-grasp :effort 100)
    ;;; update robot model
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    ;; move eef to +50mm x
    (setq ret (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :pos #f(50 0 0)))))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-socket nil)) nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; glind along r p y
    ;;;; roll
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0.05 0 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(-0.1 0 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0.05 0 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;;; yaw
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 0.05 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 -0.1 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 0.05 0))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;;; roll
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 0 0.05))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 0 -0.1))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *fetch* :inverse-kinematics (send (send (send *fetch* :rarm :end-coords) :copy-worldcoords) :transform (make-cascoords :rpy #f(0 0 0.05))))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;; release
    (ros::ros-debug "release")
    (send *ri* :stop-grasp :effort 100)
    ;;; get plug frame again and calc target pose
    (setq base-to-plug-origin (send *tfl* :lookup-transform "base_link" *plug-frame* (ros::time 0)))
    (if (eq base-to-plug-origin nil) (progn (ros::ros-error "lookup transform failed") (return-from plug-to-holder nil)) nil)
    (setq base-to-pre-grasp-pose (send (send base-to-plug-origin :copy-worldcoords) :transform *plug-to-pre-grasp-pose*))
    ;;;; move eef to pre grasp pose
    (ros::ros-debug "Moving hand to the pre grasp pose")
    (setq ret (send *fetch* :inverse-kinematics base-to-pre-grasp-pose))
    (if (not ret) (progn (ros::ros-error "IK Failed") (return-from plug-to-holder nil)) nil)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
    (send *ri* :wait-interpolation))
  t)


(defun plug-power-connector ()
  ;; reset-pose
  (ros::ros-info "moving to reset pose")
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :reset-pose) 5000)

  ;; approach to spot
  (if (approach-to-spot)
    t
    (progn
      (ros::ros-error "Failed to approach to spot")
      (return-from plug-power-connector nil)))

  ;; move head to ar marker
  (if (move-head-to-markers)
    t
    (progn
      (ros::ros-error "Failed to find markers")
      (go-back-to-dock)
      (return-from plug-power-connector nil)))

  ;; grasp plug
  (if (grasp-plug)
    t
    (progn
      (ros::ros-error "Failed to grasp plug")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from plug-power-connector nil)))

  ;; move head to ar markers again
  (if (move-head-to-markers)
    t
    (progn
      (ros::ros-error "Failed to find markers")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from plug-power-connector nil)))

  ;; insert plug to socket
  (if (plug-to-socket)
    t
    (progn
      (ros::ros-error "Failed to insert plug to socket")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from plug-power-connector nil)))

  ;;; reset pose
  (ros::ros-info "moving to reset pose")
  (recover-to-resetpose)

  ;;; go back to dock
  (go-back-to-dock))


(defun unplug-power-connector ()
  ;; reset-pose
  (ros::ros-info "moving to reset pose")
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :reset-pose) 5000)

  ;; approach to spot
  ;;(if (approach-to-spot)
  ;;  t
  ;;  (progn
  ;;    (ros::ros-error "Failed to approach to spot")
  ;;    (return-from unplug-power-connector nil)))

  ;; move head to ar marker
  (if (move-head-to-markers)
    t
    (progn
      (ros::ros-error "Failed to find markers")
      (go-back-to-dock)
      (return-from unplug-power-connector nil)))

  ;; grab plug
  (if (grasp-plug)
    t
    (progn
      (ros::ros-error "Failed to grasp a plug")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from unplug-power-connector nil)))

  ;; move head to ar markers again
  (if (move-head-to-markers)
    t
    (progn
      (ros::ros-error "Failed to find markers")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from plug-power-connector nil)))

  ;; put the plug to the holder
  (if (plug-to-holder)
    t
    (progn
      (ros::ros-error "Failed to put a plug to the holder")
      (recover-to-resetpose)
      (go-back-to-dock)
      (return-from unplug-power-connector nil)))

  ;;; reset pose
  (ros::ros-info "moving to reset pose")
  (recover-to-resetpose)

  ;; go back to the dock
  (go-back-to-dock))
