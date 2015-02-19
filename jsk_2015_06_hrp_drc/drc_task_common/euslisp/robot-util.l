(ros::roseus-add-msgs "rosgraph_msgs")

(defun robot-file (name)
  (let ((lower-name (string-downcase name)))
    (cond
     ((string= lower-name "pr2")
      "package://pr2eus/pr2-interface.l")
     ((string= lower-name "hrp2jsk")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
     ((string= lower-name "hrp2jsknt")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
     ((string= lower-name "hrp2jsknts")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
     ((string= lower-name "staro")
      "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
     ((string= lower-name "urataleg")
      "package://hrpsys_ros_bridge_tutorials/euslisp/urataleg-interface.l")
     ((string= lower-name "samplerobot")
      "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
     ((string= lower-name "hrp2w")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
     (t (error "uknown robot ~A" name))
     )))

(defun generate-robot
  (name
   &key (generate-ri? nil))
  ;; generate robot
  (unless (boundp '*robot*)
    (load (robot-file name))
    (cond (generate-ri?
           (init-robot-from-name name))
          (t
           (setq *robot* (make-robot-model-from-name name))))))

(defun model2real (&key (time 5000) (wait? t))
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  (when wait? (send *ri* :wait-interpolation)))

(defun hand-model2real (&key (time 500) (wait? t))
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) time)
  (when wait? (send *ri* :hand-wait-interpolation)))

(defun send-angle-vector-sequence-with-time
  (av-seq &key (time 5000) (wait? t))
  (send *ri* :angle-vector-sequence av-seq (make-list (length av-seq) :initial-element time))
  (when wait? (send *ri* :wait-interpolation)))

(defun publish-model-joint-states (&key (av nil))
  (let* ((msg (instance sensor_msgs::JointState :init)))
    (send msg :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
    (send msg :name (send-all (send *robot* :joint-list) :name))
    (send msg :position (mapcar #'deg2rad (coerce (if av av (send *robot* :angle-vector)) cons)))
    (ros::publish "/jsk_model_marker_interface/hrp2/reset_joint_states_and_root" msg)
    (ros::spin-once))
  )

(defun publish-motion-joint-states (motion &key (step-time 400)) ;; motion is list of angle-vector
  (dolist (av motion)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    )
  )

(defun play-motion-on-eus-viewer (motion &key (step-time 400) (wait? nil)) ;; motion is list of angle-vector
  (unless (boundp '*irtviewer*)
    (warning-message 1 "irtviewer is not generated.~%")
    (return-from play-motion-on-eus-viewer nil))
  (dolist (av motion)
    (send *robot* :angle-vector av)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *irtviewer* :draw-objects)
    (cond (wait? (read-line))
          (t (unix::usleep (* step-time 1000))))
    ))

(defun get-potentio-vector-from-joint-states ()
  (while (not (boundp '*potentio-vector*))
    (ros::spin-once))
  *potentio-vector*)

(defun joint-states-callback (msg)
  (setq *potentio-vector* (coerce (mapcar #'rad2deg (coerce (send msg :position) cons)) float-vector))
  )

(defun check-angle-vector-distance ;; return t when distance is smaller than thre
  (&key
   (dest-av (send *robot* :angle-vector))
   (thre 5) ;; [deg]
   (ignore-head? t))
  (let* ((original-av (send *robot* :angle-vector))
         (current-av (send *ri* :state :reference-vector)))
    (when ignore-head?
      (send *robot* :angle-vector dest-av)
      (send *robot* :head :angle-vector (fill (send *robot* :head :angle-vector) 0))
      (setq dest-av (send *robot* :angle-vector))
      (send *robot* :angle-vector current-av)
      (send *robot* :head :angle-vector (fill (send *robot* :head :angle-vector) 0))
      (setq current-av (send *robot* :angle-vector))
      )
    ;; set original angle-vector
    (send *robot* :angle-vector original-av)
    ;; compare
    (< (abs (apply #'max (coerce (v- dest-av current-av) cons))) thre)
    ))

