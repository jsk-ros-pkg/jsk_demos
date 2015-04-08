(ros::roseus-add-msgs "rosgraph_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "drc_task_common")

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
     ((string= lower-name "jaxon")
      "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
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
  (ros::spin-once)
  (when (and (boundp '*cancel-motion-flag*) *cancel-motion-flag*) 
    (setq *cancel-motion-flag* nil)
    (ros::ros-info "cancel driven")
    (throw 'exit :exit-with-flug)
    )
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
  (let* ((original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    (dolist (av motion)
      (send *robot* :angle-vector av)
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (send *irtviewer* :draw-objects)
      (cond (wait? (read-line))
            (t (unix::usleep (* step-time 1000))))
      )))

;; send angle-vector from ocs
(defun send-angle-vector-from-ocs
  (av time)
  (ros::service-call "/angle_vector_command_from_ocs"
                     (instance drc_task_common::AngleVectorCommandRequest :init :angle_vector (instance std_msgs::Float32MultiArray :init :data av) :interpolation_time (instance std_msgs::Uint32 :init :data (round time))))
  )

(defun wait-interpolation-from-ocs
  ()
  (unless (boundp '*wait-interpolation-from-ocs-inited*)
    (ros::advertise-service "/angle_vector_finished" std_srvs::Empty #'angle-vector-finished-cb)
    (setq *wait-interpolation-from-ocs-inited* t))
  (setq *angle-vector-finished* nil)
  (while (not *angle-vector-finished*) 
    (ros::sleep)
    (ros::spin-once))
  )

(defun angle-vector-finished-cb
  (req)
  (setq *angle-vector-finished* t)
  (send req :response))

(defun send-angle-vector-finished-service
  ()
  (ros::service-call "/angle_vector_finished" (instance std_srvs::EmptyRequest :init))
  )

;; get potentio-vector from ocs
(defun get-potentio-vector-from-joint-states ()
  (while (not (boundp '*potentio-vector*))
    (ros::spin-once)
    (ros::sleep))
  *potentio-vector*
  )

(defun joint-states-callback (msg)
  ;; check sanity of *robot*
  (when (boundp '*robot*)
    (when (<= (length (send *robot* :joint-list)) (length (send msg :name)))
      (setq *potentio-vector* (coerce (mapcar #'rad2deg (coerce (send msg :position) cons)) float-vector))))
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

(defun move-robot-model
  (coords frame-id)
  (let* ((header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id frame-id))
         (msg (instance geometry_msgs::PoseStamped :init :header header :pose (ros::coords->tf-pose coords))))
    (ros::publish "/urdf_control_marker/set_pose" msg)
    ))

(defun move-robot-model-to-current-pos
  ()
  (let* ((base-frame "/odom_on_ground") (target-frame "/ground")
         (header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id base-frame))
         (msg (instance geometry_msgs::PoseStamped :init :header header))
         coords)
    (setq coords (send (send *tf* :lookup-transform target-frame base-frame (ros::time 0)) :inverse-transformation))
    (send msg :pose (ros::coords->tf-pose coords))
    (ros::publish "/urdf_control_marker/set_pose" msg)
    ))

(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defmethod rtm-ros-robot-interface
  (:get-initial-force-list
    (limb)
    (unless (consp limb) (setq limb (list limb)))
    (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb)
   ))
(defmethod rtm-ros-robot-interface
  (:reach-until-touch
   (limb dir
         &key (thre 10) (max-dist 150) (dist-step 10) (time-step 200)
         (coords-system :world) (ik-mode :limb) (rotation-axis t) (debug nil) (initial-force nil) (p-control nil) (try-dist max-dist))
   (unless (consp limb) (setq limb (list limb)))
   (unless (consp dir) (setq dir (list dir)))
   (let* ((tmp-robot (copy-object robot))
          (ndir (mapcar #'normalize-vector dir))
          (finish-condition (make-list (length limb) :initial-element nil))
          (dist-result 0)
          (initial-pos (mapcar #'(lambda (l) (send (send tmp-robot l :end-coords) :worldpos)) limb)))
     ;; 1. initialize
     (unless initial-force (setq initial-force (send* self :get-initial-force-list limb)))
     (send tmp-robot :angle-vector (send self :state :reference-vector))
     ;; (send tmp-robot :angle-vector (send self :state :potentio-vector))
     (send tmp-robot :fix-leg-to-coords (make-coords))
     ;;(send self :set-interpolation-mode ":linear")
     (when debug (warn ";; ndir: ~a initial-force: ~a~%" ndir initial-force))
     ;; 2. loop
     (dotimes (i (round (/ try-dist dist-step)))
      ;; 2.1 check whether end condition is satisfied
      (let* ((current-force (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb))
             (relative-force (mapcar #'v- current-force initial-force)))
        (if (not p-control)
          (let ((relative-dired-force (mapcar #'(lambda (l df nd) (- (v. df (if (equal coords-system :world) nd (send (send (send tmp-robot l :end-coords) :copy-worldcoords) :rotate-vector nd))))) limb relative-force ndir)))      
            (setq finish-condition (mapcar #'(lambda (l fc rdf) (or fc (when (> rdf thre) (warn "limb ~a touched~%" l) t)))
                                           limb finish-condition relative-dired-force))
            (when debug (warn ";; relative-dired-force: ~a thre: ~a finish-cond: ~a~%" relative-dired-force thre finish-condition)))
          
          (let*
              ((target-force-vector (mapcar #'(lambda (d) (scale -1 d)) dir)) 
               (diff-force (mapcar #'v- relative-force target-force-vector))
               (mag-of-diff-force (mapcar #'norm diff-force)))
            (setq finish-condition (mapcar #'(lambda (l fc ip) (or fc (when (> (norm (v- (send (send tmp-robot l :end-coords) :worldpos) ip)) max-dist) (warn "over max-dist~%" l) t))) limb finish-condition initial-pos))

            (setq finish-condition (mapcar #'(lambda (l fc mdf) (or fc (when (< mdf thre) (warn "limb ~a touched~%" l) t)))
                                           limb finish-condition mag-of-diff-force))
            ;(setq ndir (mapcar #'(lambda (target diff) (scale (/ 1.0 (norm target)) diff)) target-force-vector diff-force))
            (setq ndir (mapcar #'normalize-vector diff-force))
            (when debug (warn ";; mag-of-diff-force: ~a thre: ~a finish-cond: ~a ndir: ~a target ~a relative ~a~%" mag-of-diff-force thre finish-condition ndir target-force-vector relative-force))
            )
          )
        (unless (some #'not finish-condition)
          (return-from nil nil)))
      ;; 2.2 solve ik and move real robot
      (let* (ik-res)
        (cond
         ((equal ik-mode :fullbody)
          (let* ((all-limb (list :rarm :larm :rleg :lleg)))
            (with-move-target-link-list
             (mt ll tmp-robot all-limb)
             (setq tc (mapcar #'(lambda (l) (send (send tmp-robot l :end-coords) :copy-worldcoords)) all-limb))
             (dotimes (i (length limb))
               (let* ((l (elt limb i))
                      (nd (elt ndir i)))
                 (when (position l all-limb)
                   (send (elt tc (position l all-limb)) :translate (scale dist-step nd) coords-system))))
             (setq ik-res
                   (send tmp-robot
                         :fullbody-inverse-kinematics tc
                         :move-target mt :link-list ll
                         :additional-weight-list ;; for hrp2
                         (list (list (send tmp-robot :rleg :toe-p :child-link) 0)
                               (list (send tmp-robot :lleg :toe-p :child-link) 0))
                         :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
                         :target-centroid-pos (apply #'midpoint 0.5 (send tmp-robot :legs :end-coords :worldpos))
                         :translation-axis (list t t t t)
                         :rotation-axis (list t t t t)
                         :max (float-vector  500  500  0  20  20  10) ;; for hrp2
                         :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
                         :stop 100
                         :debug-view nil
                         ))))
            (unless ik-res
              (warn ";; fullbody-ik failed~%")
              (return-from nil nil)))
         ((equal ik-mode :limb)
          (setq ik-res
                (mapcar #'(lambda (l nd fc) (or fc (send tmp-robot l :move-end-pos (scale dist-step nd) coords-system))) limb ndir finish-condition))
          (dotimes (i (length ik-res))
            (unless (elt ik-res i)
              (warn ";; ik failed in ~a~%" (elt limb i))
              (setf (elt finish-condition i) t))))))
      (send self :angle-vector (send tmp-robot :angle-vector) time-step)
      (unix::usleep (round (* (* time-step 1000) 0.8)))
      (setq dist-result (+ dist-result dist-step))
      )
     ;; 3. send final posture again
     ;;(send self :angle-vector (send self :state :potentio-vector) 500)
     (send self :wait-interpolation)
     (unix::usleep (* 100 1000))
     ;;(send self :set-interpolation-mode ":hoffarbib")
     dist-result)
   ))
