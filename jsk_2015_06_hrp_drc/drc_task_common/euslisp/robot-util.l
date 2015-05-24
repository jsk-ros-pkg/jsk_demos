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
  (when (or (not (boundp '*robot*)) (not *robot*))
    (setq *robot-name* (string-upcase name))
    (load (robot-file name))
    (cond (generate-ri?
           (init-robot-from-name name))
          (t
           (setq *robot* (make-robot-model-from-name name))))))

(defun model2real (&key (time 5000) (wait? t) (enable-cancel? t))
  (when enable-cancel?
    (ros::spin-once)
    (when (and (boundp '*cancel-motion-flag*) *cancel-motion-flag*) 
      (setq *cancel-motion-flag* nil)
      (ros::ros-info "cancel driven")
      (throw 'exit :exit-with-flug)
      ))
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  (when wait? (send *ri* :wait-interpolation)))

(defun hand-model2real (&key (time 500) (wait? t))
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) time)
  (when wait? (send *ri* :hand-wait-interpolation)))

(defun hand-start-grasp (&key (arm :arms) (real? nil) (time 500) (wait? t) (type :normal) (gain 30))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (cond ((equal type :door-grasp-preshape)
                (send *robot* :hand :arms :angle-vector *door-grasp-preshape-pose*)
                (when real? (hand-model2real :time time :wait? t))
                )
               ((equal type :door-grasp)
                (send *robot* :hand :arms :angle-vector *door-grasp-pose*)
                (when real? (hand-model2real :time time :wait? t))
                )
               ((equal type :default-grasp)
                (send *robot* :hand :arms :angle-vector (float-vector 90.0 0.0 0.0 0.0 90.0 90.0))
                (when real? (hand-model2real :time time :wait? t))
                )
               (t ;; (equal type :normal)
                (send *robot* :hand :arms :extension-pose) ;; hand extension-pose
                (when real? (hand-model2real :time time :wait? t))
                (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
                (when real? (hand-model2real :time time :wait? wait?))
                ))
         )
        (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (cond ((or (equal type :door-grasp-preshape))
                (when (and real? (find-method *ri* :stop-grasp))
                  (send *ri* :stop-grasp arm :wait wait?))
                )
               (t
                (when (and real? (find-method *ri* :start-grasp))
                  (send *ri* :start-grasp arm :wait wait? :gain gain))
                ))
         ))
  )

(defun hand-stop-grasp (&key (arm :arms) (real? nil) (time 500) (wait? t) (type :hook-pose))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (cond ((equal type :reset-pose)
                (send *robot* :hand arm :reset-pose)
                (when real? (hand-model2real :time time :wait? wait?))
                )
               (t ;; (equal type :hook-pose)
                (send *robot* :hand arm :hook-pose)
                (when real? (hand-model2real :time time :wait? wait?))
                ))
         )
        (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (when (and real? (find-method *ri* :stop-grasp))
           (send *ri* :stop-grasp arm :wait wait?))
         ))
  )

(defun send-angle-vector-sequence-with-time
  (av-seq &key (time 5000) (wait? t) (enable-cancel? t))
  (when enable-cancel?
    (ros::spin-once)
    (when (and (boundp '*cancel-motion-flag*) *cancel-motion-flag*) 
      (setq *cancel-motion-flag* nil)
      (ros::ros-info "cancel driven")
      (throw 'exit :exit-with-flug)
      ))
  (send *ri* :angle-vector-sequence av-seq (make-list (length av-seq) :initial-element time))
  (when wait? (send *ri* :wait-interpolation)))

(defun publish-model-joint-states (&key (av nil))
  (let* ((msg (instance sensor_msgs::JointState :init)))
    (send msg :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
    (send msg :name (send-all (send *robot* :joint-list) :name))
    (send msg :position (mapcar #'deg2rad (coerce (if av av (send *robot* :angle-vector)) cons)))
    (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg)
    (ros::spin-once))
  )

(defun publish-motion-joint-states (motion &key (step-time 400)) ;; motion is list of angle-vector
  (dolist (av motion)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    )
  )

(defun play-motion-on-eus-viewer
  (motion
   &key
   (step-time 400) (wait? nil) (interpolation? nil)
   (collision-check? nil) (collision-check-robot-link-list nil) (collision-check-obj-link nil)) ;; motion is list of angle-vector
  (unless (boundp '*irtviewer*)
    (warning-message 1 "irtviewer is not generated.~%")
    (return-from play-motion-on-eus-viewer nil))
  (let* ((original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    (dolist (av motion)
      (cond (interpolation?
             (send-angle-vector-with-interpolation
              (copy-seq (send *robot* :angle-vector)) av
              :foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
            (t
             (send *robot* :angle-vector av)))
      (when collision-check?
        (collision-check-with-obj :robot-link-list collision-check-robot-link-list :obj-link collision-check-obj-link))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (send *irtviewer* :draw-objects)
      (cond (wait? (read-line))
            ((not interpolation?) (unix::usleep (* step-time 1000))))
      (x::window-main-one)
      )))

(defun send-angle-vector-with-interpolation
  (start-av goal-av &key (devide-num 10) (foot-midcoords))
  (dotimes (i devide-num)
    (send *robot* :angle-vector
          (midpoint (/ (float i) (1- devide-num)) start-av goal-av))
    (when foot-midcoords
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    )
  )

(defun collision-check-with-obj
  (&key (robot-link-list (send *robot* :links)) (obj-link nil))
  (dotimes (i (length robot-link-list))
    (when (equal (pqp-collision-check (elt robot-link-list i) obj-link) 1)
      (ros::ros-warn "collision detected between obj and ~a" (send (elt robot-link-list i) :name)))
    ))

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

;; send reach-until-touch from ocs
(defun reach-until-touch-from-ocs
  (limb
   dir
   &key (thre 10) (max-dist 150) (dist-step 10) (time-step 200) (rotation-axis t))
  (ros::service-call "/reach_until_touch_command_from_ocs"
                     (instance drc_task_common::ReachUntilTouchCommandRequest :init
                               :arm_side (cond ((equal limb :rarm) drc_task_common::ReachUntilTouchCommandRequest::*RARM*)
                                               ((equal limb :larm) drc_task_common::ReachUntilTouchCommandRequest::*LARM*)
                                               (t drc_task_common::ReachUntilTouchCommandRequest::*RARM*))
                               :direction (ros::pos->tf-point dir)
                               :thre thre :max_dist max-dist
                               :dist_step dist-step :time_step time-step :rotation_axis rotation-axis
                               ))
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

(defun check-coords-distance ;; return t when distance is smaller than thre
  (c1 c2 &key (pos-thre 0.005) (rpy-thre (deg2rad 2)))
  (let* ((c-diff (send c1 :transformation c2))
         (pos-diff (mapcar #'abs (coerce (send c-diff :pos) cons)))
         (rpy-diff (mapcar #'abs (car (send c-diff :rpy-angle))))
         )
    (and (< (norm (coerce pos-diff float-vector)) pos-thre) (< (norm (coerce rpy-diff float-vector)) rpy-thre))
    ))
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

(defun check-angle-vector-continuous
  (start-av goal-av arm &key (xyz-thre 100) (rpy-thre (deg2rad 30)) (print? nil) (draw? nil))
  (let* ((original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (original-av (copy-seq (send *robot* :angle-vector)))
         (devide-num 2)
         start-eef-coords goal-eef-coords
         diff-between-start-goal xyzrpy-diff-between-start-goal
         (ret t)
         )
    (send *robot* :angle-vector start-av)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq start-eef-coords
          (send (send *robot* arm :end-coords) :copy-worldcoords))
    (send *robot* :angle-vector goal-av)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq goal-eef-coords
          (send (send *robot* arm :end-coords) :copy-worldcoords))
    (setq diff-between-start-goal (send start-eef-coords :transformation goal-eef-coords))
    (setq xyzrpy-diff-between-start-goal
          (mapcar #'abs (append (coerce (send diff-between-start-goal :worldpos) cons) (car (send diff-between-start-goal :rpy-angle)))))
    (dotimes (i devide-num)
      (let* (tmp-eef-coords
             diff-with-start diff-with-goal
             xyzrpy-diff-with-start xyzrpy-diff-with-goal xyzrpy-diff)
        (send *robot* :angle-vector
              (midpoint (/ (float (1+ i)) (1+ devide-num)) start-av goal-av))
        (send *robot* :fix-leg-to-coords (make-coords))
        (setq tmp-eef-coords
              (send (send *robot* arm :end-coords) :copy-worldcoords))
        (setq diff-with-start (send tmp-eef-coords :transformation start-eef-coords))
        (setq diff-with-goal (send tmp-eef-coords :transformation goal-eef-coords))
        (setq xyzrpy-diff-with-start
              (mapcar #'abs (append (coerce (send diff-with-start :worldpos) cons) (car (send diff-with-start :rpy-angle)))))
        (setq xyzrpy-diff-with-goal
              (mapcar #'abs (append (coerce (send diff-with-goal :worldpos) cons) (car (send diff-with-goal :rpy-angle)))))
        (setq xyzrpy-diff (mapcar #'max xyzrpy-diff-with-start xyzrpy-diff-with-goal))
        (when draw? (send *irtviewer* :draw-objects))
        (when print?
          (format t "[check-continuous] xyz: ~a/~a rpy: ~a/~a~%"
                  (norm (coerce (mapcar #'- (subseq xyzrpy-diff 0 3) (subseq xyzrpy-diff-between-start-goal 0 3)) float-vector)) xyz-thre
                  (norm (coerce (mapcar #'- (subseq xyzrpy-diff 3 6) (subseq xyzrpy-diff-between-start-goal 3 6)) float-vector)) rpy-thre
                  )
          )
        (when (or (> (norm (coerce (mapcar #'- (subseq xyzrpy-diff 0 3) (subseq xyzrpy-diff-between-start-goal 0 3)) float-vector)) xyz-thre)
                  (> (norm (coerce (mapcar #'- (subseq xyzrpy-diff 3 6) (subseq xyzrpy-diff-between-start-goal 3 6)) float-vector)) rpy-thre))
          (when print? (format t "[check-continuous] find uncontinuous angle-vector~%"))
          (setq ret nil)
          (return-from nil nil)
          )
        )
      )
    (send *robot* :angle-vector original-av)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ret
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

(defun convert-coords-frame
  (coords &key (base-frame "/ground") (target-frame "/odom_on_ground"))
  (setq tf-trans (send *tf* :lookup-transform target-frame base-frame (ros::time 0)))
  (setq coords (if tf-trans (send tf-trans :transform coords) (make-coords)))
  coords
  )


(defun get-robot-model-current-pos
  ()
  (let* ((base-frame "/ground") (target-frame "/robot_marker_root")
         (header (instance std_msgs::Header :init :stamp (ros::time-now) :frame_id base-frame))
         (msg (instance geometry_msgs::PoseStamped :init :header header))
         tf-trans coords)
    (setq tf-trans (send *tf* :lookup-transform target-frame base-frame (ros::time 0)))
    (setq coords (if tf-trans (send tf-trans :inverse-transformation) (make-coords)))
    coords))

(defun get-opposite-arm
  (arm)
  (cond ((equal arm :larm) :rarm)
        ((equal arm :rarm) :larm)))

(defun get-mirror-angle-vector
  ()
  (let* ((joint-name-list
          (list (cons :rarm (list :collar-y
                                  :shoulder-r :shoulder-p :shoulder-y
                                  :elbow-r :elbow-p :elbow-y
                                  :wrist-r :wrist-p :wrist-y))
                (cons :larm (list :collar-y
                                  :shoulder-r :shoulder-p :shoulder-y
                                  :elbow-r :elbow-p :elbow-y
                                  :wrist-r :wrist-p :wrist-y))
                (cons :rleg (list :crotch-r :crotch-p :crotch-y
                                  :knee-r :knee-p :knee-y
                                  :ankle-r :ankle-p :ankle-y
                                  :toe-p))
                (cons :lleg (list :crotch-r :crotch-p :crotch-y
                                  :knee-r :knee-p :knee-y
                                  :ankle-r :ankle-p :ankle-y
                                  :toe-p))
                (cons :head (list :neck-r :neck-p :neck-y))
                (cons :torso (list :waist-r :waist-p :waist-y
                                   :chest-r :chest-p :chest-y
                                   ))
                ))
         (original-av (copy-seq (send *robot* :angle-vector)))
         )
    (unless (boundp '*tmp-robot*)
      (setq *tmp-robot* (copy-object *robot*)))
    (dolist (jn joint-name-list)
      (let* ((limb (car jn))
             (jnlist (cdr jn))
             )
        (mapcar
         #'(lambda (k)
             (when (find (read-from-string (format nil "~a-~a" limb (string k)))
                         (send *robot* :methods))
               (send *tmp-robot* limb k :joint-angle (* (if (or (substringp "-r" (format nil "~a" k)) (substringp "-y" (format nil "~a" k))) -1.0 +1.0) (send *robot* (case limb (:larm :rarm) (:rarm :larm) (:lleg :rleg) (:rleg :lleg) (t limb)) k :joint-angle)))
               ))
         jnlist)

        ))
    (send *robot* :angle-vector (send *tmp-robot* :angle-vector))
    )
  )

(defun set-default-impedance-param
  (&key (arm :arms))
  (send *ri* :set-impedance-controller-param arm
        :m-p 10.0 :d-p 200.0 :k-p 400.0
        :m-r 5.0 :d-r 100.0 :k-r 200.0
        :force-gain #f(1.0 1.0 1.0) :moment-gain #f(1.0 1.0 1.0)
        :sr-gain 1.0 :avoid-gain 0.001 :reference-gain 0.01  :manipulability-limit 0.1  :controller-mode 0
        )
  )

(defun start-soft-impedance
  ()
  (set-default-impedance-param)
  (cond
   ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
    (send *ri* :start-impedance :arms :m-p 10 :d-p 200 :k-p 200 :m-r 5.0 :d-r 100 :k-r 100))
   (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
    (send *ri* :start-impedance :arms :m-p 7.5 :d-p 250 :k-p 200 :m-r 1 :d-r 15 :k-r 20)))
  )

(defun start-hard-impedance
  ()
  (set-default-impedance-param)
  (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000))
        (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)))
  )

(defun check-stand-coords
  (stand-coords &key (pos-thre 0.01) (rpy-thre 1))
  (let* ((pos (scale 0.001 (send stand-coords :worldpos)))
         (rpy (mapcar #'rad2deg (car (send stand-coords :rpy-angle))))
         )
    (or (> (abs (elt pos 0)) pos-thre) (> (abs (elt pos 1)) pos-thre) (> (abs (elt rpy 0)) rpy-thre))))

(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defmethod rtm-ros-robot-interface
  (:get-initial-force-list
   (limb)
   (unless (consp limb) (setq limb (list limb)))
   (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb)
   )
  (:reach-until-touch
   (limb dir
         &key (thre 10) (max-dist 150) (dist-step 10) (time-step 200)
         (coords-system :world) (rotation-axis t) (debug nil) (initial-force nil) (p-control nil))
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
     (dotimes (i (round (/ max-dist dist-step)))
       ;; 2.1 check whether end condition is satisfied
       (let* ((current-force (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb))
              (relative-force (mapcar #'v- current-force initial-force)))
         (cond ((not p-control)
                (let ((relative-dired-force (mapcar #'(lambda (l df nd) (- (v. df (if (equal coords-system :world) nd (send (send (send tmp-robot l :end-coords) :copy-worldcoords) :rotate-vector nd))))) limb relative-force ndir)))      
                  (setq finish-condition (mapcar #'(lambda (l fc rdf) (or fc (when (> rdf thre) (warn "limb ~a touched~%" l) t)))
                                                 limb finish-condition relative-dired-force))
                  (when debug (warn ";; relative-dired-force: ~a thre: ~a finish-cond: ~a~%" relative-dired-force thre finish-condition))))
               
               (t
                (let*
                    ((target-force (mapcar #'(lambda (d) (scale -1 d)) dir)) 
                     (diff-force  (mapcar #'(lambda (l rf tf) (v- (if (equal coords-system :world) rf (send (send (send tmp-robot l :end-coords) :copy-worldcoords) :inverse-rotate-vector rf)) tf)) limb relative-force target-force))
                     (mag-of-diff-force (mapcar #'norm diff-force)))
                  (setq finish-condition (mapcar #'(lambda (l fc ip) (or fc (when (> (norm (v- (send (send tmp-robot l :end-coords) :worldpos) ip)) max-dist) (warn "over max-dist~%" l) t))) limb finish-condition initial-pos))

                  (setq finish-condition (mapcar #'(lambda (l fc mdf) (or fc (when (< mdf thre) (warn "limb ~a touched~%" l) t)))
                                                 limb finish-condition mag-of-diff-force))
                                        ;(setq ndir (mapcar #'(lambda (target diff) (scale (/ 1.0 (norm target)) diff)) target-force diff-force))
                  (setq ndir (mapcar #'normalize-vector diff-force))
                  (when debug (warn ";; mag-of-diff-force: ~a thre: ~a finish-cond: ~a ndir: ~a target ~a relative ~a~%" mag-of-diff-force thre finish-condition ndir target-force relative-force))
                  ))
               )
         (unless (some #'not finish-condition)
           (return-from nil nil)))
       ;; 2.2 solve ik and move real robot
       (let* (ik-res)
         (setq ik-res
               (mapcar #'(lambda (l nd fc) (or fc (send tmp-robot l :move-end-pos (scale dist-step nd) coords-system))) limb ndir finish-condition))
         (dotimes (i (length ik-res))
           (unless (elt ik-res i)
             (warn ";; ik failed in ~a~%" (elt limb i))
             (setf (elt finish-condition i) t))))
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
