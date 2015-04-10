(defun request-ik-from-marker-init-for-drill
  (ros::subscribe "/grasp_pose" geometry_msgs::PoseStamped #'solve-ik-from-grasp-pose-command 1)
  (ros::subscribe "/grasp_pose_z_free" geometry_msgs::PoseStamped #'solve-ik-from-grasp-pose-command-z-free 1)
  (ros::subscribe "/push_pose" geometry_msgs::PoseStamped #'solve-ik-from-push-pose-command 1)
  (ros::subscribe "/push_pose_with_assist" geometry_msgs::PoseArray #'solve-ik-from-push-pose-with-assist-command 1)
  (ros::subscribe "/push_pose_with_assist_many_times" geometry_msgs::PoseArray #'solve-ik-from-push-pose-with-assist-command-push-many 1)
  (ros::subscribe "/move_pose" geometry_msgs::PoseStamped #'solve-ik-from-move-pose-command 1)
  (ros::subscribe "/move_by_axial_restraint_pose" geometry_msgs::PoseArray #'solve-ik-from-move-by-axial-pose-command 1)
  (ros::subscribe "/move_by_axial_restraint_not_allow_slip_pose" geometry_msgs::PoseArray #'solve-ik-from-move-by-axial-not-allow-slip-pose-command 1)
  (ros::subscribe "/grasp_dual_pose" geometry_msgs::PoseArray #'solve-ik-from-grasp-dual-pose-command 1)
  (ros::subscribe "/grasp_dual_pose_z_free" geometry_msgs::PoseArray #'solve-ik-from-grasp-dual-pose-command-z-free 1)
  (ros::subscribe "/move_dual_pose" geometry_msgs::PoseArray #'solve-ik-from-move-dual-pose-command 1)
  (ros::advertise "/grasp_pose_feedback" geometry_msgs::PoseStamped 1)
  (ros::advertise "/grasp_not_allow_slip_pose_feedback" geometry_msgs::PoseStamped 1)
  (ros::advertise "/grasp_pose_dual_feedback" geometry_msgs::PoseArray 1)
  (ros::subscribe "/rhsensor" geometry_msgs::WrenchStamped #'get-rf-sensor 1)
  (ros::subscribe "/lhsensor" geometry_msgs::WrenchStamped #'get-lf-sensor 1)
  (ros::advertise-service "/get_ik_arm" drc_task_common::GetIKArm #'get-ik-arm)
  (ros::advertise-service "/get_ik_arm_pose" drc_task_common::GetIKArmPose #'get-ik-arm-pose)

  (setq *r-force-x* 0)
  (setq *l-force-x* 0)
  (setq *draw-viewer-flag* (ros::get-param "~draw_viewer" t))
  (setq *solve-ik-from-pose-command-flag* nil)
  (setq *solve-ik-from-dual-pose-command-flag* nil)
  (setq *target-pose-coords* (make-coords))
  (setq *initial-pose-coords* (make-coords))
  (setq *move-target-from-hand* (make-coords))
  (setq *target-pose-coords-for-second-arm* (make-coords))
  (setq *initial-pose-coords-for-second-arm* (make-coords))
  (setq *move-target-from-second-hand* (make-coords))
  (setq *push-many-times-flag* nil)

  ;; hand coords that depends robots
  (setq *move-target-from-hand-for-grasp* (make-hash-table))
  ;; (setf (gethash :rarm *move-target-from-hand-for-grasp*) (send (send (make-coords) :rotate (* (/ 0.0 3) pi) :z :local) :translate (float-vector -20 -45 0) :local))
  ;; (setf (gethash :larm *move-target-from-hand-for-grasp*) (send (send (make-coords) :rotate (* (/ -0.0 3) pi) :z :local) :translate (float-vector -20 45 0) :local))
  (setf (gethash :rarm *move-target-from-hand-for-grasp*) (send (send (make-coords) :rotate (* (/ 0.0 3) pi) :z :local) :translate (float-vector 0 30 0) :local))
  (setf (gethash :larm *move-target-from-hand-for-grasp*) (send (send (make-coords) :rotate (* (/ -0.0 3) pi) :z :local) :translate (float-vector 0 -30 0) :local))

  (setq *move-target-from-hand-for-push* (make-hash-table))
  ;; (setf (gethash :rarm *move-target-from-hand-for-push*) (send (send (make-coords) :translate (float-vector 80 -20 -20) :local) :rotate (* (/ -0.0 2) pi) :x :local))
  ;; (setf (gethash :larm *move-target-from-hand-for-push*) (send (send (make-coords) :translate (float-vector 80 20 -20) :local) :rotate (* (/ 0.0 2) pi) :x :local))
  (setf (gethash :rarm *move-target-from-hand-for-push*) (send (send (make-coords) :translate (float-vector 110 0 -20) :local) :rotate (* (/ -0.0 2) pi) :x :local))
  (setf (gethash :larm *move-target-from-hand-for-push*) (send (send (make-coords) :translate (float-vector 110 0 -20) :local) :rotate (* (/ 0.0 2) pi) :x :local))
  ;; maybe for single arm only, 0 is desireble
  (setq *ik-offset* (make-hash-table))
  (setf (gethash :rarm *ik-offset*) (send (send (make-coords) :translate (float-vector -20 60 0) :local) :rotate (* (/ -0.0 2) pi) :x :local))
  (setf (gethash :larm *ik-offset*) (send (send (make-coords) :translate (float-vector 40 -110 0) :world) :rotate (* (/ 0.0 2) pi) :x :local))
  (setq *dual-arm-ik-offset* (make-hash-table))
  (setf (gethash :rarm *dual-arm-ik-offset*) (send (send (make-coords) :translate (float-vector 0 0 0) :local) :rotate (* (/ -0.0 2) pi) :x :local))
  (setf (gethash :larm *dual-arm-ik-offset*) (send (send (make-coords) :translate (float-vector 0 0 0) :local) :rotate (* (/ 0.0 2) pi) :x :local))
  (setq *pre-push-angle-vector* (float-vector 65.0 0.0 30.0 80.0 90.0 -30.0))
  ;;(setf (gethash :rarm *move-target-from-hand-for-push*) (send (send (make-coords) :rotate (* (/ -0.0 3) pi) :z :local) :translate (float-vector 90 0 -15) :local))
  ;;(setf (gethash :larm *move-target-from-hand-for-push*) (send (send (make-coords) :rotate (* (/ -0.0 3) pi) :z :local) :translate (float-vector 90 0 -15) :local))
  (setq *rotation-axis-for-push* nil)

  (setq *rotation-axis* (list t t t))
  (setq *rotation-axis-default* (list :z t t))
  (setq *pub-grasp-pose-feedback-flag* nil)
  (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* nil)
  (setq *pub-grasp-dual-pose-feedback-flag* nil)
  ;; todo
  (setq *ik-pose-list* nil)
  ;;executive map
  (setq *map-step-dist* (float-vector 200 200 100))
  (setq *map-max-point* (list 400 800 10))
  (setq *map-min-point* (list -300 -600 0))
  (setq *reachability-map-mode* nil)
  (setq *last-exec-angle-vector* (send *ri* :state :potentio-vector))
  (when *draw-viewer-flag* (objects (list *obj* *robot*)))
  )

(defun pub-grasp-pose-feedback
  ()
  (ros::ros-info "pub-grasp-pose-feedback")
  (if *ik-result*
      (progn
        (let 
            ((original-av (send *robot* :angle-vector)) hand-temp-coords)
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* (- (length *ik-result*) 1)))
          (setq hand-temp-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
          
          (send hand-temp-coords :transform (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords))
          (setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform (send hand-temp-coords :copy-worldcoords)))
          (send hand-temp-coords :transform (send (gethash *ik-arm* *ik-offset*) :inverse-transformation) :world)
          (ros::publish "/grasp_pose_feedback" (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose hand-temp-coords) :header (instance std_msgs::Header :init :frame_id *frame-id*
                                                                                                                                                          :stamp (ros::time-now))))
          (ros::ros-info "grasp feedback pose pubed")
          (send *robot* :angle-vector original-av)
          )
        )
    )
  ;(setq *pub-grasp-pose-feedback-flag* (not *pub-grasp-pose-feedback-flag*))
  )
(defun pub-grasp-not-allow-slip-pose-feedback
  ()
  (ros::ros-info "pub-grasp-pose-feedback")
  (if *ik-result*
      (progn
        (let 
            ((original-av (send *robot* :angle-vector)) hand-temp-coords)
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* (- (length *ik-result*) 1)))
          (setq hand-temp-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
          (send hand-temp-coords :transform (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords))
          (send hand-temp-coords :transform (send (gethash *ik-arm* *ik-offset*) :inverse-transformation) :world)
          (setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform (send hand-temp-coords :copy-worldcoords)))
          (ros::publish "/grasp_not_allow_slip_pose_feedback" (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose hand-temp-coords) :header (instance std_msgs::Header :init :frame_id *frame-id*
                                                                                                                                                          :stamp (ros::time-now))))
          (ros::ros-info "grasp not allow slip feedback pose pubed")
          (send *robot* :angle-vector original-av)
          )
        )
    )
  ;(setq *pub-grasp-pose-feedback-flag* (not *pub-grasp-pose-feedback-flag*))
  )
(defun pub-grasp-dual-pose-feedback
  ()
  (ros::ros-info "pub-grasp-pose-feedback")
  (if *ik-result*
      (progn
        (let 
            ((original-av (send *robot* :angle-vector)) hand-temp-coords second-hand-temp-coords)
          (mapcar #'(lambda (j a) (send j :joint-angle a)) (send *robot* :joint-list) (elt *ik-result* (- (length *ik-result*) 1)))
          (setq hand-temp-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
          (setq second-hand-temp-coords (send (send *robot* (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) :end-coords :worldcoords) :copy-worldcoords))
          ;; (if (equal *ik-arm* :rarm)
          ;;     (progn (send hand-temp-coords :rotate (* (/ 0.0 3) pi) :z :local) (send hand-temp-coords :translate (float-vector -30 -50 0) :local)(send second-hand-temp-coords :rotate (* (/ -0.0 3) pi) :z :local)(send second-hand-temp-coords :translate (float-vector -30 50 0) :local))
          ;;   (progn (send hand-temp-coords :rotate (* (/ -0.0 3) pi) :z :local)  (send hand-temp-coords :translate (float-vector -30 50 0) :local)(send second-hand-temp-coords :rotate (* (/ 0.0 3) pi) :z :local) (send second-hand-temp-coords :translate (float-vector -30 -50 0) :local))
          ;;   
          (send hand-temp-coords :transform (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords))
          (send second-hand-temp-coords :transform (send (gethash (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) *move-target-from-hand-for-grasp*) :copy-worldcoords))   
          ;(send hand-temp-coords :transform *robot-coords*)
          (setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform hand-temp-coords))
          (setq second-hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform second-hand-temp-coords))
          (ros::publish "/grasp_pose_dual_feedback" (instance geometry_msgs::PoseArray :init :poses (list (ros::coords->tf-pose hand-temp-coords) (ros::coords->tf-pose second-hand-temp-coords)) :header (instance std_msgs::Header :init :frame_id *frame-id*
                                                                                                                                                                                                                   :stamp (ros::time-now))))
          (ros::ros-info "grasp dual_feedback pose pubed")
          (send *robot* :angle-vector original-av)
          )
        )
  ;(setq *pub-grasp-pose-feedback-flag* (not *pub-grasp-pose-feedback-flag*))
    )
  )
(defun get-rf-sensor
  (msg)
  (setq *r-force-x* (send msg :wrence :force :x))
  )
(defun get-lf-sensor
  (msg)
  (setq *l-force-x* (send msg :wrence :force :x))
  )

(defun solve-ik-from-grasp-pose-command 
  (msg)
  (solve-ik-from-pose-common msg)
  (if (equal *ik-arm* :rarm)
      (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -85 -130 0) :local))
    (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -85 130 0) :local))
    )
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list t t t))
  (setq *pub-grasp-pose-feedback-flag* nil)
  (setq *solve-ik-command-flag* t)
  )
(defun solve-ik-from-grasp-pose-command-z-free
  (msg)
  (solve-ik-from-grasp-pose-command msg)
  (setq *pub-grasp-pose-feedback-flag* t)
  (setq *rotation-axis* (list :z t t))
  )
(defun solve-ik-from-move-pose-command 
  (msg)
  (solve-ik-from-pose-common msg)
  (get-current-pose-common)
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list t t t))
  )

(defun solve-ik-from-push-pose-command 
  (msg)
  (solve-ik-from-pose-common msg)
  ;;(send *target-pose-coords* :translate (float-vector -10 0 0) :local) ;; length of finger
  (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -110 0 0) :local))
  (setq *move-target-from-hand* (send (gethash *ik-arm* *move-target-from-hand-for-push*) :copy-worldcoords))
  (send *robot* :hand *ik-arm* :hook-pose)
  (send *robot* :hand (if (equal *ik-arm* :rarm) :larm :rarm) :grasp-pose)
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)))
  (send *ri* :hand-wait-interpolation)
  (send *robot* :hand *ik-arm* :angle-vector *pre-push-angle-vector*)
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)))
  (setq *solve-ik-from-pose-command-flag* t)
  (setq *rotation-axis* (list *rotation-axis-for-push* t t));;
  )
(defun solve-ik-from-push-pose-with-assist-command-push-many
  (msg)
  (solve-ik-from-push-pose-with-assist-command msg)
  (setq *push-many-times-flag* t)
  )
(defun solve-ik-from-push-pose-with-assist-command 
  (msg)
  (let (pose-msg-tmp)
    (setq pose-msg-tmp (instance geometry_msgs::PoseStamped :init :pose (elt (send msg :poses) 0) :header (send msg :header)))
    (solve-ik-from-pose-common pose-msg-tmp)
    (setq *target-pose-coords-for-second-arm* (send *target-pose-coords* :copy-worldcoords))
    ;;(send *target-pose-coords-for-second-arm* :translate (float-vector -10 0 0) :local) ;; length of finger
    (setq *move-target-from-second-hand* (send (gethash (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) *move-target-from-hand-for-push*) :copy-worldcoords))
    (setq *initial-pose-coords-for-second-arm* (send (send *target-pose-coords-for-second-arm* :copy-worldcoords) :translate (float-vector -150 0 0) :local))
    (send pose-msg-tmp :pose (elt (send msg :poses) 1))
    (solve-ik-from-pose-common pose-msg-tmp)
    (setq *initial-pose-coords* (send *target-pose-coords* :copy-worldcoords))
     
    (setq *rotation-axis* (list t *rotation-axis-for-push* t t))
    (setq *solve-ik-from-dual-pose-command-flag* t)
    ;; push pose
    (when (not (equal (elt *ik-mode-list* *ik-mode-index*) :release))
      (send *robot* :hand (if (equal *ik-arm* :rarm) :larm :rarm) :hook-pose)
      (send *robot* :hand *ik-arm* :grasp-pose)
      (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)))
      (send *ri* :hand-wait-interpolation)
      (send *robot* :hand (if (equal *ik-arm* :rarm) :larm :rarm) :angle-vector *pre-push-angle-vector*)
      (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)))
      )

    )
  )
(defun solve-ik-from-move-by-axial-pose-command
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-move-by-axial-pose-command nil))
    (let* ((transform (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts)) hand-temp-coords)
      (setq *target-pose-coords*
            (send (send transform :copy-worldcoords) :transform (ros::tf-pose->coords (elt (send msg :poses) 1))))
      (setq hand-temp-coords
            (send (send transform :copy-worldcoords) :transform (ros::tf-pose->coords (elt (send msg :poses) 0))))
	
      ;; (if (equal *ik-arm* :rarm)
      ;;     (progn (send hand-temp-coords :translate (float-vector 30 50 0)
      ;;                  :local) (send hand-temp-coords :rotate (* (/ -0.0 3) pi) :z :local))
      ;;   (progn (send hand-temp-coords :translate (float-vector 30 -50 0) :local) (send hand-temp-coords :rotate (* (/ 0.0 3) pi) :z :local))
      ;;   )
      (send hand-temp-coords :transform (send (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords) :inverse-transformation))
      (setq *move-target-from-hand* (send (send (send *target-pose-coords* :copy-worldcoords) :transformation hand-temp-coords :local) :transformation (make-coords) :local)) 
      (setq *solve-ik-from-pose-command-flag* t)
      ;;(get-current-pose-common)
      (let ((original-av (send *robot* :angle-vector)))
        (send *robot* :angle-vector *last-exec-angle-vector*)
        
        (setq *initial-pose-coords* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords)))
        )
      (send *initial-pose-coords* :transform *move-target-from-hand*)
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *rotation-axis* (list :x t t))
      ))
  )
(defun solve-ik-from-move-by-axial-not-allow-slip-pose-command
  (msg)
  (solve-ik-from-move-by-axial-pose-command msg)
  (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* t)
  )
(defun get-current-pose-common
  ()
  (let ((original-av (send *robot* :angle-vector)))
    (send *robot* :angle-vector *last-exec-angle-vector*)
    ;(setq hand-temp-coords (send (send *robot-coords* :copy-worldcoords) :transform (send hand-temp-coords :copy-worldcoords)))
    ;(setq *initial-pose-coords* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords)))
    (setq *initial-pose-coords* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords)))
    (setq *initial-pose-coords-for-second-arm* (send (send *robot-coords* :copy-worldcoords) :transform (send (send *robot* (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) :end-coords :worldcoords) :copy-worldcoords)))
    (send *initial-pose-coords* :transform (send (send *move-target-from-hand* :copy-worldcoords) :inverse-transformation))
    (send *initial-pose-coords-for-second-arm* :transform (send (send *move-target-from-second-hand* :copy-worldcoords) :inverse-transformation))
    (send *robot* :angle-vector original-av)
    )
  )
(defun solve-ik-from-pose-common
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-pose-common nil))
    (setq *target-pose-coords*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords(send msg :pose)
                                                                                                                          )
                )
          )

    (setq *move-target-from-hand* (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords)) ;;for grasp  
    )
  )
(defun solve-ik-from-grasp-dual-pose-command
  (msg)
  (solve-ik-from-dual-pose-common msg)
  (if (equal *ik-arm* :rarm)
      (progn 
        (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -85 -130 0) :local))
        (setq *initial-pose-coords-for-second-arm* (send (send *target-pose-coords-for-second-arm* :copy-worldcoords) :translate (float-vector -85 130 0) :local))
        )
    (progn
      (setq *initial-pose-coords* (send (send *target-pose-coords* :copy-worldcoords) :translate (float-vector -85 130 0) :local))
      (setq *initial-pose-coords-for-second-arm* (send (send *target-pose-coords-for-second-arm* :copy-worldcoords) :translate (float-vector -85 -130 0) :local))
      )
    )
  (setq *rotation-axis* (list t t t t))
  (setq *solve-ik-from-dual-pose-command-flag* t)
  )
(defun solve-ik-from-grasp-dual-pose-command-z-free
  (msg)
  (ros::ros-info "dual z:free driven")
  (solve-ik-from-grasp-dual-pose-command msg)
  (setq *rotation-axis* (list :z :z t t))
  (setq *pub-grasp-dual-pose-feedback-flag* t)
  )
(defun solve-ik-from-move-dual-pose-command
  (msg)
  (solve-ik-from-dual-pose-common msg)
  (get-current-pose-common)
  (setq *rotation-axis* (list t t t t))
  (setq *solve-ik-from-dual-pose-command-flag* t)
  )
(defun solve-ik-from-dual-pose-common
  (msg)
  (let* ((ts (ros::time-now))
	 )
    (unless (send *tfl* :wait-for-transform *frame-id* (send msg :header :frame_id) ts 1) (return-from solve-ik-from-dual-pose-common nil))
    (setq *target-pose-coords*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords (elt (send msg :poses) 0)))
          )
    (setq *target-pose-coords-for-second-arm*
          (send (send *tfl* :lookup-transform *frame-id* (send msg :header :frame_id) ts) :transform (ros::tf-pose->coords (elt (send msg :poses) 1)))
          )
    (setq *move-target-from-hand* (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords))
    (setq *move-target-from-second-hand* (send (gethash (find-if-not #'(lambda (x) (equal x *ik-arm*)) (list :rarm :larm)) *move-target-from-hand-for-grasp*) :copy-worldcoords))
    )
  )

(defun get-ik-arm
  (req)
  (let* ((res (send req :response)))
    (send res :ik_arm (if (equal *ik-arm* :rarm) ":rarm" ":larm"))
    res))
(defun get-ik-arm-pose
  (req)
  (let* ((res (send req :response)) (original-av (send *robot* :angle-vector)) ik-arm-coords)
    ;get current pose
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (setq ik-arm-coords (send (send *robot* *ik-arm* :end-coords :worldcoords) :copy-worldcoords))
    (send ik-arm-coords :transform (send (gethash *ik-arm* *move-target-from-hand-for-grasp*) :copy-worldcoords))

    (send res :pose_stamped (instance geometry_msgs::PoseStamped :init :pose (ros::coords->tf-pose (send (send (send (send *robot* :body_lk) :worldcoords) :copy-worldcoords) :transformation ik-arm-coords :local)) :header (instance std_msgs::Header :init :frame_id "BODY"
                                                                                                               :stamp (ros::time-now))))

    (send *robot* :angle-vector original-av)
    res))

(defun make-xy-coords (x y)
  (send (send *robot-coords* :copy-worldcoords) :translate (float-vector x y 0) :local)
  ;(make-coords :pos (float-vector x y 0))
  )
(defun inverse-reachability-map-init ()
  (let* (map-list-x map-list-y)
      (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) (elt *map-step-dist* 0))))
	(dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) (elt *map-step-dist* 1))))
	  (push (+ (* (* 1.0 ix) (elt *map-step-dist* 0)) (elt *map-min-point* 0))  map-list-x)
	  (push (+ (* (* 1.0 iy) (elt *map-step-dist* 1)) (elt *map-min-point* 1))  map-list-y)
n	  ))
      (setq *inverse-reachability-coords-list* (mapcar #'make-xy-coords map-list-x map-list-y))
      )
  )
(defun show-inverse-reachability-map ()
  (let* ((msg (instance visualization_msgs::marker :init))
         (pose-msg (instance geometry_msgs::pose :init))
         points-list-msg colors-list-msg
         )
    (send msg :header (instance std_msgs::header :init
                                :stamp (ros::time-now) :frame_id *frame-id*))
    (send msg :lifetime (ros::time 0.0))
    (send msg :frame_locked t)
    (send msg :type 6) ;; cube_list
    (send msg :action 0)
    (send pose-msg :orientation :w 1.0)
    (send msg :pose pose-msg)
    (send msg :scale (instance geometry_msgs::vector3 :init
                               :x (* 0.5 (* 0.001 (elt *map-step-dist* 0)))
                               :y (* 0.5 (* 0.001 (elt *map-step-dist* 1)))
                               :z (* 0.5 (* 0.001 (elt *map-step-dist* 2)))
))
    (dotimes (i (length *inverse-reachability-coords-list*))
      (push (instance geometry_msgs::point :init
                      :x (* 0.001 (elt (send (elt *inverse-reachability-coords-list* i) :pos) 0))
                      :y (* 0.001 (elt (send (elt *inverse-reachability-coords-list* i) :pos) 1))
                      :z (* 0.001 (elt (send (elt *inverse-reachability-coords-list* i) :pos) 2)))
            points-list-msg)
      (push (vector->rgba (float-vector 1 0 1)) colors-list-msg)
      )
    (send msg :points points-list-msg)
    (send msg :colors colors-list-msg)
    (ros::publish "/inverse_reachability_visible_map" msg)
   )
  )

(defun request-ik-from-pose-one
  (&key (initial-pose :reset-manip-pose) (robot-coords (send *robot-coords* :copy-worldcoords)) (target-pose-coords (send *target-pose-coords* :copy-worldcoords)) (initial-pose-coords (send *initial-pose-coords* :copy-worldcoords)) target-pose-coords-for-second-arm initial-pose-coords-for-second-arm move-target-from-second-hand)
  (let* (original-av original-coords)
    (warning-message 2 "[request-ik] send solve-ik service.~%")
    (setq original-av (send *robot* :angle-vector))
    (setq original-coords (send (send *robot* :worldcoords) :copy-worldcoords))
    ;; regenerate move coords
    (when initial-pose (send *robot* initial-pose))
    (send *robot* :fix-leg-to-coords (make-coords))
    ;; check mode 
    ;; solve ik
    (setq target-pose-coords (send (send target-pose-coords :transformation robot-coords :local) :transformation (make-coords) :local))
    (setq initial-pose-coords (send (send initial-pose-coords :transformation robot-coords :local) :transformation (make-coords) :local))
    (if target-pose-coords-for-second-arm (setq target-pose-coords-for-second-arm (send (send target-pose-coords-for-second-arm :transformation robot-coords :local) :transformation (make-coords) :local)))
    (if initial-pose-coords-for-second-arm (setq initial-pose-coords-for-second-arm (send (send initial-pose-coords-for-second-arm :transformation robot-coords :local) :transformation (make-coords) :local)))

    (when (not target-pose-coords-for-second-arm)
      (send target-pose-coords :transform (gethash *ik-arm* *ik-offset*) :world))
    (when target-pose-coords-for-second-arm
      (send target-pose-coords :transform (gethash *ik-arm* *dual-arm-ik-offset*) :world)
      (send target-pose-coords-for-second-arm :transform (gethash (if (equal *ik-arm* :rarm) :larm :rarm) *dual-arm-ik-offset*) :world))
    
    (when (equal (elt *ik-mode-list* *ik-mode-index*) :release)
      (let (temp-coords)
        (setq temp-coords target-pose-coords)
        (setq target-pose-coords initial-pose-coords)
        (setq initial-pose-coords temp-coords)
        (setq temp-coords target-pose-coords-for-second-arm)
        (setq target-pose-coords-for-second-arm initial-pose-coords-for-second-arm)
        (setq initial-pose-coords-for-second-arm temp-coords)
        ))
    (when *draw-viewer-flag* (objects (list *robot* target-pose-coords initial-pose-coords)))
    ;; if push some
    (cond (*push-many-times-flag*
	   (setq *ik-result* nil)
	   (dotimes (ix 3)
	     (dotimes (iy 1) 
	       (dotimes (iz 2)
                 (let (ik-result-tmp)
                 (setq ik-result-tmp (send-ik-request-from-pose :target-coords target-pose-coords :rotation-axis *rotation-axis* :move-target-from-hand *move-target-from-hand* :target-coords-for-second-arm (send (send target-pose-coords-for-second-arm :copy-worldcoords) :translate (float-vector (- (* iz 110) 110) (* (- ix 1) 10) (* (- iy 0) 20)) :local) :move-target-from-second-hand move-target-from-second-hand))
                 (if ik-result-tmp
                     (setq *ik-result* (append *ik-result* ik-result-tmp))
                   )
		 )
	       )
	     )
             )
	   ;;(setq *do-many-times-with-force-flag*)
	   )
	  (t
	   (setq *ik-result* (send-ik-request-from-pose :target-coords target-pose-coords :initial-coords initial-pose-coords :rotation-axis *rotation-axis* :interpolate-mode :from-pose-one :move-target-from-hand *move-target-from-hand*
					      :target-coords-for-second-arm target-pose-coords-for-second-arm :initial-coords-for-second-arm initial-pose-coords-for-second-arm :move-target-from-second-hand move-target-from-second-hand))
	   )
	  )
    ;; end push some
    

    (if *pub-grasp-pose-feedback-flag* (pub-grasp-pose-feedback))
    (if *pub-grasp-not-allow-slip-pose-feedback-pose-flag* (pub-grasp-not-allow-slip-pose-feedback))
    (if *pub-grasp-dual-pose-feedback-flag* (pub-grasp-dual-pose-feedback))
    (cond (*ik-result*
           (send-joint-states-to-marker))
          (t
           (send *robot* :angle-vector original-av)
           (send *robot* :newcoords original-coords)
           )
          )
    (setq *solve-ik-from-pose-command-flag* nil)
    (setq *solve-ik-from-dual-pose-command-flag* nil)
    *ik-result*
    ))


(defun send-ik-request-from-pose
  (&key 
   (interpolate-mode)
   (target-coords (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
   (initial-coords target-coords)
   (arm *ik-arm*)
   (move-target-from-hand (make-coords))
   (rotation-axis (list :z t t))
   (target-coords-for-second-arm nil)
   (initial-coords-for-second-arm target-coords-for-second-arm)
   (move-target-from-second-hand (make-coords)))
   
  (cond ((equal interpolate-mode :save)
         (setq initial-coords (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *saved-obj* :descendants)))
         )
        ((equal interpolate-mode :approach)
         (setq initial-coords (find-if #'(lambda (x) (equal (send x :name) :approach)) (send *obj* :descendants)))
         (cond ((equal (elt *ik-mode-list* *ik-mode-index*) :reach)
                )
               ((equal (elt *ik-mode-list* *ik-mode-index*) :release)
                (let (tmp)
                  (setq tmp (send initial-coords :copy-worldcoords))
                  (setq initial-coords target-coords)
                  (setq target-coords tmp))
                ))))
  (setq *push-many-times-flag* nil) ;; temporary
  (let* (ik-res ik-angle-list (divide-num (if (and (not *reachability-map-mode*) interpolate-mode) 5 1)) (ik-server-service-name (if *reachability-map-mode* "/calc_dynamic_reachability" "/solve_ik")))
    ;; service call for task state manager
    (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "SolveIK"))
    ;; loop for interpolation
    (dotimes (i divide-num)
      (if (not target-coords-for-second-arm)
          (let* ((arm-index (if (equal arm :rarm) 0 1))
                 (mt (append (list (elt *end-coords-from-hand* arm-index)) *end-coords-without-toe*))
                 (tc (send-all mt :copy-worldcoords))
                 (ll (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) mt))
                 (translation-axis (list t t t))
                 (end-coords-2 (make-cascoords :name :end-coords-2 :coords (send (elt *end-coords-from-hand* arm-index) :copy-worldcoords)))
                 res)

            (setf (elt tc 0) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords target-coords) :copy-worldcoords))
            (send end-coords-2 :transform move-target-from-hand :local)
            (send (send (elt *end-coords-from-hand* arm-index) :parent) :assoc end-coords-2)
            (setf (elt mt 0) end-coords-2)	      
            (cond ((equal arm :rarm)
                   (setq *assoc-frame-id* "jsk_model_marker_interface/robot/RARM_LINK6"))
              ((equal arm :larm)
               (setq *assoc-frame-id* "jsk_model_marker_interface/robot/LARM_LINK6")))
            ;; service call for ik server
            (cond ((or (string= *robot-name* "HRP2JSKNTS") (string= *robot-name* "HRP2JSKNT"))
                   (setq res (ik-server-call
                              :ik-server-service-name ik-server-service-name
                              :target-coords tc
                              :move-target mt
                              :link-list ll
                              :debug-view :no-message
                              :translation-axis translation-axis
                              :rotation-axis rotation-axis
                              :max (float-vector  500  500  0  20  20  10)
                              :warnp t
                              :stop (if (or *reachability-map-mode* *inverse-reachability-map-mode*) 15 30)
                              )))
                  ((string= *robot-name* "staro")
                   (setq res (ik-server-call
                              :ik-server-service-name ik-server-service-name
                              :target-coords tc
                              :move-target mt
                              :link-list ll
                              :debug-view :no-message
                              :translation-axis translation-axis
                              :rotation-axis rotation-axis
                              ;; temporary for staro
                              :additional-weight-list
                              (list (list (send *robot* :torso :waist-y :child-link) 0)
                                    (list (send *robot* :torso :waist-p :child-link) 0))
                              :warnp t
                              :stop (if (or *reachability-map-mode* *inverse-reachability-map-mode*) 15 30)
                              ))))
            (setq ik-res (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*))
            (send (send (elt *end-coords-from-hand* arm-index) :parent) :dissoc end-coords-2)
            (unless ik-res (return))
            (push (send-all (send *robot* :joint-list) :joint-angle) ik-angle-list)
            )
        (let* ((arm-index (if (equal arm :rarm) 0 1))
               (second-arm-index (if (equal arm :rarm) 1 0))
               (mt (append (list (elt *end-coords-from-hand* arm-index) (elt *end-coords-from-hand* second-arm-index)) *end-coords-without-toe*))
               (tc (send-all mt :copy-worldcoords))
               (ll (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) mt))
               (translation-axis (list t t t t))
               (end-coords-2 (make-cascoords :name :end-coords-2 :coords (send (elt *end-coords-from-hand* arm-index) :copy-worldcoords)))
               (end-coords-3 (make-cascoords :name :end-coords-3 :coords (send (elt *end-coords-from-hand* second-arm-index) :copy-worldcoords)))
               res)
          
          (setf (elt tc 0) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords target-coords) :copy-worldcoords))
          (setf (elt tc 1) (send (midcoords (/ (float (+ i 1)) divide-num) initial-coords-for-second-arm target-coords-for-second-arm) :copy-worldcoords))
          (send end-coords-2 :transform move-target-from-hand :local)
          (send (send (elt *end-coords-from-hand* arm-index) :parent) :assoc end-coords-2)
          (setf (elt mt 0) end-coords-2)
          (send (send (elt *end-coords-from-hand* second-arm-index) :parent) :assoc end-coords-3)
          (setf (elt mt 1) end-coords-3)
          (send end-coords-3 :transform move-target-from-second-hand :local)
          (cond ((equal arm :rarm)
                 (setq *assoc-frame-id* "jsk_model_marker_interface/robot/RARM_LINK6"))
                ((equal arm :larm)
                 (setq *assoc-frame-id* "jsk_model_marker_interface/robot/LARM_LINK6")))
          ;; service call for ik server
          (setq res (ik-server-call
                     :ik-server-service-name ik-server-service-name
                     :target-coords tc
                     :move-target mt
                     :link-list ll
                     :debug-view :no-message
                     :translation-axis translation-axis
                     :rotation-axis rotation-axis
                     :max (float-vector  500  500  0  20  20  10)
                     :warnp t
                     :stop (if (or *reachability-map-mode* *inverse-reachability-map-mode*) 30 60)
                     ))
          
          (setq ik-res (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*))
          (send (send (elt *end-coords-from-hand* arm-index) :parent) :dissoc end-coords-2)
          (send (send (elt *end-coords-from-hand* second-arm-index) :parent) :dissoc end-coords-3)
          (unless ik-res (return))
          (push (send-all (send *robot* :joint-list) :joint-angle) ik-angle-list)
          )
        
        )
      )
    ;; service call for task state manager
    (cond (ik-res
           (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "SuccessIK")))
          (t
           (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FailIK"))))
    (and ik-res (reverse ik-angle-list)))
  )

(defun main-loop-for-drill
  ()
  (while t
    (ros::sleep)
    (ros::spin-once)
    (when (boundp '*irtviewer*) (x::window-main-one))
    ;; publish handle and approach pose
    (when (and *obj* (find-if #'(lambda (x) (equal (send x :name) :handle)) (send *obj* :descendants)))
      (let* (frame-id)
        (cond ((equal (send *obj* :get :type) :box)
               (setq frame-id "box1"))
              ((equal (send *obj* :get :type) :cylinder)
               (setq frame-id "cylinder1"))
              ((equal (send *obj* :get :type) :torus)
               (setq frame-id "torus1")))
        (publish-object-handle :frame-id frame-id)
        (publish-object-approach :frame-id frame-id)
        ))
    (when *solve-ik-command-flag*
      (cond
       (*inverse-reachability-map-mode* 
        (let* ((inverse-reachability-coords-list-temp nil))
          (dotimes (i (length *inverse-reachability-coords-list*))
            (if (request-ik-from-marker-one :robot-coords (elt *inverse-reachability-coords-list* i))
                (push (elt *inverse-reachability-coords-list* i) inverse-reachability-coords-list-temp)
              )
            )
          (setq *inverse-reachability-coords-list* inverse-reachability-coords-list-temp)
          (show-inverse-reachability-map)
          ))
       (t
        (request-ik-from-marker-one))       
       )
      (setq *reachability-map-mode* nil)
      (setq *inverse-reachability-map-mode* nil)
      )
    (when *solve-ik-from-pose-command-flag*
      (cond
       (*inverse-reachability-map-mode* 
        (setq *pub-grasp-pose-feedback-flag* nil)
        (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* nil)
        (setq *pub-grasp-dual-pose-feedback-flag* nil)
        (let* ((inverse-reachability-coords-list-temp nil))
          (dotimes (i (length *inverse-reachability-coords-list*))
            (if (request-ik-from-pose-one :robot-coords (elt *inverse-reachability-coords-list* i))
                (push (elt *inverse-reachability-coords-list* i) inverse-reachability-coords-list-temp)
              )
            )
          (setq *inverse-reachability-coords-list* inverse-reachability-coords-list-temp)
          (show-inverse-reachability-map)
          ))
       (t
        (request-ik-from-pose-one)) 
       )
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* nil)
      (setq *pub-grasp-dual-pose-feedback-flag* nil)
      (setq *reachability-map-mode* nil)
      (setq *inverse-reachability-map-mode* nil)
      )
    (when *solve-ik-from-dual-pose-command-flag*
      (cond
       (*inverse-reachability-map-mode*
        (setq *pub-grasp-pose-feedback-flag* nil)
        (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* nil)
        (setq *pub-grasp-dual-pose-feedback-flag* nil)
        (let* ((inverse-reachability-coords-list-temp nil))
          (dotimes (i (length *inverse-reachability-coords-list*))
            (if (request-ik-from-pose-one :target-pose-coords-for-second-arm *target-pose-coords-for-second-arm* :initial-pose-coords-for-second-arm *initial-pose-coords-for-second-arm* :move-target-from-second-hand *move-target-from-second-hand* :robot-coords (elt *inverse-reachability-coords-list* i))
                (push (elt *inverse-reachability-coords-list* i) inverse-reachability-coords-list-temp)
              )
            )
          (setq *inverse-reachability-coords-list* inverse-reachability-coords-list-temp)
          (show-inverse-reachability-map)
          ))
       (t
        (request-ik-from-pose-one :target-pose-coords-for-second-arm *target-pose-coords-for-second-arm* :initial-pose-coords-for-second-arm *initial-pose-coords-for-second-arm* :move-target-from-second-hand *move-target-from-second-hand*))
       )
      (setq *pub-grasp-pose-feedback-flag* nil)
      (setq *pub-grasp-not-allow-slip-pose-feedback-pose-flag* nil)
      (setq *pub-grasp-dual-pose-feedback-flag* nil)
      (setq *reachability-map-mode* nil)
      (setq *inverse-reachability-map-mode* nil)
      )
    (when *save-obj-command-flag*
      (save-obj-model-one))
    (when *send-angle-command-flag*
      (send-angle-to-real-robot-one))
    (when *object-menu-command-flag*
      (object-menu-one))
    (when *robot-menu-command-flag*
      (robot-menu-one))
    (when *assoc-obj-flag*
      (set-obj-pose-for-assoc))
    )
  )
