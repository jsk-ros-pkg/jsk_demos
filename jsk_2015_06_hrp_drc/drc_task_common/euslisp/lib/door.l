(require "models/arrow-object")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/ik-solver.l")
(load "package://drc_task_common/euslisp/test-drc-door-task.l")
(load "package://drc_task_common/euslisp/lib/periodic-rate.l")
(load "package://drc_task_common/euslisp/lib/util.l")
(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/zmp.l")
(load "package://drc_task_common/euslisp/lib/robot-model-manager.l")
(ros::roseus-add-msgs "geometry_msgs")

(defun move-door-model-from-knob-coords (door origin->recog robot-coords)
  (send door :newcoords (make-coords))  ;force to reset
  (let* ((origin->door (send door :copy-worldcoords))
         (origin->knob (send (send door :handle :knob-handle) :copy-worldcoords))
         (door->knob (send origin->door :transformation origin->knob))
         ;;(door->knob* (send (send (send door->knob :copy-worldcoords) :rotate -pi/2 :x) :rotate 0 :z))
         (door->knob* (send (send (send door->knob :copy-worldcoords) :rotate -pi/2 :x) :rotate 0 :z))
         (knob*->door (send door->knob* :inverse-transformation))
         (origin->knob* (send (send origin->door :copy-worldcoords) :transform door->knob*))
         (origin->recog-pos (zset (send origin->recog :worldpos) (elt (send origin->knob* :worldpos) 2)))
         (origin->recog-yaw (acos (v. 
                                   (normalize-vector 
                                    (z0 (send origin->recog :rotate-vector (float-vector 1 0 0))))
                                   (float-vector 1 0 0))))
         (normalized-knob-coords (make-coords :pos origin->recog-pos)))
    (format t "origin->recog-yaw: ~A~%" (rad2deg origin->recog-yaw))
    (format t "direction: ~A~%" (elt (v* (normalize-vector 
                                        (z0 (send origin->recog :rotate-vector (float-vector 1 0 0))))
                                         (float-vector 1 0 0)) 2))
    (if (< (elt (v* (normalize-vector 
                     (z0 (send origin->recog :rotate-vector (float-vector 1 0 0))))
                    (float-vector 1 0 0)) 2) 0)
        (send normalized-knob-coords :rotate origin->recog-yaw :z :local)
        (send normalized-knob-coords :rotate (- origin->recog-yaw) :z :local)
      )
    (send normalized-knob-coords :rotate pi :z :local)
    (send normalized-knob-coords :draw-on :flush t)
    (send origin->knob* :draw-on :flush t)
    (send door :newcoords (send normalized-knob-coords :transform knob*->door))
    ))

;; for debug purpse
(defun test-door-recognition-callback (msg)
  (print 'callback)
  ;; msg := geometry_msgs/PoseStamped
  ;; it requires *tf-listener*
  (let ((origin->ground (send *tf-listener* :lookup-transform "odom" "ground" (ros::time 0)))
        (origin->msg 
         (send *tf-listener* :lookup-transform "odom" (send msg :header :frame_id) 
               ;;(send msg :header :stamp)
               (ros::time 0)
               )))
    (send *robot* :fix-leg-to-coords origin->ground)
    ;; (send *robot* :fix-leg-to-coords (make-coords :pos (z0 (send origin->ground :worldpos))
    ;;                                               :rot (send origin->ground :worldrot)))
    (when origin->msg
      (setq *perception-succeeded* t)
      (setq *perception-succeeded-time* (send msg :header :stamp))
      (send *door* :newcoords (make-coords))
      (send *robot* :fix-leg-to-coords origin->ground)
      (let ((traj (door-manipulation-trajectory *door* :robot :jaxon)))
        (setq *traj* traj)
        (let ((origin->recog (send origin->msg :transform (ros::tf-pose-stamped->coords msg))))
          ;; check orientation by ground->recog
          (let ((ground->recog (send (send origin->ground :inverse-transformation)
                                     :transform origin->recog)))
            ;; check x axis of ground->recog
            (when (> (v. (float-vector 1 0 0) (send ground->recog :rotate-vector (float-vector 1 0 0))) 0)
              ;; flip origin->recog
              (ros::ros-info "flip origin->recog")
              (send origin->recog :rotate pi :z :local)
              )
            )
          (send *recog-result* :newcoords (send origin->recog :copy-worldcoords))
          (move-door-model-from-knob-coords *door* origin->recog origin->ground)
          (let ((marker (send traj :to-marker "odom")))
            (ros::publish "trajectory_marker" marker)
            ))
        ))
    )
  ;; test ik-solver
  ;;(setup-move-targets *robot*)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  )


(defun test-door-recognition-callback-old (msg)
  (print 'callback)
  ;; msg := geometry_msgs/PoseStamped
  ;; it requires *tf-listener*
  (let ((origin->ground (send *tf-listener* :lookup-transform "odom" "ground" (ros::time 0)))
        (origin->msg 
         (send *tf-listener* :lookup-transform "odom" (send msg :header :frame_id) 
               ;;(send msg :header :stamp)
               (ros::time 0)
               )))
    (send *robot* :fix-leg-to-coords origin->ground)
    ;; (send *robot* :fix-leg-to-coords (make-coords :pos (z0 (send origin->ground :worldpos))
    ;;                                               :rot (send origin->ground :worldrot)))
    (when origin->msg
      (setq *perception-succeeded* t)
      (setq *perception-succeeded-time* (send msg :header :stamp))
      (send *door* :newcoords (make-coords))
      (send *robot* :fix-leg-to-coords origin->ground)
      (let ((traj (door-manipulation-trajectory *door* :robot :jaxon)))
        (setq *traj* traj)
        (let ((origin->recog (send origin->msg :transform (ros::tf-pose-stamped->coords msg))))
          (send *recog-result* :newcoords (send origin->recog :copy-worldcoords))
          (move-door-model-from-knob-coords *door* origin->recog origin->ground)
          (let ((marker (send traj :to-marker "odom")))
            (ros::publish "trajectory_marker" marker)
            ))
        ))
    )
  ;; test ik-solver
  ;;(setup-move-targets *robot*)
  (send *irtviewer* :draw-objects)
  (x::window-main-one))


(defun test-door-recognition (robot-type)
  (ros::roseus "test_door_recognition")
  (setq *tf-listener* (instance ros::transform-listener :init))
  (setq *man* (instance robot-model-manager :init :jaxon))
  (setq *robot* (send *man* :model :visualization))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *door* *robot* *recog-result*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::subscribe "/door_detection/door_handle" 
    geometry_msgs::PoseStamped #'test-door-recognition-callback)
  ;; (ros::subscribe "/plane_supported_cuboid_estimator/output/result_pose"
  ;;   geometry_msgs::PoseStamped #'test-door-recognition-callback)
  ;;(ros::spin)
  )

(defun door-stand-coords (door)
  (let ((c (send door :copy-worldcoords)))
    (send c :translate (float-vector -900 -100 0))
    c))

(defun test-move-door-model-from-knob-coords ()
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (require "models/arrow-object")
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (send *robot* :reset-manip-pose)
  
  (let ((rate (instance periodic-rate :init 30))
        (handle-arrow (instance arrow-object :init)))
    ;;(objects (list *robot* handle-arrow *door*))
    (unless (boundp '*irtviewer*)
      (make-irtviewer))
    (send *irtviewer* :objects (list *robot* handle-arrow *door*))
    (dolist (robot-pos (list (float-vector 0 0 0)
                             (float-vector 100 0 0)
                             (float-vector 100 100 0)
                             (float-vector 0 100 0)))
      (dolist (robot-rot (list 0 20 40 60 80 100 120 140 160 
                               180 200 220 240 260 280 300 320 340))
        (let ((robot-coords (make-coords :pos robot-pos)))
          (send robot-coords :rotate (deg2rad robot-rot) :z)
          (send *robot* :fix-leg-to-coords robot-coords)
          (dotimes (ix 100)
            (dotimes (iy 100)
              (dotimes (iz 20)
                (let* ((handle-pos-offset (float-vector 1000 0 0))
                       (handle-pos (v+ handle-pos-offset (float-vector (- 500 (* 10 ix)) 
                                                                       (- 500 (* 10 iy))
                                                                       950)))
                       (handle-coords (make-coords)))
                  (send handle-coords :locate handle-pos :world)
                  (send handle-coords :rotate (* iz (/ 2pi 20)) :z :world)
                  (send handle-arrow :newcoords handle-coords)
                  (move-door-model-from-knob-coords *door* handle-coords robot-coords)
                  (send *irtviewer* :draw-objects)
                  (x::window-main-one)
                  (send rate :sleep)
                  ;;(return-from test-move-door-model-from-knob-coords nil)
                  )))
            ))))))

(defun door-manipulation-trajectory (door-model &key (robot :hrp2))
  (let* ((door-tmp (instance param-door :init 700 60 100 20
                             (float-vector 0 -640 950)
                             (float-vector -60 -600 950)
                             :use-leg t
                             :handle-l/r :right))
         (knob-coords (send door-tmp :handle :knob-handle)))
    (instance manipulation-trajectory :init
              :via-coords
              (cond ((eq robot :hrp2)
                     (list (make-coords-relative knob-coords :pos (float-vector -100 200 -100) :rpy (float-vector 0 0 0))
                           (make-coords-relative knob-coords :pos (float-vector 0 50 -50) :rpy (float-vector 0 0 0))
                           (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle -70)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0))
                             )
                           (progn
                             (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                             (make-coords-relative knob-coords :pos (float-vector -30 -25 -50) :rpy (float-vector 0 0 0)))
                           
                           (make-coords-relative knob-coords :pos (float-vector -30 75 50) :rpy (float-vector 0 0 0))
                           (make-coords-relative knob-coords :pos (float-vector -30 150 50):rpy (float-vector 0 0 0))
                           (make-coords-relative knob-coords :pos (float-vector -100 150 50) :rpy (float-vector 0 0 0))))
                    ((eq robot :jaxon)
                     (list (make-coords-relative knob-coords :pos (float-vector -200 -25 -50) :rpy (float-vector 0 0 0))
                           :start-grasp
                           (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                             (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                             (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                             (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0)))
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                             (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0)))
                           :stop-grasp
                           (progn
                             (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                             (make-coords-relative knob-coords :pos (float-vector -100 -25 -50) :rpy (float-vector 0 0 0)))
                           )
                     ;; (list (make-coords-relative knob-coords :pos (float-vector -200 -25 -25) :rpy (float-vector 0 0 0))
                     ;;       :start-grasp
                     ;;       (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0))
                     ;;       (progn
                     ;;         (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                     ;;         (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0)))
                     ;;       (progn
                     ;;         (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                     ;;         (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0)))
                     ;;       (progn
                     ;;         (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                     ;;         (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0)))
                     ;;       (progn
                     ;;         (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                     ;;         (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0)))
                     ;;       :stop-grasp
                     ;;       (progn
                     ;;         (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                     ;;         (make-coords-relative knob-coords :pos (float-vector -150 -25 -25) :rpy (float-vector 0 0 0)))
                     ;;       )
                     )
                    )
              :parent-object (send door-model :handle :knob-handle))))

(defun door-test-motion ()
  ;; init hrp2
  (ros::roseus "test_door_motion")
  ;;(setq *tf-listener* (instance ros::transform-listener :init))
  ;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (require "models/arrow-object")
  (load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
  (load "package://drc_task_common/euslisp/lib/ik-solver.l")
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *recog-result* (instance arrow-object :init))
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (setup-move-targets *robot*)
  (hrp2jsknt-init)
  (objects (list *door* *robot* *recog-result*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::subscribe "/door_detection/door_handle" 
    geometry_msgs::PoseStamped #'test-door-recognition-callback)
  (setq *col* (instance collision-link-parameter :init *robot*))
  
  (send *robot* :reset-manip-pose)
  (send *col* :init-distance-table)
  (ros::spin)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (send *robot* :reset-manip-pose)
  (send *col* :init-distance-table)
  (send *ik-solver* :solve
        :stop 30
        :trajectory-resolution 10
        :collision-num 10 :debug-view t)
  )

(defun test-door-motion ()
  (unless (boundp '*col*)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *robot* :reset-manip-pose)
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
    (send *robot* :reset-manip-pose)
    (robot-move-height *robot* 50)
    (send *robot* :fix-leg-to-coords foot-midcoords))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (send *ik-solver* :solve
               :trajectory-resolution 10
               :collision-num 10 :debug-view t)
  )

(defun test-door (&optional (ik-type :use-torso))
  (unless (boundp '*man*)
    (setq *man* (instance robot-model-manager :init :jaxon)))
  (setq *robot* (send *man* :model :visualization))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (send *door* :locate (float-vector 600 0 0) :world)
  (objects (list *door* *robot* *traj*))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (unless (boundp '*col*)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table)
    )
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type ik-type
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  )

(defun door-ee ()
  (ros::roseus "foobar")
  (test-door)
  (let ((avs (send *ik-solver* :solve
                   :trajectory-resolution 20
                   :stop 50
                   :collision-num 10 :debug-view nil))
        (ee (instance end-effector-evaluator :init *robot* :rarm)))
    (let ((tbl (send ee :profile avs :save-full-log t :log-name "door"
                     :dspeed-factor 0.1
                     :max-speed-factor 10.0)))
      tbl)
    ))

(defun door-zmp ()
  (ros::roseus "foobar")
  (test-door)
  (let ((avs (send *ik-solver* :solve
                      :trajectory-resolution 20
                      :stop 50
                      :collision-num 10 :debug-view nil)))
    (let* ((zmp-robot (send *man* :model :zmp))
           (zmp (instance zmp-evaluator :init zmp-robot)))
      (send (send zmp :profile :angle-vector-sequence avs :start-angle-vector (send *robot* :reset-manip-pose)
                  :dt 0.01 :max-speed-factor 10.0)
            :write "zmp_door.csv")
      )))


;; (load "package://drc_task_common/euslisp/lib/door.l")
;; (test-move-door-model-from-knob-coords)
;; (test-door-recognition :jaxon)
;; (send *ri* :start-impedance :arms :k-p 300 :d-p 800 :m-p 10 :m-r 2 :d-r 80 :k-r 40)
