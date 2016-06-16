(require "models/arrow-object")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/ik-solver.l")
(load "package://drc_task_common/euslisp/test-drc-door-task.l")
(load "package://drc_task_common/euslisp/lib/periodic-rate.l")
(load "package://drc_task_common/euslisp/lib/util.l")
(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/zmp.l")
(load "package://drc_task_common/euslisp/lib/robot-model-manager.l")
(load "package://drc_task_common/euslisp/lib/end-effector.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun move-valve-model-from-recog-result (valve origin->recog robot-coords)
  (send valve :newcoords (send origin->recog :copy-worldcoords)))

(defun test-valve-recognition-callback (msg)
  (print 'callback)
  ;; msg := jsk_recognition_msgs::Torus
  (let ((header->valve (ros::tf-pose->coords (send msg :pose)))
        (origin->header (send *tf-listener* :lookup-transform 
                              "odom"
                              (send msg :header :frame_id)
                              (send msg :header :stamp)))
        (origin->ground (send *tf-listener* :lookup-transform "odom" "ground" (ros::time 0))))
    (when (and origin->header origin->ground)
      (ros::ros-info "valid perception")
      (setq *perception-succeeded* t)
      (setq *perception-succeeded-time* (send msg :header :stamp))
      (let ((origin->valve (send (send origin->header :copy-worldcoords)
                                 :transform header->valve)))
        (send *robot* :fix-leg-to-coords origin->ground)
        (send *recog-result* :newcoords (send origin->valve :copy-worldcoords))
        (move-valve-model-from-recog-result *valve* origin->valve origin->ground)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (let ((marker (send *traj* :to-marker "odom")))
          (ros::publish "trajectory_marker" marker))
        ))
    ))

(defun valve-stand-coords (valve)
  (let* ((c (send valve :copy-worldcoords))
         (x (elt (send c :worldpos) 0))
         (y (elt (send c :worldpos) 1))
         (theta-nosign (acos (v. (float-vector 1 0 0)
                                 (send c :rotate-vector (float-vector 0 0 1)))))
         (theta-sign (if (> (elt (v* (float-vector 1 0 0)
                                     (send c :rotate-vector (float-vector 0 0 1))) 2) 0)
                         1 -1))
         (theta (* theta-nosign theta-sign)))
    (let ((ret (make-coords :pos (float-vector x y 0))))
      (send ret :rotate theta :z :local)
      (send ret :translate (float-vector -800 300 0))
      )
    ))

(defun valve-manipulation-trajectory (valve-model &key (robot :hrp2) (z-offset -100))
  (let* ((valve-tmp (valve-model)))
    (let ((initial-theta 45)
          ;;(angle -270))
          (angle -90))
      ;; radius ... 200 mm
      (let ((theta initial-theta)
            (coords-list nil))
        (while (> theta angle)
          (setq coords-list (append coords-list (list (coords-on-valve theta))))
          (setq theta (- theta 20))     ;20 degree
          )
        (instance manipulation-trajectory :init
                  :via-coords
                  (append (list
                           (make-cascoords 
                            :coords
                            (send (send (car coords-list) :copy-worldcoords)
                                  :translate (float-vector -250 0 0))))
                          ;;(list :start-grasp)
                          (mapcar #'(lambda (c i)
                                      (send c :rotate (* (/ (float i) (length coords-list))  (deg2rad -15)) :z)
                                      (make-cascoords :coords (send (send c :copy-worldcoords)
                                                                    :translate (float-vector -100 0 0))))
                                  coords-list (range (length coords-list)))
                          ;;(list :stop-grasp)
                          (list
                           (make-cascoords 
                            :coords
                            (send (send (car (last coords-list)) :copy-worldcoords)
                                  :translate (float-vector -250 0 0))
                            )
                           ;; (make-cascoords 
                           ;;  :coords
                           ;;  (send (send (car coords-list) :copy-worldcoords)
                           ;;        :translate (float-vector -250 0 0))
                           ;;  )
                           )
                          )
                  :parent-object valve-model)))))

(defun coords-on-valve (theta &optional (r 180))
  (let ((c (make-cascoords :pos (point-on-valve theta r))))
    (send c :rotate pi/2 :z)
    (send c :rotate -pi/2 :y)
    (send c :rotate (deg2rad (+ theta)) :x)
    c))

(defun point-on-valve (theta &optional (r 180))
  (float-vector (* r (cos (deg2rad theta)))
                (* r (sin (deg2rad theta)))
                0))

(defun valve-model ()
  (make-ring 200 20 :segments 16))

(defun test-run-valve-recognition ()
  (setq *perception-succeeded* nil)
  (ros::spin-once)
  (setq *perception-succeeded* nil)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    (if *perception-succeeded*
        (return-from test-run-valve-recognition nil))
    ))

(defun test-valve ()
  (setq *man* (instance robot-model-manager :init :jaxon))
  (unless (boundp '*robot*)
    (setq *robot* (send *man* :model :visualization)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (send *valve* :newcoords (make-coords :pos (float-vector 900 -300 1200)))
  (send *valve* :rotate (deg2rad 90) :y)
  (send *valve* :rotate (deg2rad -90) :z)
  (objects (list *robot* *valve* *traj*))
  )

(defun test-valve-recognition (robot-type)
  (ros::roseus "test_valve_recognition")
  (setq *tf-listener* (instance ros::transform-listener :init))
  (load-robot-model :jaxon)
  (unless (boundp '*ri*)
    (jaxon_red-init))
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; run recognition until perception succeeded
  (test-run-valve-recognition)
  )

(defun test-valve-motion ()
  ;;(send *ri* :start-impedance :arms :k-p 300 :d-p 800 :m-p 10 :m-r 2 :d-r 80 :k-r 40)
  ;;(send *ri* :set-interpolation-mode :linear)
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (unless (send *col* :initializedp)
    (send *robot* :reset-manip-pose)
    (send *col* :init-distance-table))
  (send *ik-solver* :solve
        :trajectory-resolution 20
               :stop 50
               :collision-num 10 :debug-view t)
  )

(defun valve-ee ()
  (ros::roseus "foobar")
  (test-valve)
  (unless (boundp '*col*)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send (send *man* :model :collision) :reset-manip-pose)
    (send *col* :init-distance-table))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (let ((avs (send *ik-solver* :solve
                      :trajectory-resolution 20
                      :stop 50
                      :collision-num 10 :debug-view nil))
        (ee (instance end-effector-evaluator :init *robot* :rarm)))
    (let ((tbl (send ee :profile avs :save-full-log t :log-name "valve"
                     :dspeed-factor 0.1
                     :max-speed-factor 10.0)))
      tbl)
    )
  )

(defun valve-zmp ()
  (ros::roseus "foobar")
  (test-valve)
  (unless (boundp '*col*)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send (send *man* :model :collision) :reset-manip-pose)
    (send *col* :init-distance-table))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (let ((avs (send *ik-solver* :solve
                      :trajectory-resolution 20
                      :stop 50
                      :collision-num 10 :debug-view nil)))
    (let* ((zmp-robot (send *man* :model :zmp))
           (zmp (instance zmp-evaluator :init zmp-robot)))
      (send (send zmp :profile :angle-vector-sequence avs :start-angle-vector (send *robot* :reset-manip-pose)
                  :dt 0.01 :max-speed-factor 10.0)
            :write "zmp.csv")
      )))

(defun valve-motion-profile ()
  (test-valve)
  (unless (boundp '*col*)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send (send *man* :model :collision) :reset-manip-pose)
    (send *col* :init-distance-table))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *ik-profile* (send *ik-solver* :profile :test-num 10 :max-trajectory 20 :max-collision 10))
  (let ((tbl (send *ik-profile* :average '("time") '("success")))
        (target-collision-nums '(0 1 5 9)))
    (let ((collision-per-tables (mapcar #'(lambda (c)
                                            (send tbl :filter-if #'(lambda (tbl column data)
                                                                     (= c (elt data (send tbl :column-index "collision-num"))))))
                                        target-collision-nums)))
      (ros::advertise "/ik_plot_resolution" jsk_recognition_msgs::PlotDataArray)
      (ros::publish "/ik_plot_resolution"
        (multi-plot-data (mapcar #'(lambda (c i)
                                     (list (send c :column-data "trajectory-num")
                                           (send c :column-data "time")
                                           :label (format nil "$N_{collision} = ~A$" i)
                                           ))
                                 collision-per-tables target-collision-nums)
                         :legend-font-size 24))
      ))
  (let ((tbl (send *ik-profile* :average '("time") '("success")))
        (target-trajectory-nums '(11 19)))
    (let ((trajectory-per-tables (mapcar #'(lambda (c)
                                            (send tbl :filter-if #'(lambda (tbl column data)
                                                                     (= c (elt data (send tbl :column-index "trajectory-num"))))))
                                        target-trajectory-nums)))
      (ros::advertise "/ik_plot_resolution" jsk_recognition_msgs::PlotDataArray)
      (ros::publish "/ik_plot_resolution"
        (multi-plot-data (mapcar #'(lambda (c i)
                                     (list (send c :column-data "collision-num")
                                           (send c :column-data "time")
                                           :label (format nil "$N_{trajectory} = ~A$" i)
                                           ))
                                 trajectory-per-tables target-trajectory-nums)
                         :legend-font-size 24))
      ))
  )

(defun test-valve-zmp ()
  (ros::roseus "foo")
  (format t "setup environment~%")
  (test-valve)                          ;setup env
  (format t  "generate motion~%")
  (let* ((avs (test-valve-motion))
         (zmp-robot (instance jaxon_red-robot :init))
         (zmp (instance zmp-evaluator :init zmp-robot)))
    (format t "generating zmp trajectory~%")
    (let ((zmp-traj (send zmp :evaluate-zmp :angle-vector-sequence avs
                              :start-angle-vector (send *robot* :reset-manip-pose)
                              :dt 0.01
                              :speed-factor 10.0
                              :debug-view nil)))
      (send zmp-traj :publish-zmp-plot)
      (send zmp-traj :evaluate)
      )))

(defun test-valve-motion-zmp ()
  (ros::advertise "/motion_scale" std_msgs::Float32)
  (let ((scales '(10.0 9.0 8.0 7.0 6.0 5.0 4.0 3.5 3.0 2.5 2.0 1.9 1.8 1.7 1.6 1.5 1.4 1.3 1.2)))
    (dolist (scale scales)
      (ros::publish "/motion_scale" (instance std_msgs::Float32 :data scale))
      (progn (send *avs* :send-angle-vector-sequence-test *ri* *robot* scale)
             (send *ri* :wait-interpolation)
             (send *ri* :angle-vector (send *robot* :reset-manip-pose) 3000)
             (send *ri* :wait-interpolation))
      )))

;; (load "package://drc_task_common/euslisp/lib/valve.l")
;; (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
;; (test-valve-recognition :jaxon)
;; (test-run-valve-recognition)
;; (setq hoge (test-valve-motion))
;; (send *avs* :send-angle-vector-sequence-test *ri* *robot* 10.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 5.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 3.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 2.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 1.0)
;; (do-until-key (test-run-valve-recognition) (setq hoge (test-valve-motion)) (send hoge :filter-look-at-hand *robot* :rarm) (send hoge :send-angle-vector-sequence-test *ri* *robot* 2.0) (send *ri* :wait-interpolation) (unix:sleep 10))
