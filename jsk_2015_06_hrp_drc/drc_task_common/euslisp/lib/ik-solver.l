(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/angle-vector-sequence.l")
(load "package://jsk_footstep_planner/euslisp/footplace_planner_for_manipulation.l")
(load "package://drc_task_common/euslisp/lib/table.l")
(load "package://drc_task_common/euslisp/lib/plotlib.l")

(defun robot-footpoint (r)
  (apply #'midpoint 0.5
         (send r :legs :end-coords :worldpos)))

(defun robot-move-height (r height)
  ;; relative
  (send r :legs :move-end-pos (float-vector 0 0 height))
  )

;; (defmethod euscollada-robot
;;   (:footpoint ()
;;     (apply #'midpoint 0.5
;;            (send self :legs :end-coords :worldpos))
;;     )
;;   )

(defclass ik-solver
  :super propertied-object
  :slots (robot trajectory ik-type move-target col-parameter prepare-angle-vector))

(defmethod ik-solver
  (:init (r aarm &key
            ((:move-target amove-target) nil)
            ((:ik-type aik-type) :arm)
            ((:trajectory traj))
            ((:prepare-angle-vector pav))
            (collision-parameter))
    (setq robot r)
    (setq prepare-angle-vector pav)
    (setq arm aarm)
    (if (null amove-target)
        (setq move-target (send r aarm :end-coords))
      (setq move-target amove-target))
    (setq ik-type aik-type)
    (setq trajectory traj)
    (setq col-parameter collision-parameter)
    ;;(setq trajectory traj)
    self)
  (:ik-type () ik-type)
  (:link-list (&key (ik-type (send self :ik-type)))
    (cond ((eq ik-type :arm)
           (send robot :link-list
                 (send move-target :parent)
                 (car (send robot arm :links))))
          ((eq ik-type :use-torso)
           (send robot :link-list
                 (send move-target :parent)))
          ((eq ik-type :fullbody) ;legs will be appended inside of :fullbody-inverse-kinematics
           (send robot :link-list
                 (send move-target :parent)))
          ((eq ik-type :fullbody-with-standcoords) ;legs will be appended inside of :fullbody-inverse-kinematics
           (send robot :link-list (send move-target :parent)))
          (t
           (error "unknown ik-type: ~A" ik-type)))
    )
  (:solve-normal-ik (&key via-coords
                          collision-num
                          stop
                          debug-view
                          grasp-commands
                          (ik-type (send self :ik-type)))
    (when debug-view                    ;change link-list color
      (mapcar #'(lambda (c)
                  (send c :set-color (float-vector 0 1 0) nil))
              (flatten (send-all (send self :link-list :ik-type ik-type) :bodies))))
    (let ((avs (instance angle-vector-sequence :init)))
      (dolist (c via-coords)
        (let ((av (send robot :inverse-kinematics c
                        :link-list (send self :link-list :ik-type ik-type)
                        :move-target move-target
                        :stop stop
                        :avoid-collision-distance 100
                        :avoid-collision-null-gain 5.0
                        :avoid-collision-joint-gain 0.8
                        :collision-avoidance-link-pair
                        (send col-parameter :collision-avoidance-link-pair
                              (send self :link-list :ik-type ik-type)
                              collision-num)
                        :additional-weight-list
                        (list (list (send robot :torso :waist-p :child-link) 0.1)
                              (list (send robot :torso :waist-y :child-link) 0.3)
                              (list (send robot :torso :waist-r :child-link) 0.0))
                        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0 0 0 0.1)
                        :debug-view (if debug-view :no-message nil))))
          (if av (send avs :add av)
            (return-from :solve-normal-ik nil))
          (when (and grasp-commands
                     (= (caar grasp-commands) (get c :raw-index-value)))
            (send avs :add (cdr (car grasp-commands)))
            (setq grasp-commands (cdr grasp-commands)))
          ))
      avs))
  (:solve-fullbody-ik (&key via-coords
                            collision-num
                            stop
                            debug-view)
    (let ((leg-target-coords (list (send robot :lleg :end-coords :copy-worldcoords)
                                   (send robot :rleg :end-coords :copy-worldcoords)))
          (ret (instance angle-vector-sequence :init)))
      (dolist (c via-coords)
        (with-move-target-link-list
         (mt ll robot (list arm :lleg :rleg))
         (when debug-view
           (mapcar #'(lambda (c)
                  (send c :set-color (float-vector 0 1 0) nil))
                   (flatten (send-all (flatten ll) :bodies))))
         (setf (elt mt 0) move-target)  ;overwrite move-target
         (let ((res
                (send robot
                      :fullbody-inverse-kinematics (cons c leg-target-coords)
                      :move-target mt :link-list ll
                      :root-link-virtual-joint-weight
                      (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
                      :target-centroid-pos (robot-footpoint robot)
                      :translation-axis (list t t t)
                      :rotation-axis (list t t t)
                      :avoid-collision-distance 100
                      :avoid-collision-null-gain 5.0
                      :avoid-collision-joint-gain 0.8
                      :collision-avoidance-link-pair
                      (send col-parameter :collision-avoidance-link-pair
                            (send self :link-list)
                            collision-num)
                      :additional-weight-list
                      (list (list (send robot :torso :waist-p :child-link) 0.0)
                            (list (send robot :torso :waist-y :child-link) 0.3)
                            (list (send robot :torso :waist-r :child-link) 0.0))
                      ;; (list (list (send robot :rleg :toe-p :child-link) 0)
                      ;;       (list (send robot :lleg :toe-p :child-link) 0))
                      :max (float-vector  500  500  0  20  20  10)
                      :min (float-vector  -500  -500  -200  -20  -20  -10)
                      :stop stop
                      :centroid-thre 15
                      :debug-view (if debug-view :no-message nil))))
           (if res
               (send ret :add res)
             (return-from :solve-fullbody-ik nil)
           )
           )))))
  (:solve-fullbody-with-standcoords-ik (&key via-coords
                                             collision-num
                                             stop
                                             debug-view)
    (let ((leg-target-coords (list (send robot :lleg :end-coords :copy-worldcoords)
                                   (send robot :rleg :end-coords :copy-worldcoords)))
          (ret (instance angle-vector-sequence :init))
          (before-stand-coords (send robot :foot-midcoords)))
      (let ((res (fullbody-inverse-kinematics-with-standcoords 
                  robot 
                  via-coords
                  :move-centroid t
                  :move-target (list move-target)
                  ;; :translation-axis (list t t t)
                  ;; :rotation-axis (list t t t)
                  :avoid-collision-distance 100
                  :avoid-collision-null-gain 5.0
                  :avoid-collision-joint-gain 0.8
                  :collision-avoidance-link-pair
                  (send col-parameter :collision-avoidance-link-pair
                        (send self :link-list)
                        collision-num)
                  :root-link-virtual-joint-weight (float-vector 0.005 0.01 0 0 0 0.05)
                  :additional-weight-list
                  (list (list (send robot :torso :waist-p :child-link) 0.1)
                        (list (send robot :torso :waist-y :child-link) 0.3)
                        (list (send robot :torso :waist-r :child-link) 0.0))
                  :stop stop
                  :centroid-thre 15
                  :warnp nil
                  :debug (if debug-view :no-message nil))))
        ;; res := (list standpos angle-vector-list)
        (ros::ros-warn "try :use-torso because :fullbody-with-standcoords failed")
        (let ((use-torso-res (send self :solve-normal-ik :via-coords via-coords :collision-num collision-num
                                   :stop stop :debug-view debug-view :ik-type :use-torso)))
          (if use-torso-res
              (send use-torso-res :add-go-pos-arg before-stand-coords (send robot :foot-midcoords)))
          use-torso-res))))
        ;; (if (null (cadr res))
        ;;     ;; even though fullbody-inverse-kinematics-with-standcoords fails, 
        ;;     ;; it is worth to test :use-torso ik
        ;;     (progn
        ;;       (ros::ros-warn "try :use-torso because :fullbody-with-standcoords failed")
        ;;       (let ((use-torso-res (send self :solve-normal-ik :via-coords via-coords :collision-num collision-num
        ;;                                  :stop stop :debug-view debug-view :ik-type :use-torso)))
        ;;         (if use-torso-res
        ;;             (send use-torso-res :add-go-pos-arg before-stand-coords (send robot :foot-midcoords)))
        ;;         use-torso-res))
        ;;   res))))
        ;;   )
        ;; (if res
        ;;     (print res)
        ;;     ;;(send ret :add res)
        ;;   (return-from :solve-fullbody-with-standcoords-ik nil)
  (:solve (&key (trajectory-resolution 0)
                (collision-num 0)
                (stop 50)
                (debug-view nil))
    (unwind-protect
        ;; move robot model with keeping stand coords
        (progn
          (if prepare-angle-vector
              (let ((mid (send robot :foot-midcoords)))
                (send robot :angle-vector prepare-angle-vector)
                (send robot :fix-leg-to-coords mid)))
          (let ((via-coords (send trajectory :via-coords trajectory-resolution)))
            (cond ((or (eq ik-type :arm)
                       (eq ik-type :use-torso))
                   (send self :solve-normal-ik :via-coords via-coords
                         :collision-num collision-num
                         :grasp-commands (send trajectory :grasp-commands)
                         :stop stop
                         :debug-view debug-view))
                  ((eq ik-type :fullbody)
                   (send self :solve-fullbody-ik
                         :via-coords via-coords
                         :stop stop
                         :collision-num collision-num
                         :debug-view debug-view))
                  ((eq ik-type :fullbody-with-standcoords)
                   (send self :solve-fullbody-with-standcoords-ik
                         :via-coords via-coords
                         :stop stop
                         :collision-num collision-num
                         :debug-view debug-view))
                  (t
                   (error "not supported"))
                  )))
      (when debug-view
        (send-all (send robot :bodies) :set-color nil nil))
      )
    )
  (:profile 
   (&key (max-trajectory 50) (test-num 10) (stop 50) (max-collision nil))
   (let ((tbl (instance table :init '("time" "collision-num" "trajectory-num" "success" "q")))
         (start-coords (send robot :foot-midcoords)))
     (let ((min-trajectory (send trajectory :min-resolution))
           (max-collision (if max-collision max-collision (send col-parameter :max-collision-link-pairs (send self :link-list)))))
       (format t "min-trajectory: ~A~%" min-trajectory)
       (format t "max-trajectory: ~A~%" max-trajectory)
       (format t "max-collision: ~A~%" max-collision)
       (format t "test-num: ~A~%" test-num)
       (dotimes (j max-collision)
         (dotimes (i max-trajectory)
           (when (and (= (mod j 2) 0)
                      (= (mod i 2) 0))
         (when (> i min-trajectory)
             (dotimes (k test-num)
               ;; record all the data
               (format t "test ~A:~%" k)
               (format t "  trajectory: ~A~%" i)
               (format t "  collision: ~A~%" j)
               (let ((timer (instance mtimer :init)))
                 (send robot :fix-leg-to-coords start-coords)
                 (send timer :start)
                 (let ((result (send self :solve :trajectory-resolution i
                                     :collision-num j
                                     :stop stop)))
                   (unless result
                     (warn "failed to solve ik, skip")
                     (return-from nil nil))
                   (let ((tm (send timer :stop)))
                     (send tbl :add (list tm j i (if result 1 0)
                                          (* (normalize-value j max-collision 0)
                                             (normalize-value i max-trajectory min-trajectory))))
                     ))))))))
       (send tbl :print))
     tbl))
  )


(defun test-ik-solver ()
  (load "package://drc_task_common/euslisp/test-drc-door-task.l")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (setup-move-targets *robot*)
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *traj* (door-trajectory))
  (setf (get *traj* :face-color) :red)
  (send *door* :locate (float-vector 800 200 0) :world)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (objects (list *door* *robot* *traj*))
  (send *robot* :reset-manip-pose)
  (send *col* :init-distance-table)
  (bench (send *ik-solver* :solve
               :trajectory-resolution 10
               :collision-num 50 :debug-view t))
  )

(defun test-ik-solver-jaxon ()
  (load "package://drc_task_common/euslisp/test-drc-door-task.l")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (setup-move-targets *robot*)
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *traj* (door-trajectory-jaxon))
  (setf (get *traj* :face-color) :red)
  (send *door* :locate (float-vector 800 200 0) :world)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (objects (list *door* *robot* *traj*))
  (send *robot* :reset-manip-pose)
  (send *col* :init-distance-table)
  (bench (send *ik-solver* :solve
               :trajectory-resolution 10
               :collision-num 10 :debug-view t))
  )

(defun test-fullbody-ik-solver-jaxon (&optional (pos (float-vector 600 200 0)))
  (load "package://drc_task_common/euslisp/test-drc-door-task.l")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (setup-move-targets *robot*)
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *traj* (door-trajectory-jaxon))
  (setf (get *traj* :face-color) :red)
  (send *door* :locate pos :world)
  (send *door* :rotate (deg2rad 20) :z)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (objects (list *door* *robot* *traj*))
  (send *robot* :reset-manip-pose)
  (robot-move-height *robot* 90)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *col* :init-distance-table)
  (bench (send *ik-solver* :solve
               :trajectory-resolution 10
               :collision-num 10 :debug-view t))
  )


(defun test-fullbody-ik-solver ()
  (load "package://drc_task_common/euslisp/test-drc-door-task.l")
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (setup-move-targets *robot*)
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *traj* (door-trajectory))
  (setf (get *traj* :face-color) :red)
  (send *door* :locate (float-vector 800 200 0) :world)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :fullbody
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*))
  (objects (list *door* *robot* *traj*))
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *col* :init-distance-table)
  (bench (send *ik-solver* :solve
               :stop 30
               :trajectory-resolution 50
               :collision-num 50 :debug-view t))
  )

(defun ik-csv-average (file)
  (let ((tbl (load-table file)))
    (send tbl :average '("time") '())))
    

;; (load "package://drc_task_common/euslisp/lib/ik-solver.l")
;; (send (ik-csv-average "package://drc_task_common/profile_data/motion/jaxon_door_ik.csv") :write "package://drc_task_common/profile_data/motion/jaxon_door_ik_average.csv")
;; (send (ik-csv-average "package://drc_task_common/profile_data/motion/jaxon_valve_ik.csv") :write "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average.csv")
;; (send (ik-csv-average "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2.csv") :write "package://drc_task_common/profile_data/motion/jaxon_valve_ik_stand2_average.csv")
;; (send (ik-csv-average "package://drc_task_common/profile_data/motion/jaxon_door_ik_stand2.csv") :write "package://drc_task_common/profile_data/motion/jaxon_door_ik_stand2_average.csv")


;; plot ik table codes
;; (setq *tbl* (load-table "package://drc_task_common/profile_data/motion/jaxon_valve_ik_average.csv"))
;; plot ik with different collision num
(defun plot-ik-table-for-resolution (file &optional (collisions '(0 1 3 5 10 20 39)))
  (let ((tbl (load-table file)))
    (let ((per-collision-tables (mapcar #'(lambda (n)
                                            (send tbl :filter-if #'(lambda (tbl column d)
                                                                     (= n (car (send tbl :get-columns-values '("collision-num") d))))))
                                        collisions)))
      (ros::advertise "/plot" jsk_recognition_msgs::PlotDataArray)
      (ros::publish "/plot" (multi-plot-data (mapcar #'(lambda (tbl n)
                                                         (list (send tbl :get-column-value "trajectory-num")
                                                               (send tbl :get-column-value "time")
                                                               :label (format nil "Collision Pair = ~A" n)
                                                               ))
                                                     per-collision-tables collisions)
                                             :min-y 0
                                             :max-y 12
                                             :no-legend t
                                             :legend-font-size 24))
      )))

(defun plot-ik-table-for-collision (file &optional (trajectories '(8 12 20 29)))
  (let ((tbl (load-table file)))
    (let ((per-trajectory-tables (mapcar #'(lambda (n)
                                            (send tbl :filter-if #'(lambda (tbl column d)
                                                                     (= n (car (send tbl :get-columns-values '("trajectory-num") d))))))
                                        trajectories)))
      (ros::advertise "/plot" jsk_recognition_msgs::PlotDataArray)
      (ros::publish "/plot" (multi-plot-data (mapcar #'(lambda (tbl n)
                                                         (list (send tbl :get-column-value "collision-num")
                                                               (send tbl :get-column-value "time")
                                                               :label (format nil "Trajectory = ~A" n)
                                                               ))
                                                     per-trajectory-tables trajectories)
                                             :min-y 0
                                             :max-y 12
                                             :no-legend t
                                             :legend-font-size 24))
      )))


;; (mapcar #'(lambda (n) (send *tbl* :filter-if #'(lambda (tbl colu
;; (send *tbl* :filter-if #'(lambda (tbl column d) (= (car (send tbl :get-columns-values '("collision-num") d)) 0)))
;; modify ik csv  
(defun add-q-to-ik-table (orig-table)
  (let ((min-collision-num (apply #'min (send orig-table :get-column-value "collision-num")))
        (max-collision-num (apply #'max (send orig-table :get-column-value "collision-num")))
        (min-trajectory-num (apply #'min (send orig-table :get-column-value "trajectory-num")))
        (max-trajectory-num (apply #'max (send orig-table :get-column-value "trajectory-num"))))
    (let ((tbl (instance table :init (append (send orig-table :column) (list "q")))))
      (dolist (d (send orig-table :data))
        (multiple-value-bind
         (time collision trajectory success)
         (send orig-table :get-columns-values
               '("time" "collision-num" "trajectory-num" "success")
               d)
         (let ((q (* (normalize-value collision max-collision-num min-collision-num)
                     (normalize-value trajectory max-trajectory-num min-trajectory-num))))
         (send tbl :add (list time collision trajectory success q))
        )
      ))
      tbl)))

(defun filter-table-as-monotonically-increasing (tbl x-label y-label)
  (send tbl :sort-by x-label)
  (let ((new-table (instance table :init (send tbl :column)))
        (prev-y-value 0))
    (dolist (d (send tbl :data))
      (multiple-value-bind
       (y-value) (send tbl :get-columns-values (list y-label) d)
       (if (> y-value prev-y-value)
           (progn
             (send new-table :add d)
             (setq prev-y-value y-value)
             )
           )))
    (format t "data is decreased ~A => ~A~%" (send tbl :data-num) (send new-table :data-num))
    new-table))

