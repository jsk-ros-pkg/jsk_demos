(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/angle-vector-sequence.l")

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
  :slots (robot trajectory ik-type move-target col-parameter))

(defmethod ik-solver
  (:init (r aarm &key
            ((:move-target amove-target) nil)
            ((:ik-type aik-type) :arm)
            ((:trajectory traj))
            (collision-parameter))
    (setq robot r)
    (setq arm aarm)
    (if (null amove-target)
        (setq move-target (send r aarm :end-coords))
      (setq move-target amove-target))
    (setq ik-type aik-type)
    (setq trajectory traj)
    (setq col-parameter collision-parameter)
    ;;(setq trajectory traj)
    self)
  (:link-list ()
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
          (t
           (error "unknown ik-type: ~A" ik-type)))
    )
  (:solve-normal-ik (&key via-coords
                          collision-num
                          stop
                          debug-view)
    (when debug-view                    ;change link-list color
      (mapcar #'(lambda (c)
                  (send c :set-color (float-vector 0 1 0) nil))
              (flatten (send-all (send self :link-list) :bodies))))
    (let ((avs (instance angle-vector-sequence :init)))
      (dolist (c via-coords)
        (let ((av (send robot :inverse-kinematics c
                        :link-list (send self :link-list)
                        :move-target move-target
                        :stop stop
                        :avoid-collision-distance 100
                        :avoid-collision-null-gain 5.0
                        :avoid-collision-joint-gain 0.8
                        :collision-avoidance-link-pair
                        (send col-parameter :collision-avoidance-link-pair
                              (send self :link-list)
                              collision-num)
                        :additional-weight-list
                        (list (list (send robot :torso :waist-p :child-link) 0.1)
                            (list (send robot :torso :waist-y :child-link) 0.3)
                            (list (send robot :torso :waist-r :child-link) 0.0))
                        :debug-view (if debug-view :no-message nil))))
          (if av (send avs :add av)
            (return-from :solve-normal-ik nil))))
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
                      :additional-weight-list
                      (list (list (send robot :torso :waist-p :child-link) 0.1)
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
  (:solve (&key (trajectory-resolution 0)
                (collision-num 0)
                (stop 50)
                (debug-view nil))
    (unwind-protect
        (let ((via-coords (send trajectory :via-coords trajectory-resolution)))
          (cond ((or (eq ik-type :arm)
                     (eq ik-type :use-torso))
                 (send self :solve-normal-ik :via-coords via-coords
                       :collision-num collision-num
                       :stop stop
                       :debug-view debug-view))
                ((eq ik-type :fullbody)
                 (send self :solve-fullbody-ik
                       :via-coords via-coords
                       :stop stop
                       :collision-num collision-num
                       :debug-view debug-view))
                (t
                 (error "not supported"))
                ))
      (when debug-view
        (send-all (send robot :bodies) :set-color nil nil))
      )
    )
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

;; (load "package://drc_task_common/euslisp/lib/ik-solver.l")
