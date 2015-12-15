(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/angle-vector-sequence.l")

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
  (:solve (&key (trajectory-resolution 0)
                (collision-num 0)
                (debug-view nil))
    (when debug-view                    ;change link-list color
      (mapcar #'(lambda (c)
                  (send c :set-color (float-vector 0 1 0) nil))
              (flatten (send-all (send self :link-list) :bodies)))
      )
    (unwind-protect
        (let ((via-coords (send trajectory :via-coords trajectory-resolution)))
          (cond ((or (eq ik-type :arm)
                     (eq ik-type :use-torso))
                 ;; all the ik should be success
                 (let ((avs (instance angle-vector-sequence :init)))
                   (dolist (c via-coords)
                     (let ((av (send robot :inverse-kinematics c
                                     :link-list (send self :link-list)
                                     :move-target move-target
                                     :avoid-collision-distance 100
                                     :avoid-collision-null-gain 5.0
                                     :avoid-collision-joint-gain 0.8
                                     :collision-avoidance-link-pair
                                     (send col-parameter :collision-avoidance-link-pair
                                           (send self :link-list)
                                           collision-num)
                                     :debug-view (if debug-view :no-message nil))))
                       (if av (send avs :add av)
                         (return-from :solve nil))))))
                (t
                 (error "not supported"))
                ))
      (progn
        (when debug-view                    ;change link-list color
          (mapcar #'(lambda (c)
                      (send c :set-color nil nil))
                  (flatten (send-all (send self :link-list) :bodies)))
          )
        )
      ))
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

;; (load "package://drc_task_common/euslisp/lib/ik-solver.l")
