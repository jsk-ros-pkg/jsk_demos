(load "package://drc_task_common/euslisp/test-drc-door-task.l")
(load "package://drc_task_common/euslisp/lib/periodic-rate.l")
(load "package://drc_task_common/euslisp/lib/util.l")
(load "package://drc_task_common/euslisp/lib/trajectory.l")
(ros::roseus-add-msgs "geometry_msgs")

(defun move-door-model-from-knob-coords (door origin->recog robot-coords)
  (send door :newcoords (make-coords))  ;force to reset
  (let* ((origin->door (send door :copy-worldcoords))
         (origin->knob (send (send door :handle :knob-handle) :copy-worldcoords))
         (door->knob (send origin->door :transformation origin->knob))
         (door->knob* (send (send (send door->knob :copy-worldcoords) :rotate -pi/2 :x) :rotate 0 :z))
         (knob*->door (send door->knob* :inverse-transformation))
         (origin->knob* (send (send origin->door :copy-worldcoords) :transform door->knob*))
         (origin->recog-pos (zset (send origin->recog :worldpos) (elt (send origin->knob* :worldpos) 2)))
         (origin->recog-yaw (acos (v. 
                                   (normalize-vector 
                                    (z0 (send origin->recog :rotate-vector (float-vector 0 1 0))))
                                   (float-vector 0 -1 0))))
         (normalized-knob-coords (make-coords :pos origin->recog-pos)))
    (send normalized-knob-coords :rotate origin->recog-yaw :z :local)
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
               (send msg :header :stamp))))
    (send *robot* :fix-leg-to-coords origin->ground)
    (when origin->msg
      (send *door* :newcoords (make-coords))
      (let ((traj (door-manipulation-trajectory *door*)))
        (let ((origin->recog (send origin->msg :transform (ros::tf-pose-stamped->coords msg))))
          (send *recog-result* :newcoords (send origin->recog :copy-worldcoords))
          (move-door-model-from-knob-coords *door* origin->recog origin->ground)
          (let ((marker (send traj :to-marker "odom")))
            (ros::publish "trajectory_marker" marker)
            ))
        ))
    )
  ;; test ik-solver
  (setup-move-targets *robot*)
  
  (send *irtviewer* :draw-objects)
  (x::window-main-one))

(defun test-door-recognition ()
  (ros::roseus "test_door_recognition")
  (setq *tf-listener* (instance ros::transform-listener :init))
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (require "models/arrow-object")
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *recog-result* (instance arrow-object :init))
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (objects (list *door* *robot* *recog-result*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::subscribe "/door_detection/door_handle" 
    geometry_msgs::PoseStamped #'test-door-recognition-callback)
  (ros::spin)
  )

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

(defun door-manipulation-trajectory (door-model)
  (let* ((door-tmp (instance param-door :init 700 60 100 20
                             (float-vector 0 -640 950)
                             (float-vector -60 -600 950)
                             :use-leg t
                             :handle-l/r :right))
         (knob-coords (send door-tmp :handle :knob-handle)))
    (instance manipulation-trajectory :init
              :via-coords
              (list (make-coords-relative knob-coords :pos (float-vector -100 200 -100) :rpy (float-vector 0 0 0))
                    (make-coords-relative knob-coords :pos (float-vector 0 50 -50) :rpy (float-vector 0 0 0))
                    (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                    (progn
                      (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                    (progn
                      (send (send door-tmp :joint :door-knob-joint) :joint-angle -70)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                      )
                    (progn
                      (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                    (progn
                      (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                    (progn
                      (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                    (progn
                      (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                      (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                    (make-coords-relative knob-coords :pos (float-vector 0 75 50) :rpy (float-vector 0 0 0))
                    (make-coords-relative knob-coords :pos (float-vector 0 150 50):rpy (float-vector 0 0 0))
                    (make-coords-relative knob-coords :pos (float-vector -100 150 50) :rpy (float-vector 0 0 0)))
              :parent-object (send door-model :handle :knob-handle))))

;; (load "package://drc_task_common/euslisp/lib/door.l")
;; (test-move-door-model-from-knob-coords)
;; (test-door-recognition)
