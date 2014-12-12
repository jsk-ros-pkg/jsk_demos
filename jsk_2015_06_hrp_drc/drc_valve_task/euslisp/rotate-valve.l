#!/usr/bin/env roseus

(load "package://drc_valve_task/euslisp/solve-ik-to-rotate-valve.l")
(load "package://drc_valve_task/euslisp/variable-valve-model.l")
(load "package://drc_valve_task/euslisp/valve-task-util.l")

(ros::roseus "rotate_valve")
(ros::roseus-add-msgs "sensor_msgs")



(defun init-get-valve
  (&key (real nil))
  (setq *real* real)
  (cond (*real*
         (hrp2jsknts-init)
         (setq *robot* *hrp2jsknts*))
        (t
         (setq *robot* (instance hrp2jsknts-robot :init))))
  (setq *valve-thick* 10)
  (setq *valve-radius* 100)
  (setq *robot-coords* (make-coords))
  (setq *valve-coords* (make-coords))
  (setq *valve* (instance variable-valve :init :thick *valve-thick* :radius *valve-radius*))
  (objects (list *robot* *valve*))
  (send *irtviewer* :change-background #f(0.4 0.7 0.5))
  (send *irtviewer* :draw-objects)
  (setq *initial-valve-angle* 0)
  (ros::subscribe "/urdf_control_marker/pose" geometry_msgs::PoseStamped #'robot-pose-cb)
  (ros::subscribe "/valve_model_pose" geometry_msgs::PoseStamped #'valve-pose-cb)
  (ros::subscribe "/valve_model_thick" std_msgs::Float32 #'valve-model-thick-cb)
  (ros::subscribe "/valve_model_radius" std_msgs::Float32 #'valve-model-radius-cb)
  (ros::subscribe "/update_eus_model" std_msgs::bool #'update-eus-model-cb)
)

(defun main-get-valve
  (&key (real nil))
  (setq *real* real)
  (warning-message 2 "====================================~%")
  (warning-message 2 "Push rec button to update eus model.~%")
  (while t
    (x::window-main-one)
    (main-get-valve-one))
  )

(defun main-get-valve-one
  ()
  (ros::spin-once))

(defun get-valve-coords
  ()
  (let* (valve-coords)
    (while t
      (setq valve-coords (send *valve-coords* :transformation *robot-coords* :local))
      (when valve-coords
        (return-from nil nil)))
    (setq valve-coords (send valve-coords :transformation (make-coords) :local))
    valve-coords))

(defun valve-model-thick-cb
  (msg)
  (setq *valve-thick* (send msg :data))
  )

(defun valve-model-radius-cb
  (msg)
  (setq *valve-radius* (send msg :data))
  )

(defun robot-pose-cb
  (msg)
  (setq *robot-coords* (ros::tf-pose-stamped->coords msg))
  )

(defun valve-pose-cb
  (msg)
  (setq *valve-coords* (ros::tf-pose-stamped->coords msg))
  )

(defun update-eus-model-cb
  (msg)
  (setq *valve* (instance variable-valve :init :thick *valve-thick* :radius *valve-radius*))
  (objects (list *valve* *robot*))
  (send *valve* :newcoords (send (get-valve-coords) :copy-worldcoords))
  ;;(send *valve* :rotate pi :x :local)
  (set-initial-valve-angle)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *irtviewer* :draw-objects)
  (let* ((motion-ret-list
          (gen-motion-large-valve-rotation
           :start -35 :end 35 :offset *initial-valve-angle*
           :target-pos-offset (float-vector -50 0 0) :target-yaw-offset (deg2rad -20)
           :interval 10 :time-interval 500
           :valve-object *valve* :real nil)))
    (warning-message 2 "send to real robot?~%")
    (setq *real* (y-or-n-p-org))
    (while t
      (dotimes (i (length motion-ret-list))
        (let* ((motion-ret (elt motion-ret-list i))
               av int-time
               (first-int-time 2000))
          ;; pre motion optional command
          (warning-message 2 "* ~a loop~%" i)
          (cond
           ;; fist time
           ((= i 0)
            ;; start impedance
            (send *ri* :start-impedance :rarm :moment-gain #f(0 0 0) :k-p 1000 :d-p 400)
            ;; first-pre-hand-pose-1
            (warning-message 2 "* first-pre-hand-pose-1~%")
            (send (send *robot* :hand :rarm) :angle-vector #f(90.0 90.0 0.0 90.0 -20.0 0.0))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :hand-angle-vector
                    (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 500)
              (send *ri* :hand-wait-interpolation))
            ;; first-pre-pose-1
            (setq av (elt motion-ret 1))
            (send *robot* :angle-vector av)
            (warning-message 2 "* first-pre-pose-1~%")
            (send *robot* :rarm :move-end-pos (float-vector -60 -60 0) :local)
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :angle-vector (send *robot* :angle-vector) first-int-time)
              (send *ri* :wait-interpolation))
            ;; first-pre-reach-until-touch
            (warning-message 2 "* reach-until-touch~%")
            (when *real*
              (send *ri* :reach-until-touch :rarm #f(1 1 0) :coords-system :local :thre 10))
            ;; first-pre-hand-pose-2
            (warning-message 2 "* first-pre-hand-pose-2~%")
            (send (send *robot* :hand :rarm) :angle-vector #f(77.9709 -11.4732 8.28742 90.0 0.0 0.0))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :hand-angle-vector
                    (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 500)
              (send *ri* :hand-wait-interpolation))
            ;; first-pre-hand-pose-3
            (warning-message 2 "* first-pre-hand-pose-3~%")
            (send (send *robot* :hand :rarm) :angle-vector #f(77.9709 -11.4732 8.28742 90.0 0.0 100))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :hand-angle-vector
                    (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 500)
              (send *ri* :hand-wait-interpolation))
            )
           )
          (cond
           ((= i 0)
            (setq int-time first-int-time))
           (t
            (setq int-time (- (car (last motion-ret))
                              (car (last (elt motion-ret-list (- i 1))))))))

          ;; send angle vector
          (setq av (elt motion-ret 1))
          (send *robot* :fix-leg-to-coords (make-coords))
          ;;(warn "(send *ri* :angle-vector ~a ~a)~%" av int-time)
          (send *robot* :angle-vector av)
          (send *irtviewer* :draw-objects)
          (when *real*
            (send *ri* :angle-vector (send *robot* :angle-vector) int-time)
            ;;(send *ri* :wait-interpolation)
            (unix::usleep (round (* (* int-time 1000) 0.8)))
            )
          ;; post motion optional command
          (cond 
           ;; last time
           ((= i (- (length motion-ret-list) 1))
            ;; last-post-hand-pose-1
            (warning-message 2 "* last-post-hand-pose-1~%")
            (send (send *robot* :hand :rarm) :angle-vector #f(90.0 90.0 0.0 90.0 -20.0 0.0))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :hand-angle-vector
                    (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 500)
              (send *ri* :hand-wait-interpolation))
            ;; last-post-pose-1
            (warning-message 2 "* last-post-pose-1~%")
            (dotimes (i 5)
              (unless (send *robot* :rarm :move-end-pos (float-vector -5 -20 0) :world :rotation-axis nil)
                (return-from nil nil)))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
              (send *ri* :wait-interpolation))
            ;; last-post-pose-2
            (warning-message 2 "* last-post-pose-2~%")
            (dotimes (i 5)
              (unless (send *robot* :rarm :move-end-pos (float-vector -5 -10 0) :world :rotation-axis nil)
                (return-from nil nil)))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :angle-vector (send *robot* :angle-vector) 2500)
              (send *ri* :wait-interpolation))
            ;; last-post-pose-3
            (warning-message 2 "* last-post-pose-3~%")
            (dotimes (i 5)
              (unless (send *robot* :rarm :move-end-pos (float-vector -2.5 -2.5 -10) :world :rotation-axis nil)
                (return-from nil nil)))
            (send *irtviewer* :draw-objects)
            (when *real*
              (send *ri* :angle-vector (send *robot* :angle-vector) 2500)
              (send *ri* :wait-interpolation))
            ;; last-post-pose-3
            (warning-message 2 "* last-post-pose-4~%")
            (reach-valve-pose :real *real* :int-time 3000)
            )
           )))
      (unless *real* (return-from nil nil))
      (warning-message 2 "rotate valve one more?~%")
      (unless (y-or-n-p-org)
        (return-from nil nil))
      )
    )
  (warning-message 2 "Finished~%")
  (warning-message 2 "====================================~%")
  (warning-message 2 "Push rec button to update eus model.~%")
  )

(defun set-initial-valve-angle
  ()
  (let* ((search-num 500)
         serch-vavlve-dir-res)
    (dotimes (i search-num)
      (let* ((valve-angle (* i (/ 360.0 search-num))))
        (send (send *valve* :joint :crank-joint) :joint-angle valve-angle)
        (push
         (list valve-angle
               (elt (send (send *valve* :handle-valve-handle) :worldpos) 2))
         serch-vavlve-dir-res)
        ))
    (let* ((angle-z-res
            (sort serch-vavlve-dir-res #'(lambda (x y) (<= (elt x 1) (elt y 1)))))
           (valve-angle (car (elt angle-z-res 0))))
      (setq *initial-valve-angle* (+ valve-angle 90))
      )
    ))

(init-get-valve :real t)
(main-get-valve :real t)

