(load "package://drc_task_common/euslisp/robot-util.l")

;;(generate-robot "HRP2JSKNT" :generate-ri? t)

(defun get-eef-trans-diff-with-ref
  (arm &key (update? t))
  (when update? (send *ri* :state))
  (let* ((ref-vector (send *ri* :reference-vector))
         ref-eef-coords)
    (send *robot* :angle-vector ref-vector)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq ref-eef-coords (send (send *robot* arm :end-coords) :copy-worldcoords))
    (get-eef-trans-diff arm ref-eef-coords :update? nil)
    ))

(defun get-eef-trans-diff
  (arm coords &key (update? t))
  (when update? (send *ri* :state))
  (let* ((act-vector (send *ri* :potentio-vector))
         act-eef-coords)
    (send *robot* :angle-vector act-vector)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq act-eef-coords (send (send *robot* arm :end-coords) :copy-worldcoords))
    (send (send coords :transformation act-eef-coords) :copy-worldcoords)
    ))

(defun send-add-force-motion
  (arm dir &key (real? nil) (rotation-axis t) (ref-force-step (scale -5 (normalize-vector dir))))
  (let* (initial-eef-coords)
    ;; save initial eef coords
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq initial-eef-coords (send (send *robot* arm :end-coords) :copy-worldcoords))
    ;; start impedance
    (when real?
      (send *ri* :set-impedance-controller-param arm :k-p 200 :d-p 180 :m-p 5 :m-r 1 :d-r 15 :k-r 20 :force-gain #f(0.3 0.3 1))
      (send *ri* :start-impedance arm))
    ;; loop
    (while t
      ;; check the eef position following
      (while t
        (let* ((dir-diff (v. (send (get-eef-trans-diff-with-ref arm :update? t) :pos) (normalize-vector dir)))
               (dir-diff-thre 50)
               (ref-force-time 2000)
               ref-force)
          (when (> dir-diff (- dir-diff-thre))
            (return-from nil nil))
          (ros::ros-info "[add-force-motion] eef diff is over threshold: ~a < ~a" dir-diff (- dir-diff-thre))
          (unless ref-force
            (setq ref-force (send *ri* :state :reference-force-vector arm)))
          (ros::ros-info "[add-force-motion] add ref-force: ~a -> ~a" ref-force (v+ ref-force ref-force-step))
          (setq ref-force (v+ ref-force ref-force-step))
          (when real?
            (send *ri* :set-ref-force ref-force ref-force-time arm)
            (unix::usleep (* ref-force-time 1000)))
          )
        )
      ;; move robot
      (let* ((move-step 30)
             (angle-vector-time 2000))
        (send *ri* :state)
        (send *robot* :angle-vector (send *ri* :reference-vector))
        ;; (let* ((tmp-robot (copy-object *robot*)))
        ;;   (send tmp-robot :angle-vector (send *ri* :potentio-vector))
        ;;   (send *robot* arm :angle-vector (send tmp-robot arm :angle-vector))
        ;;   )
        (send *robot* :fix-leg-to-coords (make-coords))
        (let* (ik-ret)
          (setq ik-ret
                (send *robot* arm :move-end-pos (scale move-step (normalize-vector dir)) :world :rotation-axis rotation-axis))
          (cond (ik-ret
                 (ros::ros-info "[add-force-motion] move robot ~a ~a (rotation ~a)" arm (scale move-step (normalize-vector dir)) rotation-axis))
                (t
                 (ros::ros-info "[add-force-motion] ik failed")
                 (return-from nil nil)))
          (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
        (when real?
          (model2real :time angle-vector-time :wait? nil :enable-cancel? nil)
          (unix::usleep (* angle-vector-time 1000)))
        )
      ;; check if the eef reaches goal
      (let* ((goal-diff (v. (send (get-eef-trans-diff arm initial-eef-coords :update? t) :pos) (normalize-vector dir))))
        (when (> goal-diff (norm dir))
          (ros::ros-info "[add-force-motion] reached the goal: ~a > ~a" goal-diff (norm dir))
          (return-from nil nil))
        )
      )
    )
  )

(defun test-lever-switch-down
  (&key (real? t))
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 -112.89 34.5851 -19.4106 -51.526 100.948 -33.7277 78.0214 -60.0))
  (when real? (model2real :time 10000 :wait? t))
  (send-add-force-motion :larm #f(0 0 -150) :rotation-axis t :real? real?)
  )

