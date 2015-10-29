(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "package://drc_task_common/euslisp/irex-large-box/apc-box-model.l")

(defun robot-apc-box-init (&key (box-pos (float-vector 800 0 0)))
  (jaxon-init)
  (setq *robot* *jaxon*)
  (make-apc-box)
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *apc-box* :newcoords (make-coords :pos box-pos))
  (objects (list *robot* *apc-box*))
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector)) ?~%")
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  (warn ";; (send *ri* :start-auto-balancer) ?~%")
  (when (y-or-n-p)
    (send *ri* :start-auto-balancer)
    )
  (warn ";; (send *ri* :start-st) ?~%")
  (when (y-or-n-p)
    (send *ri* :start-st)
    )
  (warn ";; (send *ri* :start-impedance :arms) ?~%")
  (when (y-or-n-p)
    (send *ri* :start-impedance :arms)
    ;;(send *ri* :start-impedance :arms :k-p 3000 :d-p 600)
    )
  )

(defun reach-apc-box ()
  ;; move to pose-1
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         (send (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords) :translate (float-vector -200 0 200) :world)
         (send (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords) :translate (float-vector -200 0 200) :world)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :translation-axis (list t t t t)
        :rotation-axis (list :z :z t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  (send *irtviewer* :draw-objects)
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector)) ?~%")
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation)
    )

  (warn ";; move to pose-reach-end ?~%")
  (unless (y-or-n-p)
    (return-from reach-apc-box)
    )

  ;; move to pose-reach-end
  (send *robot*
        :fullbody-inverse-kinematics
        (list
         (send (send *apc-box* :top-front-right-grasp-edge) :copy-worldcoords)
         (send (send *apc-box* :top-front-left-grasp-edge) :copy-worldcoords)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords))
        :move-target
        (list
         (send *robot* :rarm :end-coords)
         (send *robot* :larm :end-coords)
         (send *robot* :rleg :end-coords)
         (send *robot* :lleg :end-coords))
        :link-list
        (mapcar #'(lambda (limb)
                    (send *robot* :link-list (send limb :parent)))
                (list
                 (send *robot* :rarm :end-coords)
                 (send *robot* :larm :end-coords)
                 (send *robot* :rleg :end-coords)
                 (send *robot* :lleg :end-coords))
                )
        :translation-axis (list t t t t)
        :rotation-axis (list :z :z t t)
        :target-centroid-pos (send (send *robot* :foot-midcoords) :worldpos)
        :debug-view :no-message)
  (send *irtviewer* :draw-objects)
  (warn ";; (send *ri* :angle-vector (send *robot* :angle-vector)) ?~%")
  (when (y-or-n-p)
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  )

;; (defun torque-ratio-vector ()
;;   (let* ()
;;     (setq torque-vector
;;           (send *robot* :torque-vector
;;                 :force-list (list #f(0 0 700) #f(0 0 700) #f(0 0 100) #f(0 0 100))
;;                 :moment-list (list #f(0 0 0) #f(0 0 0) #f(0 0 0) #f(0 0 0))
;;                 :target-coords (append (send *robot* :legs :end-coords) (send *robot* :arms :end-coords))))
;;     (send *robot* :torque-ratio-vector :torque torque-vector)
;;     ))

(warn "(robot-apc-box-init)~%")
(warn "(reach-apc-box)~%")
