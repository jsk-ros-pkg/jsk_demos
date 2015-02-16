(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defun set-debri-environment
  (&key (debri-r 40) (debri-h 1000)
        (debri-coords (make-coords :pos #f(500 -100 0) :rpy (list 0 0 0.3)))
        (handle-ratio 0.8)
        (generate-ri? t) (draw? t))
  ;; generate robot
  (unless (boundp '*robot*)
    (cond (generate-ri?
           (hrp2jsknts-init))
          (t
           (setq *robot* (instance hrp2jsknts-robot :init))))
    (setq *robot* *hrp2jsknts*))
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; generate debri
  (setq *debri* (make-cylinder debri-r debri-h))
  (send *debri* :set-color #f(1 1 0))
  ;; atattch handle to debri
  (send *debri* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos (float-vector 0 0 (* debri-h handle-ratio)) :rpy (list 0 0 0))
         :parent *debri*))
  ;; place debri to debri-coords
  (send *debri* :newcoords debri-coords)
  (send *debri* :translate (float-vector 0 0 (* -0.5 debri-h)) :local) ;; treat the difference of origin  between eus and recognition program
  ;; draw
  (when draw?
    (setq *ground* (make-cube 5000 5000 1))
    (objects (list *robot* *debri* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background #f(1 1 1))
    )
  )

(defun set-debri-environment-for-fc
  (debri-r
   debri-h
   debri-coords
   handle-ratio
   &key
   (draw? nil))
  (let* (debri-worldcoords)
    ;; place robot and debri to initial coords
    (set-debri-environment :debri-r debri-r :debri-h debri-h :draw? draw?)
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *debri* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform debri-coords :local))
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun generate-debri-motion
  (&key (draw? t))
  ;; loop
  (let* ((step-dist 100)
         (max-dist 1000)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; pull debri
    (dotimes (i (round (/ max-dist step-dist)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      ;; solve ik
      (let* (ik-ret
             (tc nil))
        ;; solve ik for debri handle
        (when draw? (send (send *debri* :get :grasp-coords) :draw-on :flush t))
        (setq tc
              (list
               (send *debri* :get :grasp-coords)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (with-move-target-link-list
         (mt ll *robot* '(:rarm :rleg :lleg))
         (setq ik-ret
               (send *robot*
                     :fullbody-inverse-kinematics tc
                     ;;:look-at-target (send *debri* :worldcoords)
                     :move-target mt :link-list ll
                     :additional-weight-list
                     (list (list (send *robot* :rleg :toe-p :child-link) 0)
                           (list (send *robot* :lleg :toe-p :child-link) 0))
                     :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5)
                     :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                     ;;:centroid-offset-func #'(lambda () (send *robot* :calc-static-balance-point (list #f(0 0 0) #f(0 0 0)))) 
                     ;;translation-axis can reduced translation constraint                                                              
                     :translation-axis (list t t t)
                     ;; rotation-axis can reduce rotation constraint                                                                    
                     :rotation-axis (list :z t t)
                     :stop 10
                     :debug-view nil
                     ))
         (when ik-ret
           (setq ik-ret
                 (send *robot* :look-at-target (send *debri* :worldcoords))))
         )
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        (push (list i (not (null ik-ret)) ik-ret) ik-results)
        )
      (setq *debri-ik-results* (reverse ik-results))
      ;; pull debri
      (send *debri* :translate (float-vector 0 0 step-dist) :local)
      )
    ))

(defun get-debri-motion
  ()
  (let* (angle-vector-ret)
    (dolist (result *debri-ik-results*)
      ;; break if ik result is fail
      (unless (elt result 1) (return-from nil nil))
      (push (elt result 2) angle-vector-ret)
      )
    (setq angle-vector-ret (reverse angle-vector-ret))
    (setq *debri-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-debri-motion-for-fc
  (debri-r debri-h debri-coords handle-ratio)
  (set-debri-environment-for-fc debri-r debri-h debri-coords handle-ratio :draw? nil)
  (generate-debri-motion :draw? nil)
  (get-debri-motion)
  )

(defun get-debri-motion-for-ocs
  (debri-r debri-h debri-coords handle-ratio)
  ;; TODO : robot-interface is unavailable in ocs
  (set-debri-environment-for-fc debri-r debri-h debri-coords handle-ratio :draw? nil)
  (generate-debri-motion :draw? nil)
  (get-debri-motion)
  )

(defun send-debri-motion
  (coords radius height handle-ratio &key (real? t))
  (let* ((debri-motion-av-list (get-debri-motion-for-fc radius height coords handle-ratio))
         (debri-motion-length (length debri-motion-av-list)))
    (when (not (= debri-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :reset-manip-pose)
      (when real? (model2real :time 3000))
      (when real? (send *ri* :start-impedance :arms :moment-gain #f(0 0 0) :k-p 1000 :d-p 400)) ;; start imp
      ;; pre grasp pose
      (send *robot* :angle-vector (car debri-motion-av-list))
      (send *robot* :rarm :move-end-pos #f(-100 -100 0) :local)
      (when real? (model2real :time 5000))
      ;; grasp
      (send *robot* :angle-vector (car debri-motion-av-list))
      (when real? (model2real :time 3000))
      (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
      (when real? (hand-model2real))
      ;; pull
      (when real?
        (send *ri* :angle-vector-sequence debri-motion-av-list (make-list debri-motion-length :initial-element 2000))
        (send *ri* :wait-interpolation))
      ;; relese
      (send *robot* :angle-vector
            #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 60.0 -10.7092 -87.7993 -121.883 -54.9239 -34.0184 16.2622 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)) ;; debri-relese-pose
      (when real? (model2real :time 6000))
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      ;; reset-manip-pose
      (send *robot* :reset-manip-pose)
      (when real? (model2real :time 5000))
      ))
  )

(defun publish-debri-motion
  (&key coords radius height handle-ratio (step-time 400) (update? t))
  (when update?
    (get-debri-motion-for-ocs radius height coords handle-ratio))
  (dolist (av *debri-motion*)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    ))

;;(publish-debri-motion :coords (make-coords :pos #f(500 -100 -400) :rpy (list 0 0 0.3)) :radius 10 :height 1000 :update? t)

