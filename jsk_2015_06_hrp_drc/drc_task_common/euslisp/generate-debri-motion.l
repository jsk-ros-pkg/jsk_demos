(require "package://drc_task_common/euslisp/robot-util.l")
(require "package://drc_task_common/euslisp/util.l")

(defun set-debri-environment
  (&key (debri-r 40) (debri-h 1000)
        (debri-coords (make-coords :pos (float-vector 0 -100 300) :rpy (list 0 0 0.3)))
        (handle-ratio 0.8)
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; attach handle to robot
  (send *robot* :put :larm-debri-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 5.0 0.0) :local)
         :parent (send (send *robot* :larm :end-coords) :parent)))
  (send *robot* :put :rarm-debri-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 -5.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  ;; generate debri
  (setq *debri* (make-cylinder debri-r debri-h))
  (send *debri* :set-color (float-vector 1 1 0))
  ;; atattch handle to debri
  (send *debri* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos (float-vector 0 0 (* debri-h handle-ratio)) :rpy (list 0 0 0))
         :parent *debri*))
    ;; place debri to debri-coords
  (when debri-coords
    (send *debri* :newcoords debri-coords)
    ;; treat the difference of origin  between eus and recognition program
    (send *debri* :translate (float-vector 0 0 (* -0.5 debri-h)) :local))
  (setq *debri-stand-coords* nil)
  ;; draw
  (when draw?
    (setq *ground* (make-cube 5000 5000 1))
    (objects (list *robot* *debri* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-debri-environment-full
  (debri-r
   debri-h
   debri-coords ;; body relative
   handle-ratio
   &key
   (draw? nil)
   (use-ri? t))
  (let* (debri-worldcoords)
    ;; place robot and debri to initial coords
    (set-debri-environment :debri-r debri-r :debri-h debri-h :debri-coords nil :handle-ratio handle-ratio :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *debri* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform debri-coords :local))
    ;; treat the difference of origin between eus and recognition program
    (send *debri* :translate (float-vector 0 0 (* -0.5 (elt (send *debri* :body-type) 2))) :local)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-debri
  (tc mt ll &key (draw? nil))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0))
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis (list t t t)
        :rotation-axis (list :z t t)
        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
        :stop 50
        :debug-view (if draw? :no-message nil)
        ))

(defun generate-debri-motion
  (&key (draw? t))
  ;; loop
  (let* ((step-dist 60)
         (max-dist 600)
         (reach-dist 120)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (pull-interpolation-num (round (/ max-dist step-dist))))
    ;; pull debri
    (dotimes (i pull-interpolation-num)
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      ;; solve ik to pull debri
      (let* (ik-ret
             tc)
        (when draw? (send (send *debri* :get :grasp-coords) :draw-on :flush t))
        ;; solve ik for debri handle
        (setq tc
              (list
               (send *debri* :get :grasp-coords)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (with-move-target-link-list
         (mt ll *robot* '(:rarm :rleg :lleg))
         (setf (elt mt 0) (send *robot* :get :rarm-debri-coords))
         (setq ik-ret (fullbody-inverse-kinematics-for-debri tc mt ll :draw? draw?))
         (when ik-ret
           (setq ik-ret (send *robot* :look-at-target (send *debri* :worldcoords)))))
        (push-back (list i (not (null ik-ret)) ik-ret) ik-results)
        ;; solve ik for reaching point
        (when (and (= i 0) ik-ret)
          (setq tc
                (list
                 (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) (- reach-dist) 0) :local)
                 (send *robot* :rleg :end-coords :copy-worldcoords)
                 (send *robot* :lleg :end-coords :copy-worldcoords)))
          (with-move-target-link-list
           (mt ll *robot* '(:rarm :rleg :lleg))
           (setf (elt mt 0) (send *robot* :get :rarm-debri-coords))
           (setq ik-ret (fullbody-inverse-kinematics-for-debri tc mt ll :draw? draw?))
           (when ik-ret
             (setq ik-ret (send *robot* :look-at-target (send *debri* :worldcoords)))))
          (push (list i (not (null ik-ret)) ik-ret) ik-results))
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        )
      (setq *debri-ik-results* ik-results)
      ;; pull debri
      (send *debri* :translate (float-vector 0 0 step-dist) :local)
      )
      (setq *debri-ik-num* pull-interpolation-num)
    ))

(defun get-debri-motion
  (&key (draw? nil))
  (generate-debri-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (and (elt (elt *debri-ik-results* 0) 1) (elt (elt *debri-ik-results* 1) 1))
      ;; reset-manip-pose
      (push-back (send *robot* :reset-manip-pose) angle-vector-ret)
      ;; pull
      (dolist (result *debri-ik-results*)
        ;; break if ik result is fail
        (unless (elt result 1) (return-from nil nil))
        (push-back (elt result 2) angle-vector-ret)
        )
      ;; debri-release-pose
      (push-back (get-debri-release-pose) angle-vector-ret)
      ;; reset-manip-pose
      (push-back (send *robot* :reset-manip-pose) angle-vector-ret)
      )
    (setq *debri-motion* angle-vector-ret)
    *debri-motion*
    ))

(defun get-debri-motion-full
  (debri-r debri-h debri-coords handle-ratio
           &key (draw? nil) (use-ri? t))
  (set-debri-environment-full debri-r debri-h debri-coords handle-ratio :draw? draw? :use-ri? use-ri?)
  (get-debri-motion :draw? draw?)
  )

(defun send-debri-motion
  (coords radius height handle-ratio &key (real? t))
  (let* ((debri-motion-av-list (get-debri-motion-full radius height coords handle-ratio))
         (debri-motion-length (length debri-motion-av-list))
         (debri-ik-num *debri-ik-num*))
    (when (not (= debri-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :angle-vector (elt debri-motion-av-list 0))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 800 :d-p 400)) ;; start imp
      ;; walk
      (when *debri-stand-coords*
        (let* ((pos (scale 0.001 (send *debri-stand-coords* :worldpos)))
               (rpy (mapcar #'rad2deg (car (send *debri-stand-coords* :rpy-angle)))))
          (ros::ros-warn "go-pos to ~a ~a ~a for debri~%" (elt pos 0) (elt pos 1) (elt rpy 0))
          (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
          ))
      ;; pre grasp pose
      (send *robot* :angle-vector (elt debri-motion-av-list 1))
      (when real? (model2real :time 5000))
      ;; grasp
      (send *robot* :angle-vector (elt debri-motion-av-list 2))
      (when real? (model2real :time 3000))
      (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
      (when real? (hand-model2real))
      ;; pull
      (when real? (send-angle-vector-sequence-with-time (cdr (subseq debri-motion-av-list 2 (+ *debri-ik-num* 2))) :time 1000 :wait? t))
      ;; relese
      (send *robot* :angle-vector (elt debri-motion-av-list (- debri-motion-length 2))) ;; debri-relese-pose
      (when real? (model2real :time 6000))
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      ;; reset-manip-pose
      (send *robot* :angle-vector (elt debri-motion-av-list (- debri-motion-length 1)))
      (when real? (model2real :time 4000))
      ))
  )

(defun publish-debri-motion
  (&key coords radius height handle-ratio (step-time 200) (update? t))
  (when update?
    (get-debri-motion-full radius height coords handle-ratio :use-ri? nil))
  (ros::ros-info "debri-stand-coords: ~a~%" *debri-stand-coords*)
  (ros::ros-info "debri-ik-results: ~a~%" *debri-ik-results*)
  (when *debri-stand-coords*
    (move-robot-model *debri-stand-coords* "ground"))
  (publish-motion-joint-states *debri-motion* :step-time step-time)
  )

;; debri util ;;;;

(defun get-debri-release-pose ()
  (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 59.9996 -27.7207 -85.0007 -125.779 16.8005 1.11049 76.5611 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))


