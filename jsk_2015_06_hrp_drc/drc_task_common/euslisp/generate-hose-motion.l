(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")

;; hose grasp ;;;;

(defun set-hose-grasp-environment
  (&key (hose-coords (make-coords :pos (float-vector 200 500 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t))
  ;; generate robot
  (unless (boundp '*robot*)
    (cond (generate-ri?
           (hrp2jsknts-init)
           (setq *robot* *hrp2jsknts*))
          (t
           (setq *robot* (instance hrp2jsknts-robot :init)))))
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; hose setting
  (let* ((hose-r 30) (hose-h 120))
    ;; generate hose
    (setq *hose* (make-cylinder hose-r hose-h))
    (send *hose* :set-color (float-vector 1 1 0))
    ;; atattch handle to hose
    (send *hose* :put :grasp-coords
          (make-cascoords
           :coords (make-coords :pos (float-vector 0 0 (* hose-h 0.5)))
           :parent *hose*))
    ;; place hose to hose-coords
    (send *hose* :newcoords hose-coords)
    ;; treat the difference of origin between eus and recognition program
    (send *hose* :translate (float-vector 0 0 (* -0.5 hose-h)) :local)
    )
  ;; draw
  (when draw?
    (setq *ground* (make-cube 5000 5000 1))
    (objects (list *robot* *hose* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-hose-grasp-environment-full
  (hose-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (hose-worldcoords)
    ;; place robot and hose to initial coords
    (set-hose-grasp-environment :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *hose* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform hose-coords :local))
    ;; treat the difference of origin between eus and recognition program
    (send *hose* :translate (float-vector 0 0 (* -0.5 (elt (send *hose* :body-type) 2))) :local)
    (send (make-cylinder 100 10) :body-type)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun generate-hose-grasp-motion
  (&key (draw? t))
  (let* ((ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; grasp hose
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to grasp hose
    (let* (ik-ret
           (tc nil))
      ;; solve ik for hose handle
      (when draw? (send (send *hose* :get :grasp-coords) :draw-on :flush t))
      (setq tc
            (list
             (send *hose* :get :grasp-coords)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (with-move-target-link-list
       (mt ll *robot* '(:larm :rleg :lleg))
       (setq ik-ret
             (send *robot*
                   :fullbody-inverse-kinematics tc
                   ;;:look-at-target (send *hose* :worldcoords)
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
                   :stop 100
                   :debug-view nil
                   ))
       (when ik-ret
         (setq ik-ret
               (send *robot* :look-at-target (send *hose* :worldcoords)))))
      (send *robot* :larm :move-end-pos (float-vector -100 100 0) :local :revert-if-fail nil)
      (push (list 0 (not (null ik-ret)) (send *robot* :angle-vector)) ik-results) ;; pre grasp pose
      (push (list 1 (not (null ik-ret)) ik-ret) ik-results) ;; grasp pose
      ;; draw
      (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; hose-reset-pose
      (send *robot* :angle-vector (get-hose-reset-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (push (list 2 t (send *robot* :angle-vector)) ik-results)
      (setq *hose-ik-results* (reverse ik-results))
      ))
  )

(defun get-hose-grasp-motion
  ()
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *hose-ik-results*)))
    (setq *hose-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-hose-grasp-motion-for-fc
  (hose-coords)
  (set-hose-grasp-environment-full hose-coords :draw? nil)
  (generate-hose-grasp-motion :draw? nil)
  (get-hose-grasp-motion)
  )

(defun get-hose-grasp-motion-for-ocs
  (hose-coords)
  (set-hose-grasp-environment-full hose-coords :draw? nil :use-ri? nil)
  (generate-hose-grasp-motion :draw? nil)
  (get-hose-grasp-motion)
  )

(defun send-hose-grasp-motion
  (coords &key (real? t))
  (let* ((hose-motion-av-list (get-hose-grasp-motion-for-fc coords))
         (hose-motion-length (length hose-motion-av-list)))
    (when (not (= hose-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :reset-manip-pose)
      (when real? (model2real :time 3000))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
      ;; pre-reach
      (send *robot* :angle-vector (elt hose-motion-av-list 0))
      (when real? (model2real :time 5000))
      ;; reach
      (send *robot* :angle-vector (elt hose-motion-av-list 1))
      (when real? (model2real :time 3000))
      ;; grasp
      (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
      (when real? (hand-model2real))
      ;; hose-reset-pose
      (send *robot* :angle-vector (elt hose-motion-av-list 2))
      (when real? (model2real :time 5000))
      ))
  )

(defun publish-hose-grasp-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-hose-grasp-motion-for-ocs coords))
  (dolist (av *hose-motion*)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    ))


;; hose connect ;;;;

(defun set-hose-connect-environment
  (&key (hose-coords (make-coords :pos (float-vector 500 0 800) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t))
  ;; generate robot
  (unless (boundp '*robot*)
    (cond (generate-ri?
           (hrp2jsknts-init)
           (setq *robot* *hrp2jsknts*))
          (t
           (setq *robot* (instance hrp2jsknts-robot :init)))))
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; hose setting
  (let* ((hose-r 30) (hose-h 120))
    ;; generate hose
    (setq *hose* (make-cylinder hose-r hose-h))
    (send *hose* :set-color (float-vector 1 1 0))
    ;; atattch handle to hose
    (send *hose* :put :connect-coords
          (make-cascoords
           :coords (make-coords :pos (float-vector 0 0 0))
           :parent *hose*))
    ;; place hose to hose-coords
    (send *hose* :newcoords hose-coords)
    ;; treat the difference of origin between eus and recognition program
    (send *hose* :translate (float-vector 0 0 (* -0.5 hose-h)) :local)
    )
  ;; draw
  (when draw?
    (setq *ground* (make-cube 5000 5000 1))
    (objects (list *robot* *hose* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-hose-connect-environment-full
  (hose-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (hose-worldcoords)
    ;; place robot and hose to initial coords
    (set-hose-connect-environment :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *hose* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform hose-coords :local))
    ;; treat the difference of origin between eus and recognition program
    (send *hose* :translate (float-vector 0 0 (* -0.5 (elt (send *hose* :body-type) 2))) :local)
    (send (make-cylinder 100 10) :body-type)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun generate-hose-connect-motion
  (&key (draw? t))
  (let* ((ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (connect-interpolation-num 4)
         (connect-minus-distance -200)
         (connect-plus-distance 10)
         (connect-distance (- connect-plus-distance connect-minus-distance)))
    (when draw? (send (send *hose* :get :connect-coords) :draw-on :flush t))
    ;; loop
    (dotimes (i connect-interpolation-num)
      ;; solve ik to connect hose
      (let* (ik-ret
             (tc nil)
             (connect-interpolation-ratio (/ (float i) connect-interpolation-num))
             (connect-offset (+ (* connect-distance connect-interpolation-ratio) connect-minus-distance)))
        ;; solve ik for hose connector handle
        (setq tc
              (list
               (send (send (send *hose* :get :connect-coords) :copy-worldcoords) :translate (float-vector 0 0 connect-offset) :local)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (with-move-target-link-list
         (mt ll *robot* '(:larm :rleg :lleg))
         (setq ik-ret
               (send *robot*
                     :fullbody-inverse-kinematics tc
                     ;;:look-at-target (send *hose* :worldcoords)
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
                     :stop 100
                     :debug-view nil
                     ))
         (when ik-ret
           (setq ik-ret
                 (send *robot* :look-at-target (send *hose* :worldcoords)))))
        (push (list i (not (null ik-ret)) ik-ret) ik-results) ;; connect pose
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        ))
    (setq *hose-ik-results* (reverse ik-results)))
  )

(defun get-hose-connect-motion
  ()
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*))
      ;; hose-reset-pose
      (push (send *robot* :reset-manip-pose) angle-vector-ret)
      (dolist (av (mapcar #'(lambda (x) (elt x 2)) *hose-ik-results*))
        (push av angle-vector-ret))
      (dolist (av (reverse (mapcar #'(lambda (x) (elt x 2)) *hose-ik-results*)))
        (push av angle-vector-ret))
      (push (get-hose-reset-pose) angle-vector-ret)
      )
    (setq *hose-motion* angle-vector-ret)
    angle-vector-ret))

(defun get-hose-connect-motion-for-fc
  (hose-coords)
  (set-hose-connect-environment-full hose-coords :draw? nil)
  (generate-hose-connect-motion :draw? nil)
  (get-hose-connect-motion)
  )

(defun get-hose-connect-motion-for-ocs
  (hose-coords)
  (set-hose-connect-environment-full hose-coords :draw? nil :use-ri? nil)
  (generate-hose-connect-motion :draw? nil)
  (get-hose-connect-motion)
  )

(defun send-hose-connect-motion
  (coords &key (real? t))
  (let* ((hose-motion-av-list (get-hose-connect-motion-for-fc coords))
         (hose-motion-length (length hose-motion-av-list)))
    (when (not (= hose-motion-length 0))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
      ;; hose-reset-pose
      (send *robot* :angle-vector (elt hose-motion-av-list 0))
      (when real? (model2real :time 5000))
      ;; connect
      (when real?
        (send *ri* :angle-vector-sequence (subseq *hose-motion* 1 (/ hose-motion-length 2)) (make-list (1- (/ hose-motion-length 2)) :initial-element 2000))
        (send *ri* :wait-interpolation))
      ;; release
      (when real?
        (send *ri* :angle-vector-sequence (subseq *hose-motion* (/ hose-motion-length 2) (1- hose-motion-length)) (make-list (1- (/ hose-motion-length 2)) :initial-element 2000))
        (send *ri* :wait-interpolation))
      ;; reset-manip-pose
      (send *robot* :angle-vector (elt hose-motion-av-list (1- hose-motion-length)))
      (when real? (model2real :time 5000))
      ))
  )

(defun publish-hose-connect-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-hose-connect-motion-for-ocs coords))
  (dolist (av *hose-motion*)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    ))


;; hose util ;;;;

(defun get-hose-reset-pose ()
  (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 -11.8504 28.6898 -41.2121 -109.707 -61.224 -14.2529 43.9426 -60.0))

