(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/models/dewalt_dc550ka.l")

;; drill grasp ;;;;

(defun set-drill-grasp-environment
  (&key (drill-coords (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNT"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :hand :arms :reset-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  (send *robot* :put :rarm-drill-grasp-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 48.0 12.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  ;; drill setting
  ;; generate drill
  (setq *drill* (dewalt_dc550ka-robot))
  (send-all (send *drill* :bodies) :set-color #f(0.7 0.6 0.0))
  ;; atattch handle to drill
  (send *drill* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 190) :rpy (list 0 pi 0))
         :parent *drill*))
  ;; place drill to drill-coords
  (when drill-coords
    (send *drill* :newcoords drill-coords))
  (setq *drill-grasp-stand-coords* nil)
  ;; (let* ((drill-r 30) (drill-h 120))
  ;;   ;; generate drill
  ;;   (setq *drill* (make-cylinder drill-r drill-h))
  ;;   (send *drill* :set-color (float-vector 1 1 0))
  ;;   ;; atattch handle to drill
  ;;   (send *drill* :put :grasp-coords
  ;;         (make-cascoords
  ;;          :coords (make-coords :pos (float-vector 0 0 (* drill-h 0.5)))
  ;;          :parent *drill*))
  ;;   ;; place drill to drill-coords
  ;;   (when drill-coords
  ;;     (send *drill* :newcoords drill-coords)
  ;;     ;; treat the difference of origin between eus and recognition program
  ;;     (send *drill* :translate (float-vector 0 0 (* -0.5 drill-h)) :local))
  ;;   )
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
    (objects (list *robot* *drill* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-drill-grasp-environment-full
  (drill-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (drill-worldcoords)
    ;; place robot and drill to initial coords
    (set-drill-grasp-environment :drill-coords nil :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform drill-coords :local))
    ;; treat the difference of origin between eus and recognition program
    ;;(send *drill* :translate (float-vector 0 0 (* -0.5 (elt (send *drill* :body-type) 2))) :local)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-drill-grasp
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
        :stop 100
        :debug-view (if draw? :no-message nil)
        )
  )

(defun generate-drill-grasp-motion
  (&key (draw? t))
  (let* ((reach-dist 100)
	 (grasp-up-dist 300)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; grasp drill
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to grasp drill
    (let* (ik-ret
           tc)
      (when draw? (send (send *drill* :get :grasp-coords) :draw-on :flush t))
      ;; solve ik for grasp
      (setq tc
            (list
             ;; todo reverse if
             (send (send *drill* :get :grasp-coords) :copy-worldcoords)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (with-move-target-link-list
       (mt ll *robot* '(:rarm :rleg :lleg))
       (setf (elt mt 0) (send *robot* :get :rarm-drill-grasp-coords))
       (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw?))
       (when ik-ret
         (setq ik-ret
               (send *robot* :look-at-target (send *drill* :worldcoords)))))
      (push-back (list 1 (not (null ik-ret)) ik-ret) ik-results)
      ;; solve ik for pre grasp
      (let ((robot-angle-vector (send *robot* :angle-vector)))
	(setq tc
	      (list
	       (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) (- reach-dist) 0) :local)
		   (send *robot* :rleg :end-coords :copy-worldcoords)
		   (send *robot* :lleg :end-coords :copy-worldcoords)))
	(with-move-target-link-list
	 (mt ll *robot* '(:rarm :rleg :lleg))
	 (setf (elt mt 0) (send *robot* :get :rarm-drill-grasp-coords))
	 (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw?))
	 (when ik-ret
	   (setq ik-ret
		 (send *robot* :look-at-target (send *drill* :worldcoords)))))
	(push (list 0 (not (null ik-ret)) ik-ret) ik-results)
	(send *robot* :angle-vector robot-angle-vector)
	;; solve ik for grasp up	
	(setq tc
	      (list
	       (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 0 grasp-up-dist) :world)
	       (send *robot* :rleg :end-coords :copy-worldcoords)
	       (send *robot* :lleg :end-coords :copy-worldcoords)))
	(with-move-target-link-list
	 (mt ll *robot* '(:rarm :rleg :lleg))
	 (setf (elt mt 0) (send *robot* :get :rarm-drill-grasp-coords))
	 (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw?))
	 (when ik-ret
	   (setq ik-ret
		 (send *robot* :look-at-target (send *drill* :worldcoords)))))
	(push-back (list 2 (not (null ik-ret)) ik-ret) ik-results)
	(send *robot* :angle-vector robot-angle-vector)
	)
      ;; draw
      (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; drill-reset-pose
      (send *robot* :angle-vector (get-look-drill-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (push-back (list 3 t (send *robot* :angle-vector)) ik-results)
      (setq *drill-ik-results* ik-results)
      ))
  )

(defun get-drill-grasp-motion
  (&key (draw? t))
  (generate-drill-grasp-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*)))
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-drill-grasp-stand-coords ;; relative to drill
  ()
  (make-coords :pos (float-vector -600 120 0) :rpy (list (deg2rad 30) 0 0)) ;; dependent on arm
  )

(defun get-drill-grasp-motion-full
  (drill-coords &key (draw? nil) (use-ri? t))
  (set-drill-grasp-environment-full drill-coords :draw? draw? :use-ri? use-ri?)
  (let* ((drill-motion-av-list (get-drill-grasp-motion :draw? draw?))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0)
      ;; (let* ((x-vec (send (send (send *drill* :worldcoords) :copy-worldcoords) :rotate-vector (float-vector 0 0 1)))
      ;;        (z-vec (float-vector 0 0 1))
      ;;        (y-vec (v* z-vec x-vec))
      ;;        (drill-grasp-coords-stand-rot (make-coords :pos (send (send *drill* :copy-worldcoords) :worldpos) :rot (transpose (matrix x-vec y-vec z-vec))))
      ;;        (stand-coords (send (send drill-grasp-coords-stand-rot :copy-worldcoords) :transform (get-drill-grasp-stand-coords) :local))
      ;;        (stand-pos (send stand-coords :worldpos))
      ;;        (stand-rpy (elt (send stand-coords :rpy-angle) 0))
      ;;        )
      ;;   (setq *drill-grasp-stand-coords* (make-coords :pos (float-vector (elt stand-pos 0) (elt stand-pos 1) 0) :rpy (list (elt stand-rpy 0) 0 0))
        (let ((theta (atan2 (elt (send drill-coords :pos) 1)
                            (elt (send drill-coords :pos) 0))))
          (let ((ux (float-vector (cos theta)
                                  (sin theta)
                                  0))
                (uz (float-vector 0 0 1)))
            (let ((uy (v* uz ux)))
              (let ((mat (make-matrix 3 3)))
                (setf (matrix-column mat 0) ux)
                (setf (matrix-column mat 1) uy)
                (setf (matrix-column mat 2) uz)
                (let ((drill-oriented-coords
                       (make-coords :pos (send drill-coords :worldpos)
                                    :rot mat)))

                  (setq *drill-grasp-stand-coords* (send (send drill-oriented-coords :copy-worldcoords)
                                                         :transform (get-drill-grasp-stand-coords)))
                  )))))
      (ros::ros-warn "change stand position to ~a for valve motion.~%" *drill-grasp-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-grasp-stand-coords*)
      (when draw? (send *irtviewer* :draw-objects))
      (setq drill-motion-av-list (get-drill-grasp-motion :draw? draw?))
      )
    drill-motion-av-list
    ))

  

(defun send-drill-grasp-motion
  (coords &key (real? t))
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords))
         (drill-motion-length (length drill-motion-av-list)))
    (when (not (= drill-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :reset-manip-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
    ;; walk
    (when *drill-grasp-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-grasp-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-grasp-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for valve~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))

    ;; pre-reach
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    ;; reach
    (send *robot* :angle-vector (elt drill-motion-av-list 1))
    (when real? (model2real :time 3000))
    ;; grasp
    (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
    (when real? (hand-model2real))
    ;; grasp-up
    (send *robot* :angle-vector (elt drill-motion-av-list 2))
    (when real? (model2real :time 3000))      
    ;; drill-reset-pose
    (send *robot* :angle-vector (elt drill-motion-av-list 3))
    (when real? (model2real :time 5000))
    )
  )

(defun publish-drill-grasp-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-grasp-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (when *drill-grasp-stand-coords*
    (move-robot-model *drill-grasp-stand-coords* "ground"))
  (publish-motion-joint-states *drill-motion* :step-time step-time))

;; drill button ;;;;

(defun set-drill-button-environment
  (&key (drill-coords (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNT"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :hand :arms :reset-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  ;; for hrp2jsknt
  ;; (send *robot* :put :larm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate #f(121 -21 24) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand :larm :link "L_INDEXPIP_R_LINK")))
  ;; for hrp2jsknts
  ;; (send *robot* :put :larm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate #f(120 -23 23) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand :larm :link "L_INDEXPIP_R_LINK")))
  (send *robot* :put :larm-drill-button-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate #f(50 87.5 80) :local)
         ;;:coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate #f(50 62 80) :local)
         :parent (send (send *robot* :larm :end-coords) :parent)))
  ;; drill setting
  ;; generate drill
  (setq *drill* (dewalt_dc550ka-robot))
  (send-all (send *drill* :bodies) :set-color #f(0.7 0.6 0.0))
  ;; atattch handle to drill
  (send *drill* :put :button-coords
        (make-cascoords
         ;; :coords (make-coords :pos #f(-25 30 93) :rpy (list 0 0 pi/2)) ;; center of hole
         :coords (make-coords :pos #f(-22 30 97) :rpy (list 0 0 pi/2)) ;; center of button
         :parent *drill*))
  ;; place drill to drill-coords
  (when drill-coords
    (send *drill* :newcoords drill-coords))
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
    (objects (list *robot* *drill* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-drill-button-environment-full
  (drill-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (drill-worldcoords)
    ;; place robot and drill to initial coords
    (set-drill-button-environment :drill-coords nil :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send (send *robot* :rarm :end-coords) :dissoc *drill*)
    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform drill-coords :local))
    (send (send *robot* :rarm :end-coords) :assoc *drill*)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-drill-button
  (&key (x-offset 0) (y-offset 0) (z-offset 0) (draw? t))
  (let* (ret)
    (with-move-target-link-list
     (mt ll *robot* '(:rarm :larm :rleg :lleg))
     (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
     (setq tc
           (list
            (send *robot* :rarm :end-coords :copy-worldcoords)
            (send (send (send (send *drill* :get :button-coords) :copy-worldcoords)
                        :translate (float-vector x-offset y-offset z-offset) :local) :rotate (deg2rad 150) :z :local)
            (send *robot* :rleg :end-coords :copy-worldcoords)
            (send *robot* :lleg :end-coords :copy-worldcoords)))
     (setq ret
           (send *robot*
                 :fullbody-inverse-kinematics tc
                 :move-target mt :link-list ll
                 :additional-weight-list
                 (list (list (send *robot* :rleg :toe-p :child-link) 0)
                       (list (send *robot* :lleg :toe-p :child-link) 0))
                 :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
                 :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                 :translation-axis (list t t t t)
                 :rotation-axis (list t t t t)
                 :max (float-vector  500  500  0  20  20  10) ;; for hrp2
                 :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
                 :stop 200
                 :centroid-thre 50
                 :debug-view (if draw? :no-message nil)
                 :warnp (if draw? t nil)
                 )))
    ret)
  )

(defun generate-drill-button-motion
  (&key (draw? t))
  (let* (ik-results
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; push buton
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    (let* (ik-ret)
      ;; pre-push1
      (send *robot* :angle-vector (get-look-drill-pose))
      (send *robot* :fix-leg-to-coords (make-coords))
      (setq ik-ret (send *robot* :rarm :move-end-rot -20 :z :world))
      (push-back (list 0 (not (null ik-ret)) (send *robot* :angle-vector)) ik-results)
      ;; pre-push2
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 42.5 :y-offset -15 :z-offset -100 :draw? draw?))
      (push-back (list 1 (not (null ik-ret)) ik-ret) ik-results)
      ;; pre-push3
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 42.5 :y-offset -15 :z-offset -10 :draw? draw?))
      (push-back (list 2 (not (null ik-ret)) ik-ret) ik-results)
      ;; push
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 42.5 :y-offset -15 :z-offset 15 :draw? draw?))
      (push-back (list 3 (not (null ik-ret)) ik-ret) ik-results)
      (setq *drill-ik-results* ik-results)
      ))
  )

(defun get-drill-button-motion
  (&key (draw? t))
  (generate-drill-button-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*))
      (push-back (elt (elt *drill-ik-results* 2) 2) angle-vector-ret)
      (push-back (elt (elt *drill-ik-results* 1) 2) angle-vector-ret)
      )
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-drill-button-motion-full
  (drill-coords &key (draw? t) (use-ri? t)) ;; tmp draw
  (set-drill-button-environment-full drill-coords :draw? draw? :use-ri? use-ri?)
  (get-drill-button-motion :draw? draw?)
  )

(defun send-drill-button-motion
  (coords &key (real? t))
  (let* ((drill-motion-av-list (get-drill-button-motion-full coords))
         (drill-motion-length (length drill-motion-av-list)))
    (when (not (= drill-motion-length 0))
      (when real? (send *ri* :start-impedance :larm :moment-gain (float-vector 0 0 0) :k-p 2000 :d-p 400))
      ;; pre-push1
      (send *robot* :angle-vector (elt drill-motion-av-list 0))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      ;; pre-push2
      (send *robot* :angle-vector (elt drill-motion-av-list 1))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 10000))
      ;; pre-push3
      (send *robot* :angle-vector (elt drill-motion-av-list 2))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      ;; push
      (send *robot* :angle-vector (elt drill-motion-av-list 3))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      ;; pre-push3
      (send *robot* :angle-vector (elt drill-motion-av-list 4))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      ;; pre-push2
      (send *robot* :angle-vector (elt drill-motion-av-list 5))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      )
    ))

(defun publish-drill-button-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-drill-button-motion-full coords :use-ri? nil))
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (publish-motion-joint-states *drill-motion* :step-time step-time)
  )

(defun send-drill-look-motion
  (&key (real? nil))
  ;; look
  (send *robot* :angle-vector (get-look-drill-pose))
  (send *robot* :hand :rarm :angle-vector (get-grasp-drill-hand-pose))
  (send *robot* :fix-leg-to-coords (make-coords))
  (when real?
    (model2real :time 10000)
    (hand-model2real))
  )

;;(set-drill-button-environment :drill-coords nil :draw? t :generate-ri? t)
;;(send-drill-look-motion :real? t)
;;(send-drill-button-motion (get-primitive-marker-pose :frame-id "BODY"))

;; drill wall ;;;;

;; drill util ;;;;

(defun get-reach-drill-pre-pose ()
  (float-vector -8.144307e-06 -0.000153 -36.5591 70.8929 -34.3338 0.000178 0.0 8.144307e-06 0.000153 -36.5591 70.8929 -34.3338 -0.000178 0.0 40.0 0.0 0.0 40.0 -56.1706 -73.4951 92.0 -127.965 106.0 37.3389 20.1853 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-pierce-drill-pre-pose ()
  (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 1.19212 -20.9065 -5.36626 -98.3812 50.0 6.28459 -83.1406 60.0 -17.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-push-drill-pre-pose ()
(float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -23.0209 -7.67889 10.4985 -75.8274 97.2715 -28.3553 -71.5569 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-overhand-grasp-pre-pose ()
#f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -130.0 -25.0 -5.0 80.0 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-grasp-drill-default-pose ()
  #f(-3.00133 2.02892 -27.2933 50.8864 -23.7853 -2.19867 0.0 -3.00129 2.03943 -27.2041 49.4034 -22.3915 -2.20917 0.0 39.4881 32.13 -45.0 17.2412 33.0021 -84.6379 -28.4436 -91.286 89.3123 6.3681 36.9771 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-look-drill-pose ()
  #f(-1.73786 12.7998 -28.717 58.3273 -27.2775 -3.19394 0.0 -1.75078 13.1089 -32.0242 64.2536 -29.8937 -3.50327 0.0 -18.0177 40.9471 -15.5394 -20.0 -71.9257 -1.63271 9.57461 -68.7006 -18.7947 12.8132 13.4518 60.0 -67.3083 63.2485 -56.2882 -89.6456 -32.2721 16.8311 -17.2177 -60.0)
  ;;#f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 40.0 -16.9174 -3.95202 39.3961 -81.2715 -74.5661 -40.3588 -35.2676 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-button-drill-pre-pose ()
  #f(-3.00133 2.02892 -27.2933 50.8864 -23.7853 -2.19867 0.0 -3.00129 2.03943 -27.2041 49.4034 -22.3915 -2.20917 0.0 39.4881 32.13 -45.0 17.2412 -18.1543 -92.9917 30.9097 -118.766 87.4559 26.6536 47.0384 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )

(defun get-grasp-drill-hand-pose ()
  #f(90.0 40.0 5.0 10.0 100.0 50.0)
  ;;#f(90.0 40.0 5.0 10.0 45.0 50.0)
  )
(defun get-button-drill-hand-pose ()
  #f(90.0 90.0 0.0 0.0 0.0 115.0)
  ;;#f(90.0 0.0 0.0 0.0 0.0 115.0)
  )

;; TODO pre-push-pose
(defun test-drill-grasp-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 600 -150 0) :rpy (list 0 0 0)) :draw? t))
  (play-motion-on-eus-viewer *drill-motion*)
  )

;; (defun tmp-drill
;;   ()
;;   ;; generate
;;   (set-drill-grasp-environment)
;;   (get-drill-grasp-motion)
;;   ;; grasp
;;   (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   ;;(send *robot* :angle-vector (elt *drill-motion* 1))
;;   (send *robot* :hand :rarm :angle-vector (get-grasp-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; look
;;   (send *robot* :angle-vector (get-look-drill-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; set
;;   (send (send *robot* :rarm :end-coords) :dissoc *drill*)
;;   (send *robot* :angle-vector (send *ri* :state :potentio-vector))
;;   (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-primitive-marker-pose :frame-id "BODY") :local))
;;   (send (send *robot* :rarm :end-coords) :assoc *drill*)
;;   ;; move
;;   ;; (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   ;; (send *robot* :rarm :move-end-pos #f(-100 0 0) :world)
;;   ;; (send *robot* :rarm :move-end-pos #f(0 100 0) :world)
;;   ;; (send *robot* :rarm :move-end-rot 10 :z :world) ;; for hrp2jsknt
;;   ;; (send *robot* :rarm :move-end-rot 20 :z :world) ;; for hrp2jsknts
;;   (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   (tmptmp)
;;   (send *robot* :rarm :move-end-pos #f(-100 0 0) :world)
;;   (send *robot* :rarm :move-end-pos #f(0 100 0) :world)
;;   ;; (send *robot* :angle-vector (get-button-drill-pre-pose))
;;   (send *robot* :hand :larm :angle-vector (get-button-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; pre-push
;;   (tmp-drill-pre-push)
;;   ;; push
;;   (dolist (i (list -5 0 5))
;;     (dolist (j (list -5 0 5))
;;   ;; (dolist (i (list -10 -5 0 5 10))
;;   ;;   (dolist (j (list -10 -5 0 5 10))
;;       (print (list i j))
;;       (tmp-drill-pre-push i j)
;;       (model2real :time 2000)
;;       (read-line)
;;       (send *ri* :reach-until-touch :larm (float-vector 0 0 1) :coords-system :local :thre 1.0 :ik-mode :fullbody :dist-step 5 :max-dist 120 :time-step 200 :debug t)
;;       ))
;;   )


;; (defun tmptmp
;;   ()
;;   (with-move-target-link-list
;;    (mt ll *robot* '(:rarm :rleg :lleg))
;;    (setf (elt mt 0) (send *drill* :get :button-coords))
;;    ;; (setf (elt mt 0)
;;    ;;       (make-cascoords
;;    ;;        :coords (send (send *drill* :get :button-coords) :copy-worldcoords)
;;    ;;        :parent (send (send *robot* :rarm :end-coords) :parent)))
;;    (setq tc
;;          (list
;;           (make-coords :pos (send (send (send *drill* :get :button-coords) :copy-worldcoords) :worldpos) :rpy (list 0 0 pi/2))
;;           (send *robot* :rleg :end-coords :copy-worldcoords)
;;           (send *robot* :lleg :end-coords :copy-worldcoords)))
;;    (send *robot*
;;          :fullbody-inverse-kinematics tc
;;          :move-target mt :link-list ll
;;          :additional-weight-list
;;          (list (list (send *robot* :rleg :toe-p :child-link) 0)
;;                (list (send *robot* :lleg :toe-p :child-link) 0))
;;          :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
;;          :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
;;          :translation-axis (list t t t)
;;          :rotation-axis (list t t t)
;;          :max (float-vector  500  500  0  20  20  10) ;; for hrp2
;;          :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
;;          :stop 200
;;          :centroid-thre 50
;;          :debug-view :no-message
;;          ))
;;   )

;; (defun tmp-drill-pre-push
;;   (&optional (x-offset 0) (y-offset 0))
;;   (with-move-target-link-list
;;    (mt ll *robot* '(:rarm :larm :rleg :lleg))
;;    (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
;;    (setq tc
;;          (list
;;           (send *robot* :rarm :end-coords :copy-worldcoords)
;;           ;;(send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset 10) :local)
;;           ;;(send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset -25) :local)
;;           (send (send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset -25) :local) :rotate (deg2rad 150) :z :local)
;;           ;;(send *robot* :larm :end-coords :copy-worldcoords)
;;           (send *robot* :rleg :end-coords :copy-worldcoords)
;;           (send *robot* :lleg :end-coords :copy-worldcoords)))
;;    (send *robot*
;;          :fullbody-inverse-kinematics tc
;;          :move-target mt :link-list ll
;;          :additional-weight-list
;;          (list (list (send *robot* :rleg :toe-p :child-link) 0)
;;                (list (send *robot* :lleg :toe-p :child-link) 0))
;;          :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
;;          :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
;;          :translation-axis (list t t t t)
;;          :rotation-axis (list t t t t)
;;          ;; :rotation-axis (list t t t t)
;;          :max (float-vector  500  500  0  20  20  10) ;; for hrp2
;;          :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
;;          :stop 200
;;          :centroid-thre 50
;;          :debug-view :no-message
;;          ;; :debug-view nil
;;          ))
;;   )

;; (defun tmp-drill-push
;;   (&optional (x-offset 0) (y-offset 0))
;;   (with-move-target-link-list
;;    (mt ll *robot* '(:rarm :larm :rleg :lleg))
;;    (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
;;    (setq tc
;;          (list
;;           (send *robot* :rarm :end-coords :copy-worldcoords)
;;           ;;(send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset 60) :local)
;;           ;;(send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset 30) :local)
;;           (send (send (send (send *drill* :get :button-coords) :copy-worldcoords) :translate (float-vector x-offset y-offset 25) :local) :rotate (deg2rad 150) :z :local)
;;           ;;(send *robot* :larm :end-coords :copy-worldcoords)
;;           (send *robot* :rleg :end-coords :copy-worldcoords)
;;           (send *robot* :lleg :end-coords :copy-worldcoords)))
;;    (send *robot*
;;          :fullbody-inverse-kinematics tc
;;          :move-target mt :link-list ll
;;          :additional-weight-list
;;          (list (list (send *robot* :rleg :toe-p :child-link) 0)
;;                (list (send *robot* :lleg :toe-p :child-link) 0))
;;          :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
;;          :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
;;          :translation-axis (list t t t t)
;;          :rotation-axis (list t t t t)
;;          :max (float-vector  500  500  0  20  20  10) ;; for hrp2
;;          :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
;;          :stop 200
;;          :centroid-thre 50
;;          :debug-view nil
;;          ))
;;   )

;;(send *ri* :reach-until-touch :larm #f(0 0 1) :coords-system :local :thre 2)

;;(send *robot* :angle-vector #f(-1.73786 12.7998 -28.717 58.3273 -27.2775 -3.19394 0.0 -1.75078 13.1089 -32.0242 64.2536 -29.8937 -3.50327 0.0 -18.0177 40.9471 -15.5394 -20.0 -71.9257 -1.63271 9.57461 -68.7006 -18.7947 12.8132 13.4518 60.0 -67.3083 63.2485 -56.2882 -89.6456 -32.2721 16.8311 -17.2177 -60.0))
;;(send *robot* :fix-leg-to-coords (make-coords))
;; (send *robot* :rarm :move-end-rot -20 :z :world)
;;(tmp-drill-pre-push 0 20)
;;(tmp-drill-push 0 20)

