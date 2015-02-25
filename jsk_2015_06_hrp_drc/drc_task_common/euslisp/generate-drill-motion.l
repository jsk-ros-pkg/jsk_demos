(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")

;; drill grasp ;;;;

(defun set-drill-grasp-environment
  (&key (drill-coords (make-coords :pos (float-vector 600 0 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; attach handle to robot
  (send *robot* :put :larm-drill-grasp-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 5.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  (send *robot* :put :rarm-drill-grasp-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 -5.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  ;; drill setting
  (let* ((drill-r 30) (drill-h 120))
    ;; generate drill
    (setq *drill* (make-cylinder drill-r drill-h))
    (send *drill* :set-color (float-vector 1 1 0))
    ;; atattch handle to drill
    (send *drill* :put :grasp-coords
          (make-cascoords
           :coords (make-coords :pos (float-vector 0 0 (* drill-h 0.5)))
           :parent *drill*))
    (send *drill* :put :button-coords
          (make-cascoords
           :coords (make-coords :pos (float-vector 25 25 10))
           :parent *drill*))
    ;; place drill to drill-coords
    (when drill-coords
      (send *drill* :newcoords drill-coords)
      ;; treat the difference of origin between eus and recognition program
      (send *drill* :translate (float-vector 0 0 (* -0.5 drill-h)) :local))
    )
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
    (send *drill* :translate (float-vector 0 0 (* -0.5 (elt (send *drill* :body-type) 2))) :local)
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
  (let* ((reach-dist 150)
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
             (send (send (send (send *drill* :get :grasp-coords) :copy-worldcoords) :translate (float-vector 0 0 -100) :local) :rotate 0 :x)
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
      (send *robot* :angle-vector (get-push-drill-pre-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (push-back (list 3 t (send *robot* :angle-vector)) ik-results)
      (setq *drill-ik-results* ik-results)
      ))
  )

(defun get-drill-grasp-motion
  ()
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*)))
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-drill-grasp-motion-full
  (drill-coords &key (draw? nil) (use-ri? t))
  (set-drill-grasp-environment-full drill-coords :draw? draw? :use-ri? use-ri?)
  (generate-drill-grasp-motion :draw? draw?)
  (get-drill-grasp-motion)
  )

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
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (publish-motion-joint-states *drill-motion* :step-time step-time))
;; drill connect ;;;;

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
(defun get-overhand-grasp-pre-pose()
#f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -130.0 -25.0 -5.0 80.0 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
;;TODO pre-push-pose
(defun test-drill-grasp-motion ()
  (set-drill-grasp-environment)
  (generate-drill-grasp-motion)
  (get-drill-grasp-motion)
  (play-motion-on-eus-viewer *drill-motion*)
  )



;; (defun tmp-drill
;;   ()
;;   (send *robot* :hand :rarm :hook-pose)
;;   (send *drill* :move-coords (send (send *robot* :get :rarm-drill-grasp-coords) :copy-worldcoords) (send (send *drill* :get :grasp-coords) :copy-worldcoords))

 
;;   (send (send *drill* :get :button-coords) :copy-worldcoords)


;;   (send (send (send *drill* :get :button-coords) :copy-worldcoords) :draw-on :flush t :color #f(1 0 0))
;;   )


;; (with-move-target-link-list
;;  (mt ll *robot* '(:rarm :larm :rleg :lleg))
;;  (setq tc
;;        (list
;;         (send *robot* :rarm :end-coords :copy-worldcoords)
;;         (send (send *drill* :get :button-coords) :copy-worldcoords)
;;         ;;(send *robot* :larm :end-coords :copy-worldcoords)
;;         (send *robot* :rleg :end-coords :copy-worldcoords)
;;         (send *robot* :lleg :end-coords :copy-worldcoords)))
;;  (send *robot*
;;        :fullbody-inverse-kinematics tc
;;        :move-target mt :link-list ll
;;        :additional-weight-list
;;        (list (list (send *robot* :rleg :toe-p :child-link) 0)
;;              (list (send *robot* :lleg :toe-p :child-link) 0))
;;        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
;;        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
;;        :translation-axis (list t t t t)
;;        :rotation-axis (list t t t t)
;;        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
;;        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
;;        :stop 100
;;        :debug-view nil
;;        ))
