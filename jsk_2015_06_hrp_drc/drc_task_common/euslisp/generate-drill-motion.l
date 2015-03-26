(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/models/dewalt_dc550ka.l")
(load "package://drc_task_common/models/gun-drill.l")

(setq *drill-main-arm* :rarm)
(setq *drill-support-arm* :larm)
(setq *default-drill-type* :gun)
(setq *use-dual-arm-ik* t)

;; takenoko drill
(defun init-takenoko-drill ()
  (setq *drill* (dewalt_dc550ka-robot))
  ;; generate drill
  (send-all (send *drill* :bodies) :set-color #f(0.7 0.6 0.0))
  ;; atattch handle to drill
  (send *drill* :put :type :takenoko)
  (send *drill* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 190) :rpy (list 0 pi 0))
         :parent *drill*))
  (send *drill* :put :support-grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 190) :rpy (list 0 pi 0))
         :parent *drill*))
  
  (send *drill* :put :button-coords
        (make-cascoords
         :coords (make-coords :pos #f(-22 30 97) :rpy (list 0 0 pi/2))
         :parent *drill*))
  (send *drill* :put :put-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0))
         :parent *drill*))
  (send *drill* :put :origin-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0))
         :parent *drill*))
  (send *drill* :put :edge-coords
        (make-cascoords
         ;;:coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0))
         :coords (make-coords :pos #f(0 0 340) :rpy (list 0 0 0))
         :parent *drill*))
  ;; (when drill-coords
  ;;   (send *drill* :newcoords drill-coords))
  )
;; gun drill

(defun init-gun-drill ()
  (setq *drill* (gun-drill))
  ;; generate drill
  (send-all (send *drill* :bodies) :set-color #f(0.0 0.6 0.7))
  ;; atattch handle to drill
  (send *drill* :put :type :gun)
  (send *drill* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 -10) :rpy (list 0 0 0))
         :parent *drill*))
  (send *drill* :put :support-grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(50 100 100) :rpy (list 0 0 pi/2))
         :parent *drill*))
  (send *drill* :put :button-coords 
        (make-cascoords
         :coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0))
         :parent *drill*))
  (send *drill* :put :put-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 -150) :rpy (list 0 0 0)) 
         :parent *drill*))
  (send *drill* :put :origin-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0)) 
         :parent *drill*))
  (send *drill* :put :edge-coords
        (make-cascoords
         ;;:coords (make-coords :pos #f(0 0 0) :rpy (list 0 0 0))
         :coords (make-coords :pos #f(120 0 50) :rpy (list 0 pi/2 0))
         :parent *drill*))
  (send *drill* :put :destination-coords-list nil)
  ;; (when drill-coords
  ;;   (send *drill* :newcoords drill-coords))  
  )

(defun get-drill-coords-with-symbol (coords symbol)
  (ros::ros-info (format nil "got coords ~A symbol ~A drill ~A" (send *drill* :get symbol) symbol *drill*))
  (send (send coords :copy-worldcoords) :transform (send (send (send *drill* :get symbol) :copy-coords) :inverse-transformation))
  )

;; drill grasp ;;
(defun set-drill-environment
  (&key (drill-coords (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNT")
        (drill-type *default-drill-type*)
        (recognized-coords-symbol :origin-coords))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :hand :arms :reset-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  (setq *arm-drill-grasp-coords* (make-hash-table))
  (setf (gethash :rarm *arm-drill-grasp-coords*)
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 48.0 12.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  (setf (gethash :larm *arm-drill-grasp-coords*)
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 48.0 -12.0 0.0) :local)
         :parent (send (send *robot* :larm :end-coords) :parent)))


  (cond ((equal drill-type :takenoko) (init-takenoko-drill))
        ((equal drill-type :gun) (init-gun-drill))
        )
  (when drill-coords 
     (send *drill* :newcoords (get-drill-coords-with-symbol drill-coords recognized-coords-symbol)))
  ;; drill setting
  (setq *drill-stand-coords* nil)
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
    (objects (list *robot* *drill* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun set-drill-environment-full
  (drill-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t)
   (drill-type *default-drill-type*)
   (recognized-coords-symbol :origin-coords))
  ;; place robot and drill to initial coords
  (set-drill-environment :drill-coords nil :draw? draw? :generate-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  (cond (use-ri?
         (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
        (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
  (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
  (ros::ros-info (format nil "drill coords ~A" drill-coords))
  (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform 
                                 (get-drill-coords-with-symbol drill-coords recognized-coords-symbol)
                                 :local))
  ;treat the difference of origin between eus and recognition program
  ;(send *drill* :translate (float-vector 0 0 (* -0.5 (elt (send *drill* :body-type) 2))) :local)
  (when draw? (send *irtviewer* :draw-objects))
  )

(defun fullbody-inverse-kinematics-for-drill-grasp
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0))
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis translation-axis
        :rotation-axis rotation-axis
        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
        :stop 100
        :debug-view (if draw? :no-message nil)
        )
  )

(defun fullbody-inverse-kinematics-for-drill-grasp-single
  (&key (arm-symbol :grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (draw? t))
  (let 
      ((tc (list
           arm-tc
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* :rleg :lleg))
     (setf (elt mt 0) (gethash *drill-main-arm* *arm-drill-grasp-coords*))
     (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw?))
     (when ik-ret
       (setq ik-ret
             (send *robot* :look-at-target (send *drill* :worldcoords)))))
    ik-ret))
(defun fullbody-inverse-kinematics-for-drill-grasp-dual
  (&key (arm-symbol :grasp-coords) (support-arm-symbol :support-grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (support-arm-tc (send (send *drill* :get support-arm-symbol) :copy-worldcoords)) (draw? t))
  (let 
      ((tc (list
           arm-tc
           support-arm-tc
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
     (setf (elt mt 0) (gethash *drill-main-arm* *arm-drill-grasp-coords*))
     (setf (elt mt 1) (gethash *drill-support-arm* *arm-drill-grasp-coords*))
     (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw? :translation-axis (list t t t t) :rotation-axis (list t t t t)))
     (when ik-ret
       (setq ik-ret
             (send *robot* :look-at-target (send *drill* :worldcoords)))))
    ik-ret))


(defun search-drill-grasp-place (&key (reach-dist 100) (debug nil)) ;; thinks recognition 3cm diff
  (let ((reach-dist-x reach-dist) (reach-dist-y (if (equal *drill-main-arm* :rarm) reach-dist (- reach-dist))) (reach-dist-z (/ reach-dist 4)))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector reach-dist-x reach-dist-y 0) :max-dist (- (sqrt (+ (* reach-dist-x reach-dist-x) (* reach-dist-y reach-dist-y))) -1) :debug debug :thre 0.71 :dist-step 3)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector reach-dist-x 0 0) :max-dist (+ 0 100) :debug debug :thre 1 :dist-step 2)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector (- reach-dist-x) 0 0) :max-dist (+ 0) :debug debug :thre 13 :dist-step 2)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector 0 reach-dist-y 0) :max-dist (+ 0 100) :debug debug :thre 1 :dist-step 2)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector 0 (- reach-dist-y) 0) :max-dist (+ 13) :debug debug :thre 13 :dist-step 2)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector 0 0 reach-dist-z) :max-dist (+ 0 100) :debug debug :thre 1 :dist-step 2)) (return-from nil nil))
    ;; (if (not (send *ri* :reach-until-touch *drill-main-arm* (float-vector 0 0 -1) :max-dist (+ 30) :debug debug :thre 13 :dist-step 2)) (return-from nil nil))
    ;; t
    (if (equal *drill-main-arm* :rarm) (send *ri* :reach-until-touch :rarm (float-vector 1 1 1) :max-dist (+ 0 250) :debug t :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t)
      (send *ri* :reach-until-touch :larm (float-vector 1 -1 1) :max-dist (+ 0 250) :debug t :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t)
      )
    )
  )

(defun generate-drill-grasp-motion
  (&key (draw? t))
  (let* ((reach-dist 100)
	 (grasp-up-dist 150)
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
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw?))
      (push-back (list 2 (not (null ik-ret)) ik-ret) ik-results)
      ;; solve ik for pre grasp
      (let ((robot-angle-vector (send *robot* :angle-vector)))
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :translate (float-vector (- 20) (if (equal *drill-main-arm* :rarm) (- 20) 20) -15) :local) :draw? draw?))
        (push (list 1 (not (null ik-ret)) ik-ret) ik-results)
	(send *robot* :angle-vector robot-angle-vector)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) (if (equal *drill-main-arm* :rarm) (- reach-dist) reach-dist) -15) :local) :draw? draw?))
	(push (list 0 (not (null ik-ret)) ik-ret) ik-results)
	(send *robot* :angle-vector robot-angle-vector)
	;; solve ik for grasp up	
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :translate (float-vector 0 0 grasp-up-dist) :world) :draw? draw?))
	(push-back (list 3 (not (null ik-ret)) ik-ret) ik-results)
	(send *robot* :angle-vector robot-angle-vector)
	)
      ;; draw
      (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; drill-reset-pose
      ;(send *robot* :angle-vector (get-look-drill-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      ;(push-back (list 3 t (send *robot* :angle-vector)) ik-results)
      (setq *drill-ik-results* ik-results)
      )))

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
  (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -600 300 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -600 -300 0) :rpy (list (deg2rad 0) 0 0)))
  )
(defun get-drill-grasp-motion-full
  (drill-coords &key (draw? nil) (use-ri? t) (drill-type *default-drill-type*) (recognized-coords-symbol :origin-coords))
  (set-drill-environment-full drill-coords :draw? draw? :use-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  
  (let* ((drill-motion-av-list (get-drill-grasp-motion :draw? draw?))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0)
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

                  (let ((drill-grasp-coords (send (send drill-oriented-coords :copy-worldcoords)
                                                         :transform (get-drill-grasp-stand-coords))))
                    (setq *drill-stand-coords* (make-coords :pos (float-vector (elt (send drill-grasp-coords :worldpos) 0) (elt (send drill-grasp-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-grasp-coords :rpy-angle)) 0) 0 0)))
                    )
                  )))))
      (ros::ros-warn "change stand position to ~a for valve motion.~%" *drill-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-stand-coords*)
      (when draw? (send *irtviewer* :draw-objects))
      (setq drill-motion-av-list (get-drill-grasp-motion :draw? draw?))
      )
    drill-motion-av-list
    ))

  

(defun send-drill-grasp-motion
  (coords &key (real? t) (drill-type *default-drill-type*))
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords :drill-type drill-type))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-grasp-motion))
    (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
    (when real? (hand-model2real))
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for valve~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    (send *robot* :reset-manip-pose)    
    ;; first reaching
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      )
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
    ;; pre-reach
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    (send *robot* :angle-vector (elt drill-motion-av-list 1))
    (when real? (model2real :time 5000))
    ;; reach
    (when (not (search-drill-grasp-place)) ;; cannot solve ik
        (send *robot* :angle-vector (elt drill-motion-av-list 2))
      (when real? (model2real :time 2000))
      )
    ;grasp
    (when real? (drill-t-grasp))
    (unix::usleep 500000)
    (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 90.0))
    (when real? (hand-model2real))
    ;;grasp-up
    (send *robot* :angle-vector (elt drill-motion-av-list 3))
    (when real? (model2real :time 3000))
    ;; ref-force todo
    
    ;; ;; drill-reset-pose
    ;; (send *robot* :angle-vector (elt drill-motion-av-list 4))
    ;; (when real? (model2real :time 5000))
    )
  )

(defun publish-drill-grasp-motion
  (&key coords (step-time 400) (update? t) (drill-type *default-drill-type*))
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil :drill-type drill-type))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (if *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground")
    (move-robot-model (make-coords) "ground"))
  
  (publish-motion-joint-states *drill-motion* :step-time step-time))



;; drill put
;; almost same with grasp at present
(defun send-drill-put-motion
  (coords &key (real? t))
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords :drill-tupe drill-type :recognized-coords-symbol :put-coords))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-grasp-motion))
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for valve~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    
    ;; first reaching
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp

    ;; ;; drill-reset-pose
    ;; (send *robot* :angle-vector (elt drill-motion-av-list 3))
    ;; (when real? (model2real :time 5000))
    ;; put-up
    (send *robot* :angle-vector (elt drill-motion-av-list 2))
    (when real? (model2real :time 3000))      
    ;; put
    (send *robot* :angle-vector (elt drill-motion-av-list 1))
    (when real? (model2real :time 3000))
    ;; hook
    (drill-hook)
    ;; release
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (send *robot* :reset-manip-pose)
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    )
  )

(defun publish-drill-put-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil :recognized-coords-symbol :put-coords :drill-type :gun))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (when *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground"))
  (setq *drill-motion* (reverse *drill-motion*))
  (publish-motion-joint-states *drill-motion* :step-time step-time))



;; drill button ;;;;

(defun set-drill-button-environment
  (&key (drill-coords (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNT")
        (drill-type *default-drill-type*))
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
  ;;        :coords (send (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(121 -21 24) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand *drill-support-arm* :link "L_INDEXPIP_R_LINK")))
  ;; for hrp2jsknts
  ;; (send *robot* :put :larm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(120 -23 23) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand *drill-support-arm* :link "L_INDEXPIP_R_LINK")))
  ;; TODO change to use *drill-main-arm*
  (send *robot* :put :larm-drill-button-coords
        (make-cascoords
         :coords (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(50 87.5 80) :local)
         ;;:coords (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(50 62 80) :local)
         :parent (send (send *robot* *drill-support-arm* :end-coords) :parent)))
  ;; drill setting
  ;; generate drill
  (cond ((equal drill-type :takenoko) (init-takenoko-drill))
        ((equal drill-type :gun) (init-gun-drill))
        )
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
    (send (send *robot* *drill-main-arm* :end-coords) :dissoc *drill*)
    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform drill-coords :local))
    (send (send *robot* *drill-main-arm* :end-coords) :assoc *drill*)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-drill-button
  (&key (x-offset 0) (y-offset 0) (z-offset 0) (draw? t))
  (let* (ret)
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
     (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
     (setq tc
           (list
            (send *robot* *drill-main-arm* :end-coords :copy-worldcoords)
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
      (setq ik-ret (send *robot* *drill-main-arm* :move-end-rot -20 :z :world))
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
      (when real? (send *ri* :start-impedance *drill-support-arm* :moment-gain (float-vector 0 0 0) :k-p 2000 :d-p 400))
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
  (send *robot* :hand *drill-main-arm* :angle-vector (get-grasp-drill-hand-pose))
  (send *robot* :fix-leg-to-coords (make-coords))
  (when real?
    (model2real :time 10000)
    (hand-model2real))
  )

;;(set-drill-button-environment :drill-coords nil :draw? t :generate-ri? t)
;;(send-drill-look-motion :real? t)
;;(send-drill-button-motion (get-primitive-marker-pose :frame-id "BODY"))

;; drill wall ;;;;

(defun set-drill-rotation (&key (rotate 0))
  (let* ((edge-coords (send (send *drill* :get :edge-coords) :copy-worldcoords)) 
         (edge-mat (send edge-coords :copy-rot))
         (uz (matrix-column edge-mat 2))
         (uy (normalize-vector (v* (float-vector 0 0 1) uz)))
         (ux (v* uy uz))
         (new-mat (make-matrix 3 3))
         new-coords
         )
    (if (equal uy (float-vector 0.0 0.0 0.0)) (return-from nil nil))
    (setf (matrix-column new-mat 0) ux)
    (setf (matrix-column new-mat 1) uy)
    (setf (matrix-column new-mat 2) uz)
    (setq new-coords
          (make-coords :pos (send edge-coords :pos)
                       :rot new-mat))
    (send new-coords :rotate rotate :x)
    (send *drill* :newcoords (get-drill-coords-with-symbol new-coords :edge-coords))
    )
  )

(defun generate-drill-wall-motion
  (&key (draw? t))
  (let* (
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; wall drill
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to wall drill
    (let* (ik-ret
           tc
           (dt-coords-list (send *drill* :get :destination-coords-list))
           (dt-coords-length (length dt-coords-list)))
      (when draw? (send (send *drill* :get :grasp-coords) :draw-on :flush t))
      ;; solve ik for walls
      (ros::ros-info (format nil "dt-coords-list ~A" dt-coords-list))
      (dotimes (i dt-coords-length)
        (send *drill* :newcoords (elt dt-coords-list i))
        ;;(send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-drill-coords-with-symbol (elt dt-coords-list i) :origin-coords) :local))
        (set-drill-rotation)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw?))
        (push-back (list (+ i 1) (not (null ik-ret)) ik-ret) ik-results)
        (when (or (equal i 0) (equal i (- dt-coords-length 1)))
          (send *drill* :translate (float-vector -60 0 0))
          (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw?))
           (when ik-ret
             (setq ik-ret
                   (send *robot* :look-at-target (send *drill* :worldcoords)))))
          (if (equal i 0) (push (list 0 (not (null ik-ret)) ik-ret) ik-results))
          (if (or (equal i (- dt-coords-length 1)) (equal dt-coords-length 0)) (push-back (list (+ dt-coords-length 1) (not (null ik-ret)) ik-ret) ik-results))
          )
        )
      ;; draw
      (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; drill-reset-pose
      ;(send *robot* :angle-vector (get-look-drill-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      ;(push-back (list 3 t (send *robot* :angle-vector)) ik-results)
      (setq *drill-ik-results* ik-results)
      ))
  )

(defun get-drill-wall-motion
  (&key (draw? t))
  (generate-drill-wall-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*)))
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-drill-wall-stand-coords ;; relative to drill
  ()
  (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -600 0 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -600 0 0) :rpy (list (deg2rad 0) 0 0)))
  )
(defun get-drill-wall-motion-full
  (drill-coords-list &key (draw? nil) (use-ri? t) (drill-type *default-drill-type*) (recognized-coords-symbol :edge-coords))
  (when (equal (length drill-coords-list) 0) (ros::ros-info "wall coords are empty") (return-from nil nil))
  (set-drill-environment-full (send (elt drill-coords-list 0) :copy-worldcoords) :draw? draw? :use-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  ;(send *drill* :put :destination-coords-list drill-coords-list)
  (let ((dest-coords-list nil))
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (dotimes (i (length drill-coords-list))
      (cond (use-ri?
             (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
            (t
             (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
      (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
      (push-back (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-drill-coords-with-symbol (elt drill-coords-list i) recognized-coords-symbol) :local) dest-coords-list)
      )
    (send *drill* :put :destination-coords-list dest-coords-list)
    )
  (let* ((drill-motion-av-list (get-drill-wall-motion :draw? draw?))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0)
        (let ((theta (atan2 (elt (send (elt drill-coords-list 0) :pos) 1)
                            (elt (send (elt drill-coords-list 0) :pos) 0))))
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
                       (make-coords :pos (send (elt drill-coords-list 0) :worldpos)
                                    :rot mat)))

                  (let ((drill-wall-coords (send (send drill-oriented-coords :copy-worldcoords)
                                                         :transform (get-drill-wall-stand-coords))))
                    (setq *drill-stand-coords* (make-coords :pos (float-vector (elt (send drill-wall-coords :worldpos) 0) (elt (send drill-wall-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-wall-coords :rpy-angle)) 0) 0 0)))
                    )
                  )))))
      (ros::ros-warn "change stand position to ~a for valve motion.~%" *drill-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-stand-coords*)
      (when draw? (send *irtviewer* :draw-objects))
      (setq drill-motion-av-list (get-drill-wall-motion :draw? draw?))
      )
    drill-motion-av-list
    ))

  

(defun send-drill-wall-motion
  (coords-list &key (real? t) (drill-type *default-drill-type*))
  (let* ((drill-motion-av-list (get-drill-wall-motion-full coords-list :drill-type drill-type :recognized-coords-symbol :edge-coords))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-wall-motion))
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for valve~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    (send *robot* :reset-manip-pose)    
    ;; first reaching
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      )
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
    ;; pre-reach
    (dotimes (i (length drill-motion-av-list))
      (send *robot* :angle-vector (elt drill-motion-av-list i))
      (when real? (model2real :time 5000))
      )
    )
  )

(defun publish-drill-wall-motion
  (&key coords-list (step-time 400) (update? t) (drill-type *default-drill-type*))
  (when update?
    (get-drill-wall-motion-full coords-list :use-ri? nil :drill-type drill-type :recognized-coords-symbol :edge-coords))
  (ros::ros-info "drill-wall-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (if *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground")
    (move-robot-model (make-coords) "ground"))
  
  (publish-motion-joint-states *drill-motion* :step-time step-time))



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
(defun get-drill-test-pose ()
  #f(4.991507e-32 2.204884e-30 -26.0 50.0 -24.0 4.587941e-33 -1.378053e-31 2.552867e-32 1.247877e-32 -26.0 50.0 -24.0 -9.841308e-33 -6.315679e-31 3.719923e-32 -5.951877e-31 2.570109e-33 40.0 -3.98489 -10.7817 30.9963 -74.3163 -14.2952 -23.9135 -18.2156 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  )
(defun get-drill-larm-test-pose()
  #f(6.353318e-47 1.511780e-45 -26.0 50.0 -24.0 -1.024111e-48 -9.448623e-47 9.628560e-48 1.588330e-47 -26.0 50.0 -24.0 -5.407690e-48 5.445592e-46 -3.615974e-47 5.785559e-46 -4.503218e-48 6.202082e-14 33.8752 -23.3947 -2.34573 -71.2747 -14.0104 2.82435 13.9053 15.0 17.6065 18.0935 -5.46659 -81.6239 14.0 5.0 -26.5981 -15.0)
  )

;; TODO pre-push-pose
(defun test-drill-grasp-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 1000 -150 200) :rpy (list 0 0 0)) :draw? t :drill-type :gun))
  (play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-drill-put-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 1000 -150 200) :rpy (list 0 0 0)) :draw? t :drill-type :gun :recognized-coords-symbol :put-coords))
  
  (play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-drill-wall-motion ()
  (setq *drill-coords-list* (list (make-coords :pos (float-vector 1000 -150 300) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 500) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -350 300) :rpy (list 0 pi/2 0))))
  (setq *answer-for-grasp-test* (get-drill-wall-motion-full *drill-coords-list* :draw? t :drill-type :gun :recognized-coords-symbol :edge-coords))
  (play-motion-on-eus-viewer *drill-motion*)
  )
(defun re-view ()
  (play-motion-on-eus-viewer *drill-motion*)
  )

(defun re-test ()
  (load "generate-drill-motion.l")
  (test-drill-grasp-motion)
  (send *robot* :angle-vector (elt *drill-motion* 1))
  )
(defun drill-hook () ;; test code
  (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
  (hand-model2real)
  )
(defun drill-hook-a5 () ;; test code
  (unix::sleep 5)
  (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
  (hand-model2real)
  )
(defun drill-grasp () ;; test code
  (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 90.0))
  (hand-model2real)
  )
(defun drill-t-grasp () ;; test code
  (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 0.0 -10.0 0.0 -40.0 -40.0))
  (hand-model2real)
  )

;; (defun tmp-drill
;;   ()
;;   ;; generate
;;   (set-drill-environment)
;;   (get-drill-grasp-motion)
;;   ;; grasp
;;   (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   ;;(send *robot* :angle-vector (elt *drill-motion* 1))
;;   (send *robot* :hand *drill-main-arm* :angle-vector (get-grasp-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; look
;;   (send *robot* :angle-vector (get-look-drill-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; set
;;   (send (send *robot* *drill-main-arm* :end-coords) :dissoc *drill*)
;;   (send *robot* :angle-vector (send *ri* :state :potentio-vector))
;;   (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-primitive-marker-pose :frame-id "BODY") :local))
;;   (send (send *robot* *drill-main-arm* :end-coords) :assoc *drill*)
;;   ;; move
;;   ;; (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   ;; (send *robot* *drill-main-arm* :move-end-pos #f(-100 0 0) :world)
;;   ;; (send *robot* *drill-main-arm* :move-end-pos #f(0 100 0) :world)
;;   ;; (send *robot* *drill-main-arm* :move-end-rot 10 :z :world) ;; for hrp2jsknt
;;   ;; (send *robot* *drill-main-arm* :move-end-rot 20 :z :world) ;; for hrp2jsknts
;;   (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   (tmptmp)
;;   (send *robot* *drill-main-arm* :move-end-pos #f(-100 0 0) :world)
;;   (send *robot* *drill-main-arm* :move-end-pos #f(0 100 0) :world)
;;   ;; (send *robot* :angle-vector (get-button-drill-pre-pose))
;;   (send *robot* :hand *drill-support-arm* :angle-vector (get-button-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; pre-push
;;   (tmp-drill-pre-push)
;;   ;; push;;   (dolist (i (list -5 0 5))
;;     (dolist (j (list -5 0 5))
;;   ;; (dolist (i (list -10 -5 0 5 10))
;;   ;;   (dolist (j (list -10 -5 0 5 10))
;;       (print (list i j))
;;       (tmp-drill-pre-push i j)
;;       (model2real :time 2000)
;;       (read-line)
;;       (send *ri* :reach-until-touch *drill-support-arm* (float-vector 0 0 1) :coords-system :local :thre 1.0 :ik-mode :fullbody :dist-step 5 :max-dist 120 :time-step 200 :debug t)
;;       ))
;;   )


;; (defun tmptmp
;;   ()
;;   (with-move-target-link-list
;;    (mt ll *robot* (list *drill-main-arm* :rleg :lleg))
;;    (setf (elt mt 0) (send *drill* :get :button-coords))
;;    ;; (setf (elt mt 0)
;;    ;;       (make-cascoords
;;    ;;        :coords (send (send *drill* :get :button-coords) :copy-worldcoords)
;;    ;;        :parent (send (send *robot* *drill-main-arm* :end-coords) :parent)))
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
;;    (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
;;    (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
;;    (setq tc
;;          (list
;;           (send *robot* *drill-main-arm* :end-coords :copy-worldcoords)
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
;;    (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
;;    (setf (elt mt 1) (send *robot* :get :larm-drill-button-coords))
;;    (setq tc
;;          (list
;;           (send *robot* *drill-main-arm* :end-coords :copy-worldcoords)
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
;; (send *robot* *drill-main-arm* :move-end-rot -20 :z :world)
;;(tmp-drill-pre-push 0 20)
;;(tmp-drill-push 0 20)
