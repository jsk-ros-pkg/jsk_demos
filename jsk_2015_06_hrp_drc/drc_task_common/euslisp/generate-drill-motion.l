(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")
 ;(load "package://drc_task_common/models/dewalt_dc550ka.l")
(load "package://drc_task_common/models/takenoko-drill")
(load "package://drc_task_common/models/gun-drill.l")

(setq *drill-main-arm* :rarm)
(setq *drill-support-arm* :larm)
(setq *default-drill-type* :takenoko)
(setq *use-dual-arm-ik* nil)
(setq *use-f-sensor-search* nil)
(setq *drill-reverse-hand* t)
(setq *support-reverse-hand* nil)

;; takenoko drill

(defun set-attachment ()
  (when (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
    (setq *hand-attachment* (make-cube 10 10 100))
    (objects (list *robot* *hand-attachment* *drill*))
    (send *hand-attachment* :set-color #f(1 0 0))
    (send *hand-attachment* :newcoords (send (send (send *robot* :larm
							 :end-coords) :copy-worldcoords) :translate #f(80 62 50)))
    (send (send (send *robot* :larm :end-coords) :parent) :assoc *hand-attachment*)
    (send *robot* :reset-manip-pose)
    (send *robot* :reset-pose)
    )
  )
(defun init-takenoko-drill ()
  (setq *drill* (takenoko-drill))
  ;; generate drill
  (send-all (send *drill* :bodies) :set-color #f(0.7 0.6 0.0))
  ;; atattch handle to drill
  (send *drill* :put :type :takenoko)
  (send *drill* :put :grasp-coords
        (make-cascoords
         ;; :coords (make-coords :pos #f(0 0 170) :rpy (if *drill-reverse-hand* (list (deg2rad 40) 0 pi) (list (deg2rad -40) 0 0)))
         :coords (make-coords :pos #f(0 0 140) :rpy (if *drill-reverse-hand* (if *support-reverse-hand* (list (deg2rad -20) 0 pi) (list (deg2rad 150) 0 pi)) (list (deg2rad -40) 0 0)))
         ;; :coords (make-coords :pos #f(0 0 170) :rpy (if *drill-reverse-hand* (list (deg2rad 180) 0 pi) (list (deg2rad 90) 0 0)))
         :parent *drill*))
  (send *drill* :put :support-grasp-coords
        (make-cascoords
         :coords (make-coords :pos #f(0 0 170) :rpy (list 0 0 pi))
         :parent *drill*))
  (send *drill* :put :pre-support-grasp-coords
        (make-cascoords
         :coords (send (send (send *drill* :get :support-grasp-coords) :copy-coords) :translate (float-vector -100 (if (equal *drill-main-arm* :rarm) 100 -100) 0) :local)
         :parent *drill*))
  (send *drill* :put :button-coords
        (make-cascoords
         :coords (make-coords :pos #f(-22 30 77) :rpy (list 0 0 pi/2))
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
         :coords (make-coords :pos #f(70 100 100) :rpy (list 0 0 -pi/2))
         :parent *drill*))
  (send *drill* :put :pre-support-grasp-coords
        (make-cascoords
         :coords (send (send (send *drill* :get :support-grasp-coords) :copy-coords) :translate (float-vector -100 (if (equal *drill-main-arm* :rarm) 100 -100) 0) :local)
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
        ;(robot-name "HRP2JSKNT")
        (drill-type *default-drill-type*) ;; not used now
        (recognized-coords-symbol :origin-coords))
  ;; generate robot
  (if (not (boundp '*robot-name*))
      (setq *robot-name* "HRP2JSKNT")
      )
  (generate-robot *robot-name* :generate-ri? generate-ri?)
  (if (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
      (progn
        (setq *use-dual-arm-ik* nil)
        (setq *use-f-sensor-search* nil)
        (setq *drill-reverse-hand* t)
        (setq *support-reverse-hand* nil)
        (setq *default-drill-type* :takenoko))
    (progn 
      (setq *use-dual-arm-ik* nil)
      (setq *use-f-sensor-search* nil)
      (setq *drill-reverse-hand* nil)
      (setq *support-reverse-hand* nil)
      (setq *default-drill-type* :gun))
    )
  
  
  ;; place robot to origin:
  (send *robot* :angle-vector (get-drill-reset-pose))
  
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :hand :arms :reset-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  (setq *arm-drill-grasp-coords* (make-hash-table))
  (setf (gethash :rarm *arm-drill-grasp-coords*)
        (make-cascoords
         :coords 
	 (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
		(send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 50.0 25.0 0.0) :local) :rotate (deg2rad 45) :z :local))
		(t
		 (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0.0 0.0 0.0) :local) :rotate (deg2rad 0) :z :local)
		 )
	       )
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  (setf (gethash :larm *arm-drill-grasp-coords*)
        (make-cascoords
         :coords 
	 (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
		(send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 50.0 -25.0 0.0) :local) :rotate (deg2rad -45) :z :local))
		(t
		 (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0.0 0.0 0.0) :local) :rotate (deg2rad 0) :z :local)
		 )		
		)
         :parent (send (send *robot* :larm :end-coords) :parent)))
  ;; (send *robot* :hand :arms :hook-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  ;; for hrp2jsknt
  ;; (send *robot* :put :support-arm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(121 -21 24) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand *drill-support-arm* :link "L_INDEXPIP_R_LINK")))
  ;; for hrp2jsknts
  ;; (send *robot* :put :support-arm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(120 -23 23) :local)
  ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
  ;; (send *robot* :put :support-arm-drill-button-coords
  ;;       (make-cascoords
  ;;        :coords (send (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(120 -0 0) :local)
  ;;                                  :rotate pi/2 :y :local) :rotate pi :z :local) :rotate 0 :z :local)
  ;;        :parent (send *robot* :hand *drill-support-arm* :link "L_INDEXPIP_R_LINK")))
  (if (not *support-reverse-hand*)
      (send *robot* :put :support-arm-drill-button-coords
	    (make-cascoords
;	     :coords (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(50 87.5 80) :local)
	     :coords (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(80 62 80) :local)
	     :parent (send (send *robot* *drill-support-arm* :end-coords) :parent)))
   (send *robot* :put :support-arm-drill-button-coords ;; ohara 4/12
        (make-cascoords
         :coords (send (send (send (send (send *robot* *drill-support-arm* :end-coords) :copy-worldcoords) :translate #f(120 62 -80) :local) :rotate (deg2rad 180) :y) :rotate (deg2rad -90) :z)
         :parent (send (send *robot* *drill-support-arm* :end-coords) :parent))))

  (cond ((equal *default-drill-type* :takenoko) (init-takenoko-drill))
        ((equal *default-drill-type* :gun) (init-gun-drill))
        )
  (when drill-coords 
     (send *drill* :newcoords (get-drill-coords-with-symbol drill-coords recognized-coords-symbol)))
    ;; drill setting
  (setq *drill-stand-coords* nil)
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
    (objects (list *robot* *drill* *ground*))
    (set-attachment)
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 0.5 0.5 1))
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
  (if (eq recognized-coords-symbol :put-coords)
      (let* ((mat (send (send *drill* :copy-worldcoords) :copy-rot))
             (uz (matrix-column mat 2)))
        (format t "uz ~A" uz)
        (if (< (v. uz (float-vector 0 0 1)) 0)
            (send *drill* :rotate (deg2rad 180) :x :local))
        )
    )
  ;treat the difference of origin between eus and recognition program
  ;(send *drill* :translate (float-vector 0 0 (* -0.5 (elt (send *drill* :body-type) 2))) :local)
  )

(defun fullbody-inverse-kinematics-for-drill-grasp
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
      (fullbody-inverse-kinematics-for-drill-grasp-for-hrp2 tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis))
	(t
	 (fullbody-inverse-kinematics-for-drill-grasp-for-jaxon tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis))
      )
  )
(defun fullbody-inverse-kinematics-for-drill-grasp-for-hrp2
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
(defun fullbody-inverse-kinematics-for-drill-grasp-for-jaxon
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)))
  (let (ret)
    (setq ret
          (send *robot*
                :fullbody-inverse-kinematics tc
                :move-target mt :link-list ll
                :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.01 0.15 0.5 0.5)
                :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                :translation-axis translation-axis
                :rotation-axis rotation-axis
                :warnp (if draw? t nil)
                :additional-weight-list
                (list (list (send *robot* :torso :waist-r :child-link) 0.1)
                      (list (send *robot* :torso :waist-y :child-link) 0.1)
                      (list (send *robot* :torso :waist-p :child-link) 0.1))
                :max (float-vector  500  200  25  10  20  10)
                :min (float-vector  -500  -200  -200  -10  -20  -10)
                :stop 50
                :debug-view (if draw? :no-message nil)
                ))
    (when (and ret (send *robot* :self-collision-check)) (setq ret nil))
    ret)
  )
(defun fullbody-inverse-kinematics-for-drill-grasp-single
  (&key (arm-symbol :grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (arm-mt (gethash *drill-main-arm* *arm-drill-grasp-coords*)) (draw? t) (rotation-axis (list :z t t)))
  (let 
      ((tc (list
           arm-tc
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* :rleg :lleg))
     (setf (elt mt 0) arm-mt)
     (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw? :rotation-axis rotation-axis))
     (when ik-ret
       (setq ik-ret
             (send *robot* :look-at-target (send *drill* :worldcoords)))))
    ik-ret))
(defun fullbody-inverse-kinematics-for-drill-grasp-dual
  (&key (arm-symbol :grasp-coords) (support-arm-symbol :support-grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (support-arm-tc (send (send *drill* :get support-arm-symbol) :copy-worldcoords)) (draw? t) (main-arm-coords (gethash *drill-main-arm* *arm-drill-grasp-coords*)) (suppor-arm-coords (gethash *drill-support-arm* *arm-drill-grasp-coords*)))
  (let 
      ((tc (list
           arm-tc
           support-arm-tc
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
     (setf (elt mt 0) main-arm-coords)
     (setf (elt mt 1) support-arm-coords)
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
      (send *ri* :reach-until-touch :larm (float-vector 1 -1 1) :max-dist (+ 0 120) :try-dist 250 :debug t :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t)
      )
    )
  )
(defun search-drill-support-grasp-place (&key (debug nil)) ;; thinks recognition 3cm diff
  (if (equal *drill-support-arm* :rarm) (send *ri* :reach-until-touch :rarm (float-vector 1 1 1) :max-dist (+ 0 120) :debug t :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t :try-dist 250)
    (send *ri* :reach-until-touch :larm (float-vector 1 0 1) :max-dist (+ 0 120) :debug debug :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t)
    )
  )
(defun generate-drill-grasp-motion
  (&key (draw? t) (rotation-axis (if (eq *default-drill-type* :takenoko)(list :z t t) (list t t t))))
  (let* ((reach-dist 140)
         ;(x-factor 0.0)
	 (grasp-up-dist 100)
	 (air-grasp-up-dist 30)
         (down-dist 25)
         (push-dist 0)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (original-drill-coords (send *drill* :copy-worldcoords))
         (drill-type *default-drill-type*)
         (fin? t))
    ;; grasp drill
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to grasp drill
    (block drill-ik
    (let* (ik-ret
           tc)
      (when draw? (send (send *drill* :get :grasp-coords) :draw-on :flush t))
      ;; solve ik for grasp
 
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis rotation-axis))
      (when (and fin? (not ik-ret)) (push-back (list 0 (not (null ik-ret)) ik-ret) ik-results) (return-from drill-ik nil))
      (let ((robot-angle-vector ik-ret))
	(when ik-ret 
	  (send *drill* :put :grasp-coords
		(make-cascoords
		 :coords (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords)
		 :parent *drill*)))
	(setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :translate (float-vector push-dist 0 (- air-grasp-up-dist)) :local) :draw? draw? :rotation-axis (list t t t))) ;; push!
	(push-back (list 3 (not (null ik-ret)) ik-ret) ik-results)
	(when (and fin? (not ik-ret)) (return-from drill-ik nil))
	;; solve ik for pre grasp
  	(send *robot* :angle-vector robot-angle-vector)
	(send *robot* :fix-leg-to-coords original-foot-midcoords)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :translate (float-vector push-dist 0 down-dist) :local) :draw? draw? :rotation-axis (list t t t)))
        (push (list 2 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
	(send *robot* :angle-vector robot-angle-vector)
	(send *robot* :fix-leg-to-coords original-foot-midcoords)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :translate (float-vector -30 0 down-dist) :local) :draw? draw? :rotation-axis (list t t t)))
        (push (list 1 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
	(send *robot* :angle-vector robot-angle-vector)
	(send *robot* :fix-leg-to-coords original-foot-midcoords)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :translate (float-vector (- reach-dist) 0 -0) :local) :draw? draw? :rotation-axis (list t t t)))
	(push (list 0 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
	(send *robot* :angle-vector robot-angle-vector)
	(send *robot* :fix-leg-to-coords original-foot-midcoords)
	;; solve ik for grasp up	
        (send *drill* :translate (float-vector 0 0 grasp-up-dist) :world)
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis rotation-axis))
	(push-back (list 4 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
	(send *robot* :angle-vector robot-angle-vector)
	(send *robot* :fix-leg-to-coords original-foot-midcoords)
	)
      (when *use-dual-arm-ik*
        (send *drill* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform (make-coords :pos (float-vector 450 0 1100)) :local))
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis rotation-axis))
	(push-back (list 5 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-dual :support-arm-symbol :pre-support-grasp-coords :draw? draw?))
	(push-back (list 6 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?))
	(push-back (list 7 (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
        )
      ;; draw
      )
    )
    (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; drill-reset-pose
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (send *drill* :newcoords original-drill-coords)
      (setq *drill-ik-results* ik-results)
      ))

(defun get-drill-grasp-motion
  (&key (draw? t) (rotation-axis (list :z t t)))
  (generate-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*)))
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-drill-grasp-stand-coords ;; relative to drill
  (&key (drill-type *default-drill-type*))
  (if (equal drill-type :takenoko)
      ;; (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad -0) 0 0))
      ;;     )
      (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad 30) 0 0)) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad -30) 0 0))
          )
    (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -600 100 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -600 -100 0) :rpy (list (deg2rad 0) 0 0))
          )
      )
  )
(defun get-drill-grasp-motion-full
  (drill-coords &key (draw? nil) (use-ri? t) (drill-type *default-drill-type*) (recognized-coords-symbol :origin-coords) (rotation-axis (if (eq drill-type :takenoko)(list :z t t) (list t t t))))
  (set-drill-environment-full drill-coords :draw? draw? :use-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  
  (let* ((drill-motion-av-list (get-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis))
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
                                                         :transform (get-drill-grasp-stand-coords :drill-type drill-type))))
                    (setq *drill-stand-coords* (make-coords :pos (float-vector (elt (send drill-grasp-coords :worldpos) 0) (elt (send drill-grasp-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-grasp-coords :rpy-angle)) 0) 0 0)))
                    )
                  )))))
      (ros::ros-warn "change stand position to ~a for drill motion.~%" *drill-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-stand-coords*)
      (when draw? (send *irtviewer* :draw-objects))
      (setq drill-motion-av-list (get-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis))
      )
    drill-motion-av-list
    ))

  

(defun send-drill-grasp-motion
  (coords &key (real? t) (drill-type *default-drill-type*) (rotation-axis (list :z t t)))
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords :drill-type drill-type :rotation-axis rotation-axis))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-grasp-motion))
    (when real? (send *ri* :start-impedance :arms :k-p 200 :d-p 180 :m-p 5 :m-r 1 :d-r 15 :k-r 20)) ;; start imp soft
    (drill-hook :arm :arms)
    (send *robot* :angle-vector (get-drill-reset-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      )
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    ;; first reaching
    ;; pre-reach
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    (send *robot* :angle-vector (elt drill-motion-av-list 1))
    (when real? (model2real :time 5000))
    (when real? (send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp hard
    ;; reach
    (if (and *use-f-sensor-search* (equal drill-type :gun))
        (when (not (search-drill-grasp-place)) ;; cannot solve ik
          (send *robot* :angle-vector (elt drill-motion-av-list 2))
          (when real? (model2real :time 2000))
          )
      (progn (send *robot* :angle-vector (elt drill-motion-av-list 2))
             (when real? (model2real :time 2000)))
      )
    (when real? (drill-air-grasp))
    (unix::usleep 500000)
    ;; reach up
    (send *robot* :angle-vector (elt drill-motion-av-list 3))
    (when real? (model2real :time 2000))
    ;grasp
    (when (and (equal drill-type :gun) real?) (drill-t-grasp))
    (unix::usleep 500000)
    (when real? (drill-full-grasp))
    ;;grasp-up
    ;(send *ri* :set-ref-force #f(0 0 -20) 1000 *drill-main-arm*)
    (send *robot* :angle-vector (elt drill-motion-av-list 4))
    (when real? (model2real :time 3000))
    (when *use-dual-arm-ik*
      (send *robot* :angle-vector (elt drill-motion-av-list 5))
      (when real? (model2real :time 3000))
      (send *robot* :angle-vector (elt drill-motion-av-list 6))
      (when real? (model2real :time 3000))
      ;;todo reach-until-touch! or some other.
      (send *robot* :angle-vector (elt drill-motion-av-list 7))
      (when real? (model2real :time 3000))
      (drill-t-grasp :arm *drill-support-arm*)
      (drill-full-grasp :arm *drill-support-arm*)
      ;(send *ri* :set-ref-force #f(0 0 -10) 1000 *drill-main-arm*)
      ;(send *ri* :set-ref-force #f(0 0 -10) 1000 *drill-support-arm*)
      ;;todo? change drill manip coords with some recog methods
      )
    
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      (if *support-reverse-hand* (send *robot* :angle-vector (get-watch-drill-pose)) (send *robot* :angle-vector (get-look-drill-pose)))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 5000)))
      )

    ;; ;; drill-reset-pose
    ;; (send *robot* :angle-vector (elt drill-motion-av-list 4))
    ;; (when real? (model2real :time 5000))
    )
  )

(defun publish-drill-grasp-motion
  (&key coords (step-time 400) (update? t) (drill-type *default-drill-type*))
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil :drill-type drill-type :draw? t))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (ros::ros-info "drill coords ~a" (send (send *drill* :get :grasp-coords) :copy-coords))
  (if *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground")
    (move-robot-model (make-coords) "ground"))
  
  (publish-motion-joint-states *drill-motion* :step-time step-time))



;; drill put
;; almost same with grasp at present
(defun send-drill-put-motion
  (coords &key (real? t) (drill-type *default-drill-type*))
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords :drill-type drill-type :recognized-coords-symbol :put-coords :draw? t :rotation-axis (list :z t t)))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-put-motion))
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    
    ;; first reaching
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
    (when *use-dual-arm-ik*
      (send *robot* :angle-vector (elt drill-motion-av-list 7))
      (when real? (model2real :time 3000))
      ;(send *ri* :set-ref-force #f(0 0 0) 1000 *drill-support-arm*)
      (drill-hook :arm *drill-support-arm*)
      (send *robot* :angle-vector (elt drill-motion-av-list 6))
      (when real? (model2real :time 3000))
      )
    ;; ;; drill-reset-pose
    ;; (send *robot* :angle-vector (elt drill-motion-av-list 3))
    ;; (when real? (model2real :time 5000))
    ;; put
    (send *robot* :angle-vector (elt drill-motion-av-list 4))
    (when real? (model2real :time 3000))      
    ;; put
    (send *robot* :angle-vector (elt drill-motion-av-list 3))
    (when real? (model2real :time 3000))
    ;(send *ri* :set-ref-force #f(0 0 0) 1000 *drill-main-arm*)
    ;; hook
    (drill-hook :arm *drill-main-arm*)
    ;; release
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    (send *robot* :angle-vector (get-reach-drill-pre-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    (send *robot* :angle-vector (get-drill-reset-pose))
    (unless (check-angle-vector-distance)
      (when real? (model2real :time 3000)))
    )
  )

(defun publish-drill-put-motion
  (&key coords (step-time 400) (update? t))
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil :recognized-coords-symbol :put-coords :draw? t :rotation-axis (list :z t t)))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (when *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground"))
  (setq *drill-motion* (reverse *drill-motion*))
  (publish-motion-joint-states *drill-motion* :step-time step-time))



;; drill button ;;;;
(defun set-drill-default-coords
  (&key (arm *drill-main-arm*))
  (send *drill* :newcoords (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :transform (send (send (send *drill* :get :grasp-coords) :copy-coords) :inverse-transformation)))
  )

(defun set-drill-button-environment-full
  (drill-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (drill-worldcoords)
    ;; place robot and drill to initial coords
    (set-drill-environment :drill-coords nil :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send (send *robot* *drill-main-arm* :end-coords) :dissoc *drill*)
    (if use-ri? (send *robot* :angle-vector (send *ri* :state :reference-vector)))
    (if drill-coords
        (progn (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform drill-coords :local))
               (send *drill* :put :grasp-coords
                     (make-cascoords
                      :coords (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords)
                      :parent *drill*)))
      (set-drill-default-coords)
      )

    ;(send (send *robot* *drill-main-arm* :end-coords) :assoc *drill*)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-drill-button
  (&key (x-offset 0) (y-offset 0) (z-offset 0) (draw? t))
  (let* (ret)
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* *drill-support-arm* :rleg :lleg))
     (setf (elt mt 0) (gethash *drill-main-arm* *arm-drill-grasp-coords*))
     (setf (elt mt 1) (send *robot* :get :support-arm-drill-button-coords))
     (setq tc
           (list
            ;(send *robot* *drill-main-arm* :end-coords :copy-worldcoords)
            (send (send *drill* :get :grasp-coords) :copy-worldcoords)
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
                 :rotation-axis (list t :z t t)
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
    (send *robot* :angle-vector (get-look-drill-pose))
    ;;(send *robot* :fix-leg-to-coords original-foot-midcoords)
    (send *robot* :fix-leg-to-coords (make-coords))
    (let* (ik-ret)
      ;; pre-push1
      ;; (send *robot* :angle-vector (get-look-drill-pose))
      ;; (send *robot* :fix-leg-to-coords (make-coords))
      ;; (setq ik-ret (send *robot* *drill-main-arm* :move-end-rot -20 :z :world))
      (if *drill-reverse-hand*
	  (if *support-reverse-hand*
	      ;; (send *drill* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform (make-coords :pos (float-vector 518 -180 1123) :rpy (list -3.01 -0.009 3.14)) :local))
	    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (make-coords :pos (float-vector 500 0 300) :rpy (list 0 (deg2rad -30) pi/2)) :local))
	    (send *drill* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform (make-coords :pos (float-vector 348.528 -110.496 1385.46) :rpy (list -2.963 0.413 -3.02)) :local)))
	    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (make-coords :pos (float-vector 500 0 300) :rpy (list 0 (deg2rad -30) pi/2)) :local)))
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset -100 :draw? draw?))
	
      (push-back (list 0 (not (null ik-ret)) (send *robot* :angle-vector)) ik-results)
      ;; pre-push2
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset -100 :draw? draw?))
      (push-back (list 1 (not (null ik-ret)) ik-ret) ik-results)
      ;; pre-push3
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset -10 :draw? draw?))
      (push-back (list 2 (not (null ik-ret)) ik-ret) ik-results)
      ;; push
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset 50 :draw? draw?))
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
      (when real? (send *ri* :start-impedance *drill-support-arm* :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 400))
      ;; first try
      ;; pre-push1
      (drill-full-grasp :arm :arms)
      (send *robot* :angle-vector (elt drill-motion-av-list 0))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 6000))
      ;; pre-push2
      (send *robot* :angle-vector (elt drill-motion-av-list 1))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 3000))
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
      ;; second try
      (dotimes (ix 3)
	(dotimes (iy 3)
	  (let ((button-ik nil) (width 5))
	    (send *robot* :fix-leg-to-coords (make-coords))
	    (push (fullbody-inverse-kinematics-for-drill-button :x-offset (+ 0 (* (- ix 1) width)) :y-offset (+ 0 (* (- iy 1) width)) :z-offset -35 :draw? nil) button-ik)
	    (push (fullbody-inverse-kinematics-for-drill-button :x-offset (+ 0 (* (- ix 1) width)) :y-offset (+ 0 (* (- iy 1) width)) :z-offset 50 :draw? nil) button-ik)
	    (when (and (elt button-ik 0) (elt button-ik 1))
	      (send *robot* :angle-vector (elt button-ik 1))
	      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
	      (when real? (model2real :time 1000)) ;;back
	      (send *robot* :angle-vector (elt button-ik 0))
	      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
	      (when real? (model2real :time 1500)) ;;push
	      )
	    )
	  )
	)
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

;; (defun send-drill-look-motion
;;   (&key (real? nil))
;;   ;; look
;;   (send *robot* :angle-vector (get-look-drill-pose))
;;   (send *robot* :hand *drill-main-arm* :angle-vector (get-grasp-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   (when real?
;;     (model2real :time 10000)
;;     (hand-model2real))
;;   )

;;(set-drill-button-environment :drill-coords nil :draw? t :generate-ri? t)
;;(send-drill-look-motion :real? t)
;;(send-drill-button-motion (get-primitive-marker-pose :frame-id "BODY"))

;; drill wall ;;;;

(defun set-drill-rotation (&key (deg 0))
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
    (send new-coords :rotate (deg2rad deg) :z :local)
    (send *drill* :newcoords (get-drill-coords-with-symbol new-coords :edge-coords))
    )
  )

(defun generate-drill-wall-motion
  (&key (draw? t) (drill-type *default-drill-type*))
  (let* (
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (fin? t)
	 drill-edge-mt
	 )
    (set-drill-default-coords)
    (setq drill-edge-mt 
	  (make-cascoords
	   :coords 
	   (send (send *drill* :get :edge-coords) :copy-worldcoords)
	   :parent (send (send *robot* :rarm :end-coords) :parent)))
    
    ;; wall drill
    (send *robot* :angle-vector (get-drill-reset-pose))
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to wall draill
    (let* (ik-ret
           tc
           (dt-coords-list (send *drill* :get :destination-coords-list))
           (dt-coords-length (length dt-coords-list)))
      (when draw? (send (send *drill* :get :grasp-coords) :draw-on :flush t))
      ;; solve ik for walls
      (ros::ros-info (format nil "dt-coords-list ~A" dt-coords-list))
      (block drill-ik
      (dotimes (i dt-coords-length)
        (send *drill* :newcoords (send (elt dt-coords-list i) :copy-worldcoords))
        ;;(send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-drill-coords-with-symbol (elt dt-coords-list i) :origin-coords) :local))
        (if (equal drill-type :gun) (set-drill-rotation :deg 0))
        (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
        (push-back (list (+ i 1) (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
        (when (or (equal i 0) (equal i (- dt-coords-length 1)))
          (send *drill* :translate (float-vector -150 0 0) :world)
          (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
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
      )))


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
  (&key (drill-type *default-drill-type*))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (if (equal drill-type :gun)
	     (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -680 0 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -580 0 0) :rpy (list (deg2rad 0) 0 0)))
	   (if *drill-reverse-hand*
	       (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -780 200 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -780 -200 0) :rpy (list (deg2rad 0) 0 0)))
	     (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -750 600 0) :rpy (list (deg2rad -20) 0 0)) (make-coords :pos (float-vector -750 -600 0) :rpy (list (deg2rad 20) 0 0)))
	     )
	   ))
	(t ;;JAXON
	 (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -850 200 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -850 -200 0) :rpy (list (deg2rad 0) 0 0)))
	 )
	)
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
                                                         :transform (get-drill-wall-stand-coords :drill-type drill-type))))
                    (setq *drill-stand-coords* (make-coords :pos (float-vector (elt (send drill-wall-coords :worldpos) 0) (elt (send drill-wall-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-wall-coords :rpy-angle)) 0) 0 0)))
                    )
                  )))))
      (ros::ros-warn "change stand position to ~a for drill motion.~%" *drill-stand-coords*)
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
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-reach-drill-pre-pose))
      (when real? (model2real :time 15000))
      (send *robot* :angle-vector (get-pierce-drill-pre-pose))
      (when real? (model2real :time 15000))
      )
    ;; walk
    (when *drill-stand-coords*
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    
    (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
    ;; pre-reach
    (dotimes (i (length drill-motion-av-list))
      (send *robot* :angle-vector (elt drill-motion-av-list i))
      (when real? (model2real :time 15000))
      )
    )
  )

(defun publish-drill-wall-motion
  (&key coords-list (step-time 400) (update? t) (drill-type *default-drill-type*))
  (when update?
    (get-drill-wall-motion-full coords-list :use-ri? nil :drill-type drill-type :recognized-coords-symbol :edge-coords :draw? t))
  (ros::ros-info "drill-wall-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (if *drill-stand-coords*
    (move-robot-model *drill-stand-coords* "ground")
    (move-robot-model (make-coords) "ground"))
  
  (publish-motion-joint-states *drill-motion* :step-time step-time))



;; drill util ;;;;

(defun get-reach-drill-pre-pose ()
  (cond ((equal *robot-name* "HRP2JSKNT")
	 (if *drill-reverse-hand*
	     #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -29.0596 -86.9801 78.8826 -136.118 101.324 -9.8847 55.9352 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
	     (send *robot* :reset-manip-pose)
	     )
	 )
	(t (send *robot* :reset-manip-pose))
	)
  )
(defun get-pierce-drill-pre-pose ()
  ;; (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 1.19212 -20.9065 -5.36626 -98.3812 50.0 6.28459 -83.1406 60.0 -17.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  (cond ((equal *robot-name* "HRP2JSKNT") 
	(if *drill-reverse-hand* #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -15.8902 -24.0028 13.0297 -91.879 5.61774 -4.8117 -73.0948 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
      (send *robot* :reset-manip-pose)
      ))
	(t (send *robot* :reset-manip-pose))
	)
  )
;; (defun get-push-drill-pre-pose ()
;; (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -23.0209 -7.67889 10.4985 -75.8274 97.2715 -28.3553 -71.5569 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
;;   )
;; (defun get-overhand-grasp-pre-pose ()
;; #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -130.0 -25.0 -5.0 80.0 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
;;   )
;; (defun get-grasp-drill-default-pose ()
;;   #f(-3.00133 2.02892 -27.2933 50.8864 -23.7853 -2.19867 0.0 -3.00129 2.03943 -27.2041 49.4034 -22.3915 -2.20917 0.0 39.4881 32.13 -45.0 17.2412 33.0021 -84.6379 -28.4436 -91.286 89.3123 6.3681 36.9771 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
;;   )
(defun get-look-drill-pose ()
  (cond ((equal *robot-name* "HRP2JSKNT")

	 ;; #f(-1.73786 12.7998 -28.717 58.3273 -27.2775 -3.19394 0.0 -1.75078 13.1089 -32.0242 64.2536 -29.8937 -3.50327 0.0 -18.0177 40.9471 -15.5394 -20.0 -71.9257 -1.63271 9.57461 -68.7006 -18.7947 12.8132 13.4518 60.0 -67.3083 63.2485 -56.2882 -89.6456 -32.2721 16.8311 -17.2177 -60.0))
	 ;; #f(-3.18801 14.0246 -30.3389 64.3646 -30.1868 -5.8666 0.0 -3.20998 14.342 -32.5966 67.7287 -31.2878 -6.18463 0.0 -6.39784 15.1167 -23.2174 -16.7713 -58.3745 -17.3128 21.0908 -111.942 -21.179 -21.2009 34.1876 60.0 -64.9891 55.4053 -56.0059 -114.503 -79.2957 19.8623 67.9176 -60.0)) ;; 04/05
#f(-3.18801 14.0246 -30.3389 64.3646 -30.1868 -5.8666 0.0 -3.20998 14.342 -32.5966 67.7287 -31.2878 -6.18463 0.0 -6.39784 15.1167 -23.2174 -16.7713 -63.8998 -26.7475 19.7525 -93.797 -60.1496 14.1419 -8.73053 60.0 -64.9891 55.4053 -56.0059 -114.503 -79.2957 19.8623 67.9176 -60.0)) ;; 04/21
	;;   ;;#f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 40.0 -16.9174 -3.95202 39.3961 -81.2715 -74.5661 -40.3588 -35.2676 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
	
	;; (defun get-test-drill-button-pose ()
	;;   #f(0.59214 1.27329 -31.1491 53.397 -22.4072 -0.218713 -7.103630e-32 0.592151 1.27702 -31.4606 54.2527 -22.9514 -0.222441 8.448493e-32 -5.36723 12.1833 -2.205876e-32 20.0 -32.1809 3.41271 19.6491 -63.67 -79.641 23.6402 -15.3413 60.0 -30.2688 9.72047 -28.8943 -97.1086 -3.57328 -24.9216 -6.62067 -60.0)
	;;   )
	(t (send *robot* :reset-manip-pose))
	)
  )
(defun get-drill-reset-pose (&key (arm *drill-main-arm*) (drill-type *default-drill-type*))
  (cond ((equal *robot-name* "HRP2JSKNT")
      (cond 
       ((not *drill-reverse-hand*)
	(send *robot* :reset-manip-pose))
       ((equal arm :rarm)
	#f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 20.0 49.2654 -15.6889 -8.04143 -131.198 78.4518 -8.73041 1.4443 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
       ((equal arm :larm) #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 49.3037 15.7767 8.00799 -131.215 -78.4251 8.75576 1.53598 -60.0)
	)
       (t #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 20.0 50.8516 -36.1267 0.371474 -131.17 72.8481 -13.9098 21.4063 60.0 50.8516 36.1267 -0.371464 -131.17 -72.8481 13.9098 21.4063 -60.0)))
      )
	(t
	 (send *robot* :reset-manip-pose))
	)
  )


(defun get-watch-drill-pose ()
  (cond ((equal *robot-name* "HRP2JSKNT")
	 (if *drill-reverse-hand* #f(-1.38133 5.04464 -28.5655 52.7906 -23.2636 -5.40806 0.0 -1.38071 5.00818 -27.1647 49.5024 -21.3763 -5.3716 0.0 22.1862 23.6363 -22.5578 43.5 -11.4883 -6.82302 22.4676 -115.948 82.417 18.2691 48.5902 60.0 -58.2284 33.3699 -40.8042 -81.4217 5.90815 -41.9868 -46.7235 -60.0)
	     #f(-1.38133 5.04464 -28.5655 52.7906 -23.2636 -5.40806 0.0 -1.38071 5.00818 -27.1647 49.5024 -21.3763 -5.3716 0.0 22.1862 23.6363 -22.5578 43.5 -2.7897 7.6894 8.91765 -101.289 -83.2938 0.700085 -30.2438 60.0 -58.2284 33.3699 -40.8042 -81.4217 5.90815 -41.9868 -46.7235 -60.0)
	     )
	 )
	(t
	 (send *robot* :reset-manip-pose))
	)
  )

(defun get-grasp-drill-hand-pose ()
  #f(90.0 40.0 5.0 10.0 100.0 50.0)
  ;;#f(90.0 40.0 5.0 10.0 45.0 50.0)
  )
(defun get-button-drill-hand-pose ()
  #f(90.0 90.0 0.0 0.0 0.0 115.0)
  ;;#f(90.0 0.0 0.0 0.0 0.0 115.0)
  )
;; (defun get-drill-test-pose ()
;;   #f(4.991507e-32 2.204884e-30 -26.0 50.0 -24.0 4.587941e-33 -1.378053e-31 2.552867e-32 1.247877e-32 -26.0 50.0 -24.0 -9.841308e-33 -6.315679e-31 3.719923e-32 -5.951877e-31 2.570109e-33 40.0 -3.98489 -10.7817 30.9963 -74.3163 -14.2952 -23.9135 -18.2156 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
;;   )
;; (defun get-drill-larm-test-pose()
;;   #f(6.353318e-47 1.511780e-45 -26.0 50.0 -24.0 -1.024111e-48 -9.448623e-47 9.628560e-48 1.588330e-47 -26.0 50.0 -24.0 -5.407690e-48 5.445592e-46 -3.615974e-47 5.785559e-46 -4.503218e-48 6.202082e-14 33.8752 -23.3947 -2.34573 -71.2747 -14.0104 2.82435 13.9053 15.0 17.6065 18.0935 -5.46659 -81.6239 14.0 5.0 -26.5981 -15.0)
;;   )

;; TODO pre-push-pose
(defun test-drill-grasp-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 1000 -150 100) :rpy (list (deg2rad 0) 0 0)) :draw? t :drill-type :takenoko))
  (if (equal *robot-name* "HRPJSKNT")
      (send *robot* :hand :arms :hook-pose))
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-send-drill-grasp-motion ()
  (send-drill-grasp-motion (make-coords :pos (float-vector 1000 -150 100) :rpy (list (deg2rad 0) 0 0)) :drill-type :takenoko :real? t :rotation-axis (list :z t t)))
  
(defun test-drill-put-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 1000 -150 200) :rpy (list 0 0 (deg2rad 180))) :draw? t :drill-type :takenoko :recognized-coords-symbol :put-coords :rotation-axis (list :z t t)))
  ;(send *robot* :hand :arms :hook-pose)
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-send-drill-put-motion ()
  (send-drill-put-motion (make-coords :pos (float-vector 1000 -150 200) :rpy (list 0 0 0)):drill-type :takenoko :real? t)
  )

(defun test-drill-button-motion ()
  (set-drill-button-environment-full nil)
  (setq *answer-for-grasp-test* (get-drill-button-motion-full nil :draw? t))
  )
(defun test-send-drill-button-motion ()
  (send-drill-button-motion nil :real? t)
  )
(defun test-drill-wall-motion ()
  (setq *drill-coords-list* (list (make-coords :pos (float-vector 1000 -150 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 500) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -50 500) :rpy (list 0 pi/2 0))))
  (setq *answer-for-grasp-test* (get-drill-wall-motion-full *drill-coords-list* :draw? t :drill-type :takenoko :recognized-coords-symbol :edge-coords))
  ;(send *robot* :hand :arms :hook-pose)
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-send-drill-wall-motion ()
  (send-drill-wall-motion (list (make-coords :pos (float-vector 1000 -150 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 500) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -50 500) :rpy (list 0 pi/2 0))) :real? t)
  )
(defun re-view ()
  (play-motion-on-eus-viewer *drill-motion*)
  )

(defun re-test ()
  (load "generate-drill-motion.l")
  (test-drill-grasp-motion)
  (send *robot* :angle-vector (elt *drill-motion* 1))
  )
(defun drill-hook (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
         (hand-model2real)
         )
	(t (send *ri* :stop-grasp :arm arm :wait t))
        )
  )
(defun drill-hook2 (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 70.0 -10.0 0.0 -40.0 -40.0))
         (hand-model2real)
         )
	(t (send *ri* :stop-grasp :arm arm :wait t))
        )
  )
(defun drill-hook-after (&key (arm *drill-main-arm*)) ;; test code
  (unix::sleep 5)
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
	 (hand-model2real)
	 )
	(t (send *ri* :stop-grasp :arm arm :wait t))
	)
  )
(defun drill-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 90.0))
	 (hand-model2real)
	 )
	(t (send *robot* :start-grasp :arm arm :wait t))
	)
  )
	
(defun drill-air-grasp (&key (arm *drill-main-arm*))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 45.0 -10.0 0.0 -40.0 40.0))
	 (hand-model2real)
	 )	
	(t (send *ri* :start-grasp :arm arm :wait t))
	)
  )

(defun drill-t-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 -10.0 0.0 -40.0 -40.0))
	 (hand-model2real)
	 )
	(t (send *ri* :start-grasp :arm arm :wait t))
	)
  )

(defun drill-full-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 0.0 0.0 90.0 90.0))
	 (hand-model2real)
	 )
	(t (send *ri* :start-grasp :arm arm :wait t))
	)
  )
(defun drill-full-grasp2 (&key (arm *drill-main-arm*)) ;; grasp without index finger
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
	 (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 -10.0 0.0 -40.0 90.0))
	 (hand-model2real)
	 )
	(t (send *ri* :start-grasp :arm arm :wait t))
	)
  )

(defun ik-drill (vec &key (time 1000) (real? nil))
  (send *drill* :newcoords (make-coords :pos vec))
  (fullbody-inverse-kinematics-for-drill-grasp-dual)
  (if real? (model2real :time time))
  )
(defun do-takenoko-motion (&key (x-dist 420) (y-dist -200) (real? t) (hook? nil))
  (set-drill-environment)
  (if (and real? hook?) (drill-hook :arm :rarm))
  (send *drill* :newcoords (send (make-coords :pos (float-vector (- x-dist 100) y-dist 1000)) :rotate (deg2rad 90) :y))  
  (fullbody-inverse-kinematics-for-drill-grasp-single :rotation-axis '(:z t t))
  (if real? (model2real :time 15000))
  (ros::ros-info (format nil "PUT DRILL"))
  (read-line)
  (if real? (drill-grasp :arm :arms))
  (if real? (drill-full-grasp :arm :larm))
  (ros::ros-info (format nil "OK?"))
  (read-line)
  (send *drill* :newcoords (send (make-coords :pos (float-vector (- x-dist 0) y-dist 1000)) :rotate (deg2rad 90) :y))  
  (fullbody-inverse-kinematics-for-drill-grasp-single :rotation-axis '(:z t t))
  (if real? (model2real :time 5000))
  (send *drill* :newcoords (send (make-coords :pos (float-vector (- x-dist 0) y-dist 1100)) :rotate (deg2rad 90) :y))  
  (fullbody-inverse-kinematics-for-drill-grasp-single)
  (if real? (model2real :time 5000))
  (send *drill* :newcoords (send (make-coords :pos (float-vector (- x-dist 0) y-dist 1200)) :rotate (deg2rad 90) :y))  
  (fullbody-inverse-kinematics-for-drill-grasp-single)
  (if real? (model2real :time 5000))
  (send *drill* :newcoords (send (make-coords :pos (float-vector (- x-dist 100) y-dist 1200)) :rotate (deg2rad 90) :y))  
  (fullbody-inverse-kinematics-for-drill-grasp-single
 :rotation-axis '(:z t t))
  (if real? (model2real :time 5000))  
  )

(defun do-gun-motion (&key (x-dist 480) (y-dist 0) (real? t) (hook? nil))
  (set-drill-environment)
  (if (and real? hook?) (drill-hook :arm :arms))
  ;(when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
  (send *drill* :newcoords (make-coords :pos (float-vector (- x-dist 100) y-dist 900)))
  (fullbody-inverse-kinematics-for-drill-grasp-dual)
  (if real? (model2real :time 15000))
  (ros::ros-info (format nil "PUT DRILL"))
  (read-line)
  (if real? (drill-grasp :arm :arms))
  (if real? (drill-full-grasp :arm :larm))
  (ros::ros-info (format nil "OK?"))
  (read-line)

  (ik-drill (float-vector x-dist y-dist 900) :time 5000 :real? real?)
  (dotimes (i 20)
    (ik-drill (float-vector (- x-dist 20) y-dist (+ 900 (* i 10))) :real? real?)
    (ik-drill (float-vector (- x-dist 0) y-dist (+ 900 (* i 10))) :real? real?)
    )
  (ik-drill (float-vector (- x-dist 100) y-dist 1100) :time 5000 :real? real?)
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
;;    (setf (elt mt 1) (send *robot* :get :support-arm-drill-button-coords))
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
;;    (setf (elt mt 1) (send *robot* :get :support-arm-drill-button-coords))
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
