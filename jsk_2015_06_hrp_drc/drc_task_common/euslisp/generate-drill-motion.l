(require "package://drc_task_common/euslisp/robot-util.l")
(require "package://drc_task_common/euslisp/util.l")
;(require "package://drc_task_common/models/dewalt_dc550ka.l")
(require "package://drc_task_common/models/takenoko-drill")
(require "package://drc_task_common/models/gun-drill.l")
(ros::roseus-add-srvs "drc_task_common")
(require "package://pr2eus/speak.l")


(setq *drill-main-arm* :larm)
(setq *use-dual-arm-ik* nil)
(setq *use-f-sensor-search* nil)
(setq *drill-reverse-hand* t)
(setq *drill-button-point* :finger) ;; one of (:attachment-r :attachment :finger)
(setq *default-drill-type* :takenoko) ;; one of (:takenoko :gun)

;; takenoko drill
(defun init-takenoko-drill ()
  (setq *drill* (takenoko-drill))
  ;; generate drill
  (send-all (send *drill* :bodies) :set-color #f(0.7 0.6 0.0))
  ;; atattch handle to drill
  (send *drill* :put :type :takenoko)
  (send *drill* :put :grasp-coords
        (make-cascoords
         ;; :coords (make-coords :pos #f(0 0 170) :rpy (if *drill-reverse-hand* (list (deg2rad 40) 0 pi) (list (deg2rad -40) 0 0)))
         :coords (make-coords
                  :pos (float-vector 0 0
                               (if *drill-reverse-hand*
                                   145 ;; 20150516 on rviz
                                 (if (equal *robot-name* "HRP2JSKNTS") 185 140)
                                 )
                               )
                  :rpy ;;depends on button motion
                  (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
                         (if *drill-reverse-hand*
                             (cond
                              ((equal *drill-button-point* :attachment-r)
                               (list (deg2rad -20) 0 pi))
                              ((equal *drill-button-point* :attachment)
                               (list (deg2rad 150) 0 pi))
                              ((equal *drill-button-point* :finger)
                               (list (deg2rad (if (equal *drill-main-arm* :rarm) 155 60)) 0 pi)))
                           (list (deg2rad (if (equal *drill-main-arm* :rarm) 80 160)) 0 0) ;; for finger only
                           ))
                        (t ;;jaxon
                         (if *drill-reverse-hand* (list (deg2rad (if (equal *drill-main-arm* :rarm) 150 30)) 0 pi) (list (deg2rad (if (equal *drill-main-arm* :rarm) 10 180)) 0 0))
                         )
                        ))
         ;; :coords (make-coords :pos #f(0 0 170) :rpy (if *drill-reverse-hand* (list (deg2rad 180) 0 pi) (list (deg2rad 90) 0 0)))
         :parent *drill*))
  (send *drill* :put :grasp-coords-true
        (make-cascoords
         :coords (send (send *drill* :get :grasp-coords) :copy-worldcoords)
         :parent *drill*
         )
        )
  (send *drill* :put :support-grasp-coords
        (make-cascoords
         :coords 
         (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNTS"))
                (send (send (make-coords :pos #f(0 0 0) :rpy (list 0 -pi/2 0)) :rotate (deg2rad 0) :x) :rotate (deg2rad 90) :z)
                )
               (t
                (make-coords :pos #f(0 0 0) :rpy (list 0 -0 0))
                ))
         :parent *drill*)
        )
  (send *drill* :put :pre-support-grasp-coords
        (make-cascoords
         :coords (send (send (send *drill* :get :support-grasp-coords) :copy-coords) :translate (float-vector -100 (if (equal *drill-main-arm* :rarm) 100 -100) 0) :local)
         :parent (send *drill* :get :support-grasp-coords)))
  (send *drill* :put :button-coords
        (make-cascoords
         :coords (make-coords :pos #f(-22 30 77) :rpy (list (deg2rad 22) 0 pi/2))
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
         :coords (make-coords :pos #f(0 0 260) :rpy (list 0 0 0))
         :parent *drill*))
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
  )

;; set drill coords with symble
(defun get-drill-coords-with-symbol (coords symbol)
  (ros::ros-info (format nil "got coords ~A symbol ~A drill ~A" (send *drill* :get symbol) symbol *drill*))
  (send (send coords :copy-worldcoords) :transform (send (send (send *drill* :get symbol) :copy-coords) :inverse-transformation))
  )

;; collision check wit drill (only with a few links)
(defun c-check-for-drill
  (&key (link-list (list (elt (send *robot* :head :links) 1) (elt (send *robot* :torso :links) 1) (elt (send *robot* *drill-main-arm* :links) 0))))
  (dotimes (i (length link-list))
    (when (equal (pqp-collision-check (elt link-list i) *drill*) 1)
        (ros::ros-info "collision detected")
        (return-from c-check-for-drill t)
        )
    )
  nil
  )

;; drill grasp ;;
;; initialize drill and robot
(defun set-drill-environment
  (&key (arm :rarm) (drill-coords (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? nil)
        ;(robot-name "HRP2JSKNT")
        (drill-type *default-drill-type*) ;; not used now
        (recognized-coords-symbol :origin-coords))
  ;; (setq *drill-main-arm* arm)
  ;; generate robot
  (if (not (boundp '*robot-name*))
      (setq *robot-name* "HRP2JSKNTS")
    )
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (setq *use-dual-arm-ik* nil)
         (setq *use-f-sensor-search* nil)
         (setq *drill-reverse-hand* t)
         (setq *drill-button-point* :finger)
         (setq *default-drill-type* :takenoko)
         )
        (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (setq *use-dual-arm-ik* nil)
         (setq *use-f-sensor-search* nil)
         (setq *drill-reverse-hand* nil)
         (setq *drill-button-point* :finger) ;; finger fixed
         (setq *default-drill-type* :takenoko)
         ))
  (setq *drill-ik-results* (list (list nil nil nil))) ;; to prevent unpredictable motion sent, not needed in reality
  (generate-robot *robot-name* :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :angle-vector (get-drill-reset-pose))
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :hand :arms :reset-pose) ;; this is neccesary for assoc coords to finger
  ;; attach handle to robot
  (attach-handle-to-robot-for-drill)
  ;; generate drill and attach handle to drill
  (cond ((equal *default-drill-type* :takenoko) (init-takenoko-drill))
        ((equal *default-drill-type* :gun) (init-gun-drill))
        )
  ;; place drill to drill-coords
  (when drill-coords
     (send *drill* :newcoords (get-drill-coords-with-symbol drill-coords recognized-coords-symbol)))
  ;; draw
  (setq *ground* (make-cube 1000 1000 1))
  (when (or draw? (boundp '*irtviewer*))
    (objects (list *robot* *drill* *ground*)))
  (when draw?
    (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)
          (send *irtviewer* :change-background (float-vector 0.5 0.5 1))
          )
    )
  )

;; initialize drill and robot, calculate drill coords
(defun set-drill-environment-full
  (drill-coords ;; body relative
   &key
   (arm :rarm)
   (draw? nil)
   (use-ri? t)
   (drill-type *default-drill-type*)
   (recognized-coords-symbol :origin-coords))
  ;; place robot and drill to initial coords
  (set-drill-environment :arm arm :drill-coords nil :draw? draw? :generate-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
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

(defun attach-handle-to-robot-for-drill
 ()
 (setq *arm-drill-grasp-coords* (make-hash-table))
 (setf (gethash :rarm *arm-drill-grasp-coords*)
       (make-cascoords
        :coords
        (cond ((equal *robot-name* "HRP2JSKNT")
               (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 50 2.0 0.0) :local) :rotate (deg2rad 53) :z :local))
              ((equal *robot-name* "HRP2JSKNTS")
               (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 50 25.0 0.0) :local) :rotate (deg2rad 53) :z :local))
              (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
               (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector -5.0 -10.0 0.0) :local) :rotate (deg2rad 0) :z :local)
               )
              )
        :parent (send (send *robot* :rarm :end-coords) :parent)))
 (setf (gethash :larm *arm-drill-grasp-coords*)
       (make-cascoords
        :coords
        (cond ((equal *robot-name* "HRP2JSKNT")
               (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 50 -2.0 0.0) :local) :rotate (deg2rad -53) :z :local))
              ((equal *robot-name* "HRP2JSKNTS")
               (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 50 -25.0 0.0) :local) :rotate (deg2rad -53) :z :local))
              (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
               (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector -5.0 10.0 0.0) :local) :rotate (deg2rad 0) :z :local)
               )
              )
        :parent (send (send *robot* :larm :end-coords) :parent)))
 ;; (send *robot* :hand :arms :hook-pose) ;; this is neccesary for assoc coords to finger
 ;; attach handle to robot
 ;; for hrp2jsknt
 ;; (send *robot* :put :support-arm-drill-button-coords
 ;;       (make-cascoords
 ;;        :coords (send (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(121 -21 24) :local)
 ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
 ;;        :parent (send *robot* :hand (get-opposite-arm *drill-main-arm*) :link "L_INDEXPIP_R_LINK")))
 ;; for hrp2jsknts
 ;; (send *robot* :put :support-arm-drill-button-coords
 ;;       (make-cascoords
 ;;        :coords (send (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(120 -23 23) :local)
 ;;                                  :rotate (deg2rad 25) :z :local) :rotate pi/2 :x :local) :rotate 0 :z :local)
 ;; (send *robot* :put :support-arm-drill-button-coords
 ;;       (make-cascoords
 ;;        :coords (send (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(120 -0 0) :local)
 ;;                                  :rotate pi/2 :y :local) :rotate pi :z :local) :rotate 0 :z :local)
 ;;        :parent (send *robot* :hand (get-opposite-arm *drill-main-arm*) :link "L_INDEXPIP_R_LINK")))
 (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
        (cond ((equal *drill-button-point* :finger) ;; finger
               (send *robot* :put :support-arm-drill-button-coords
                     (make-cascoords
                      :coords (send (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(130 0 -35) :local) :rotate (deg2rad 90) :y) :rotate (deg2rad (if (equal *drill-main-arm* :rarm) 180 0)) :z) :rotate (deg2rad (if *drill-reverse-hand* 0 180)) :x :world)
                      :parent (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :parent))))

              ((equal *drill-button-point* :attachment)
               (send *robot* :put :support-arm-drill-button-coords
                     (make-cascoords
                      :coords (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(80 62 80) :local) :rotate (deg2rad 135) :z)
                      :parent (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :parent))))
              (t
               (send *robot* :put :support-arm-drill-button-coords ;; ohara 4/12
                     (make-cascoords
                      :coords (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(120 62 -80) :local) :rotate (deg2rad 180) :y) :rotate (deg2rad -90) :z)
                      :parent (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :parent))))
              ))
       (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
        (send *robot* :put :support-arm-drill-button-coords
              (make-cascoords
                                        ;             :coords (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate #f(50 87.5 80) :local)
               :coords (send (send (send (send (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :copy-worldcoords) :translate (float-vector 80 (if (equal *drill-main-arm* :rarm) -65 65) 0) :local) :rotate (deg2rad (if (equal *drill-main-arm* :rarm) 90 90)) :y :local) :rotate (deg2rad (if (equal *drill-main-arm* :rarm) 180 180)) :z :local) :rotate (deg2rad (if (equal *drill-main-arm* :rarm) 0 180)) :z :local)
               :parent (send (send *robot* (get-opposite-arm *drill-main-arm*) :end-coords) :parent)))
        )
       )
 )

;; drill ik with tc, mt...
(defun fullbody-inverse-kinematics-for-drill-grasp
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)) (move-centroid t) (self-colision-check t) stop)
  (let
      ((tmp-angle-vector (send *robot* :angle-vector)) ret)
    (setq ret
          (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
                 (if stop
                     (fullbody-inverse-kinematics-for-drill-grasp-for-hrp2 tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis :stop stop)
                   (fullbody-inverse-kinematics-for-drill-grasp-for-hrp2 tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis)
                   ))
                (t
                 (if stop
                     (fullbody-inverse-kinematics-for-drill-grasp-for-jaxon tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis :stop stop)
                   (fullbody-inverse-kinematics-for-drill-grasp-for-jaxon tc mt ll :draw? draw? :translation-axis translation-axis :rotation-axis rotation-axis))
                 ))
          )
    (if (and ret move-centroid)
        (setq ret (send *robot* :move-centroid-on-foot :both '(:lleg :rleg :larm :rarm)))
      )
    (if (and ret self-colision-check (send *robot* :self-collision-check))
        (progn
          (setq ret nil)
          (ros::ros-info (format nil "self collision found ~A" (send *robot* :self-collision-check)))
          )
        )
    (when (not ret) (send *robot* :angle-vector tmp-angle-vector))
    ret
    )
  )
(defun fullbody-inverse-kinematics-for-drill-grasp-for-hrp2
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)) (stop 60))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0)
              (list (send *robot* :torso :waist-p :child-link) 0.1)
              (list (send *robot* :torso :waist-y :child-link) 0.2)
              )
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis translation-axis
        :rotation-axis rotation-axis
        :max (float-vector  500  500  0  20  20  10) ;; for hrp2
        :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
        :stop stop
        :debug-view (if draw? :no-message nil)
        )
  )
(defun fullbody-inverse-kinematics-for-drill-grasp-for-jaxon
  (tc mt ll &key (draw? nil) (translation-axis (list t t t)) (rotation-axis (list t t t)) (stop 50))
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
                :stop stop
                :debug-view (if draw? :no-message nil)
                :avoid-collision-distance 100
                :avoid-collision-null-gain 5.0
                :avoid-collision-joint-gain 0.8
                :collision-avoidance-link-pair
                (list (list (elt (send *robot* *drill-main-arm* :links) 2) (send *robot* :link "CHEST_LINK2")))
                ))
    (when (and ret (send *robot* :self-collision-check)) (setq ret nil))
    ret)
  )
;; ik for drill (use *drill* coords)
(defun fullbody-inverse-kinematics-for-drill-grasp-single
  (&key (arm-symbol :grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (arm-mt (gethash *drill-main-arm* *arm-drill-grasp-coords*)) (draw? t) (rotation-axis (list :z t t)) stop)
  (let
      ((tc (list
           arm-tc
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords)))
       ik-ret)
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* :rleg :lleg))
     (setf (elt mt 0) arm-mt)
     (if stop
         (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw? :rotation-axis rotation-axis :stop stop))
       (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw? :rotation-axis rotation-axis))
       )
     (when ik-ret
       (setq ik-ret
             (send *robot* :look-at-target (send *drill* :worldcoords)))))
    ik-ret))
(defun fullbody-inverse-kinematics-for-drill-grasp-dual
  (&key (arm-symbol :grasp-coords) (support-arm-symbol :support-grasp-coords) (arm-tc (send (send *drill* :get arm-symbol) :copy-worldcoords)) (support-arm-tc (send (send *drill* :get support-arm-symbol) :copy-worldcoords)) (draw? t) (main-arm-coords (gethash *drill-main-arm* *arm-drill-grasp-coords*)) (support-arm-coords (gethash (get-opposite-arm *drill-main-arm*) *arm-drill-grasp-coords*)) (rotation-axis (list t t t t)))
  (let
      ((tc (list
            arm-tc
            support-arm-tc
            (send *robot* :rleg :end-coords :copy-worldcoords)
            (send *robot* :lleg :end-coords :copy-worldcoords)))
       ik-ret)
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* (get-opposite-arm *drill-main-arm*) :rleg :lleg))
     (setf (elt mt 0) main-arm-coords)
     (setf (elt mt 1) support-arm-coords)
     (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp tc mt ll :draw? draw? :translation-axis (list t t t t) :rotation-axis rotation-axis))
     (when ik-ret
       (setq ik-ret
             (send *robot* :look-at-target (send *drill* :worldcoords)))))
    ik-ret))

;; use f sensor and grasp (for gun drill)
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

;; (for gun drill)
(defun search-drill-support-grasp-place (&key (debug nil)) ;; thinks recognition 3cm diff
  (if (equal (get-opposite-arm *drill-main-arm*) :rarm) (send *ri* :reach-until-touch :rarm (float-vector 1 1 1) :max-dist (+ 0 120) :debug t :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t :try-dist 250)
    (send *ri* :reach-until-touch :larm (float-vector 1 0 1) :max-dist (+ 0 120) :debug debug :thre 0.1 :dist-step 0.5 :time-step 20 :p-control t)
    )
  )

;; genetate drill motion (with current drill coords)
(defun generate-drill-grasp-motion
  (&key (draw? t) (rotation-axis (if (eq *default-drill-type* :takenoko) (list t t t) (list t t t))) (fin? t))
  (let* ((reach-dist 150)
         ;(x-factor 0.0)
         (grasp-up-dist 120)
         (air-grasp-up-dist (if (or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT")) 60 0))
         (down-dist (if (or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT")) 15 0))
         (push-dist (if (or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT")) 0 -10))
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (original-drill-coords (send *drill* :copy-worldcoords))
         (drill-type *default-drill-type*)
         )
    (when (not *drill-reverse-hand*)
      (setq down-dist (- down-dist))
      (setq air-grasp-up-dist (- air-grasp-up-dist))
      )
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
                 :parent *drill*))
          )
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
        (setq ik-ret (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :translate (float-vector (- reach-dist) 0 (if (not *drill-reverse-hand*) 100 0)) :local) :draw? draw? :rotation-axis (list t t t)))
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
      )
    )
    ;; draw
    (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t)))
    ;; drill-reset-pose
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    (send *drill* :newcoords original-drill-coords)
    (setq *drill-ik-results* ik-results)
    ))
;; calls generate-drill-grasp-motion and change results to *drill-motion*(global)
(defun get-drill-grasp-motion
  (&key (draw? t) (rotation-axis (list :z t t)))
  (generate-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *drill-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *drill-ik-results*)))
    (setq *drill-motion* angle-vector-ret)
    angle-vector-ret
    ))
(defun get-drill-grasp-stand-coords-full
  (drill-coords)
  (let (drill-oriented-coords)
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
            (setq drill-oriented-coords (make-coords :pos (send drill-coords :worldpos)
							 :rot mat))))))
                                        ;(setq drill-oriented-coords (send (send drill-coords :copy-worldcoords) :rotate (deg2rad -90) :z))
    (let ((drill-grasp-coords (send (send drill-oriented-coords :copy-worldcoords)
                                    :transform (get-drill-grasp-stand-coords))))
      (make-coords :pos (float-vector (elt (send drill-grasp-coords :worldpos) 0) (elt (send drill-grasp-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-grasp-coords :rpy-angle)) 0) 0 0)))
    )
  )
(defun get-drill-grasp-stand-coords ;; relative to drill
  (&key (drill-type *default-drill-type*))
  (if (equal drill-type :takenoko)
      ;; (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -650 0 0) :rpy (list (deg2rad -0) 0 0))
      ;;     )
      (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNTS"))
             (if *drill-reverse-hand*
                 (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -680 0 0) :rpy (list (deg2rad 44) 0 0)) (make-coords :pos (float-vector -680 -0 0) :rpy (list (deg2rad -44) 0 0)))
               (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -610 -300 0) :rpy (list (deg2rad 55) 0 0))
                 (make-coords :pos (float-vector -710 200 0) :rpy (list (deg2rad -35) 0 0)))) ;;
             )
            (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
             (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -710 -0 0) :rpy (list (deg2rad 35) 0 0)) (make-coords :pos (float-vector -710 0 0) :rpy (list (deg2rad -35) 0 0))))
            )
    (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -800 0 0) :rpy (list (deg2rad 10) 0 0)) (make-coords :pos (float-vector -800 0 0) :rpy (list (deg2rad -10) 0 0))
          )
      )
  )

;; get-drill-grasp-motion with drill-coords(Body Relative)
(defun get-drill-grasp-motion-full
  (drill-coords &key (arm *drill-main-arm*) (draw? nil) (use-ri? t) (drill-type *default-drill-type*) (recognized-coords-symbol :origin-coords) (rotation-axis (if (eq drill-type :takenoko)(list :z t t) (list t t t))) (overwrite-stand-coords? t) (stand-coords nil))
  (setq *drill-stand-coords* (or stand-coords (make-coords)))
  ;(ros::ros-info (format nil "default-stand-coords ~A" *drill-stand-coords*))
  ;(setq *drill-main-arm* arm)
  ;(setq (get-opposite-arm *drill-main-arm*) (get-opposite-arm arm))
  (set-drill-environment-full drill-coords :arm arm :draw? draw? :use-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  (setq *drill-stand-coords-forced* (get-drill-grasp-stand-coords-full (send *drill* :copy-worldcoords))) 
  ;(ros::ros-info (format nil "default-stand-coords ~A" *drill-stand-coords*))
  (send *robot* :fix-leg-to-coords *drill-stand-coords*)
  (let* ((drill-motion-av-list (if (and (equal overwrite-stand-coords? :force) (check-stand-coords *drill-stand-coords-forced* :pos-thre 0.1 :rpy-thre 4)) nil (get-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis)))
         (drill-motion-length (length drill-motion-av-list)))
    (when (and overwrite-stand-coords? (= drill-motion-length 0))
      (setq *drill-stand-coords* *drill-stand-coords-forced*)
      (ros::ros-warn "change stand position to ~a for drill motion.~%" *drill-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-stand-coords*)
      (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
      (setq drill-motion-av-list (get-drill-grasp-motion :draw? draw? :rotation-axis rotation-axis))
      )
    drill-motion-av-list
    ))

;; send drill motion for real robot(fc side)
(defun send-drill-grasp-motion
  (coords &key (real? t) (drill-type *default-drill-type*) (rotation-axis (list :z t t)) (arm *drill-main-arm*) (stand-coords nil) (overwrite-stand-coords? nil) (draw? nil)
          (drill-rotate-num nil))
  (objects (list *robot*));;for test
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (when stand-coords
    (setq stand-coords (make-coords :pos (float-vector (elt (send stand-coords :worldpos) 0) (elt (send stand-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send stand-coords :rpy-angle)) 0) 0 0)))
    )
  (let* ((drill-motion-av-list (or 
                                nil ;(get-drill-grasp-motion-full coords :drill-type drill-type :rotation-axis (list t t t) :arm arm :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?) 
                                (get-drill-grasp-motion-full coords :drill-type drill-type :rotation-axis rotation-axis :arm arm :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords? :draw? draw?)))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-grasp-motion))
    (when real?
      (set-default-impedance-param)
      (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
             (send *ri* :start-impedance :arms :k-p 200 :d-p 280 :m-p 5 :m-r 1 :d-r 15 :k-r 20)) ;; start imp soft
             ;(send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp hard
            (t
             (send *ri* :start-impedance :arms :k-p 200 :d-p 1000 :m-p 5 :m-r 1 :d-r 15 :k-r 20))))
    (send *ri* :set-ref-force #f(0 0 0) 2000 *drill-main-arm*)
    (send *ri* :set-ref-force #f(0 0 0) 2000 (get-opposite-arm *drill-main-arm*))
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
    (when (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    ;; first reaching
    ;; pre-reach
    (send *robot* :angle-vector (elt drill-motion-av-list 0))
    (when real? (model2real :time 5000))
    (when real?
      (set-default-impedance-param)
      (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
             (send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp hard
            (t
             (send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 1800))))
    (send *robot* :angle-vector (elt drill-motion-av-list 1))
    (when real? (model2real :time 5000))
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

    (send *robot* :angle-vector (elt drill-motion-av-list 4))
    (when real? (model2real :time 3000))
    (if (not (set-ref-force-with-condition)) (return-from send-drill-grasp-motion nil))
          
    ;(if (equal drill-type :takenoko) (rotate-drill-horizontal))
    (when *use-dual-arm-ik*
      (send *robot* :angle-vector (elt drill-motion-av-list 5))
      (when real? (model2real :time 3000))
      (send *robot* :angle-vector (elt drill-motion-av-list 6))
      (when real? (model2real :time 3000))
      ;;todo reach-until-touch! or some other.
      (send *robot* :angle-vector (elt drill-motion-av-list 7))
      (when real? (model2real :time 3000))
      (drill-t-grasp :arm (get-opposite-arm *drill-main-arm*))
      (drill-full-grasp :arm (get-opposite-arm *drill-main-arm*))
      (send *ri* :set-ref-force #f(0 0 -10) 2000 *drill-main-arm*)
      (send *ri* :set-ref-force #f(0 0 -10) 2000 (get-opposite-arm *drill-main-arm*))
      ;;todo? change drill manip coords with some recog methods
      )
    ;; (when (equal drill-type :takenoko)
    ;;   (send *robot* :angle-vector (get-reach-drill-pre-pose))
    ;;   (unless (check-angle-vector-distance)
    ;;     (when real? (model2real :time 3000))))
    ;;   (if (equal *drill-button-point* :attachment-r) (send *robot* :angle-vector (get-watch-drill-pose)) (send *robot* :angle-vector (get-pre-drill-button-pose)))
    ;;   (unless (check-angle-vector-distance)
    ;;     (when real? (model2real :time 5000)))
    ;;   )
    (let ((default-drill-coords (send *drill* :copy-worldcoords)))
      (when drill-rotate-num
        ;; (while (and (> drill-rotate-num 0) (> (abs (check-diff-angle)) 20))
        (send *robot* :angle-vector (send *ri* :state :potentio-vector))
        (send *robot* :fix-leg-to-coords (or *drill-stand-coords* (make-coords)))
        (set-drill-default-coords)
        (send-drill-rotate-motion (get-body-relative-coords (send *drill* :copy-worldcoords)) (get-body-relative-coords default-drill-coords) :drill-rotate-num drill-rotate-num :mid-pose? nil)
        ;(setq drill-rotate-num (- drill-rotate-num 1))
                                        ;          )
        )
      )
    ;; ;; drill-reset-pose
    ;; (send *robot* :angle-vector (elt drill-motion-av-list 4))
    ;; (when real? (model2real :time 5000))
    )
  )
(defun set-ref-force-with-condition
  ()
  ;(return-from set-ref-force-with-condition t);; for test

  (if (< (elt (send *ri* :state :absolute-force-vector *drill-main-arm*) 2) -7)
      (progn
        (speak-en "grasp succeeded, add ref force")
        (send *ri* :set-ref-force #f(0 0 -20) 2000 *drill-main-arm*)
        t
        )
    (progn
      (speak-en "grasp failed, not add ref force")
      (send *ri* :set-ref-force #f(0 0 0) 2000 *drill-main-arm*)
      nil
      )
    )
  )

;; publish drill motion in rviz(ocs-side)
(defun publish-drill-grasp-motion
  (&key coords (step-time 400) (update? t) (drill-type *default-drill-type*) (arm *drill-main-arm*) (stand-coords nil) (overwrite-stand-coords? t) (publish? t) (drill-rotate-num 0)
        (use-ri? nil) ;; for test
        (draw? nil)
        )
  (setq *drill-diff-angle* 0)
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (when update?
    (or
     nil ;;(get-drill-grasp-motion-full coords :arm arm :use-ri? nil :drill-type drill-type :draw? t :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?  :rotation-axis (list t t t)) 
     (get-drill-grasp-motion-full coords :arm arm :use-ri? use-ri? :drill-type drill-type :draw? draw? :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?)))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (ros::ros-info "drill coords ~a" (send (send *drill* :get :grasp-coords) :copy-coords))
  (setq *drill-diff-angle* (check-diff-angle))
  (when publish?
    (if (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
        (move-robot-model *drill-stand-coords* "ground")
      (move-robot-model (make-coords) "ground"))    
    (publish-motion-joint-states *drill-motion* :step-time step-time)
    )
  (when (and *drill-motion* (> drill-rotate-num 0))
    (let ((angle-vector (send *robot* :angle-vector)))
      (cond (use-ri?
             (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
      (send *robot* :fix-leg-to-coords (or *drill-stand-coords* (make-coords)))
      (let ((default-drill-coords (send *drill* :copy-worldcoords)))
        (set-drill-default-coords)
        (publish-drill-rotate-motion :drill-coords (get-body-relative-coords (send *drill* :copy-worldcoords)) :plane-coords (get-body-relative-coords default-drill-coords) :use-ri? use-ri? :angle-vector angle-vector :publish? publish? :move-robot-model? nil :draw? draw?)
        )
      )
    )
  )


;; drill put
;; almost same with grasp(at present)
(defun send-drill-put-motion
  (coords &key (arm *drill-main-arm*) (real? t) (drill-type *default-drill-type*)
          (stand-coords nil) (overwrite-stand-coords? nil)
          (draw? nil))
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (let* ((drill-motion-av-list (get-drill-grasp-motion-full coords :drill-type drill-type :arm arm :recognized-coords-symbol :put-coords :draw? draw? :rotation-axis (list :z t t) :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-put-motion))
    (let (c1 c2
             (original-stand-coords (send *robot* :foot-midcoords))
             (original-av (send *robot* :angle-vector)))
      (send *robot* :angle-vector (elt drill-motion-av-list 4))
      (send *robot* :fix-leg-to-coords (make-coords))
      (setq c1 (send *robot* *drill-main-arm* :end-coords :copy-worldcoords))
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *robot* :angle-vector (send *ri* :state :potentio-vector))
      ;(send *robot* :angle-vector (send *robot* :angle-vector))
      (setq c2 (send *robot* *drill-main-arm* :end-coords :copy-worldcoords))
      (ros::ros-info (format nil "c1: ~A, c2: ~A" c1 c2))
      (when (or 
             (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*)) 
             (not (check-coords-distance c1 c2 :pos-thre 300 :rpy-thre (deg2rad 20)))
             )
        (send *robot* :angle-vector (get-reach-drill-pre-pose))
        (when real? (model2real :time 3000))
        )
      (send *robot* :angle-vector original-av)
      (send *robot* :fix-leg-to-coords original-stand-coords)            
      )
    ;; walk
    (when (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))

    ;; first reaching
    (when real?
      (set-default-impedance-param)
      (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
             (send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp hard
            (t
             (send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 1800))))

    (when *use-dual-arm-ik*
      (send *robot* :angle-vector (elt drill-motion-av-list 7))
      (when real? (model2real :time 3000))
      (send *ri* :set-ref-force #f(0 0 0) 2000 (get-opposite-arm *drill-main-arm*))
      (drill-hook :arm (get-opposite-arm *drill-main-arm*))
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
    (send *ri* :set-ref-force #f(0 0 0) 2000 *drill-main-arm*)
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
  (&key coords (step-time 400) (update? t) (arm *drill-main-arm*) (stand-coords nil) (overwrite-stand-coords? t) (draw? nil))
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (when update?
    (get-drill-grasp-motion-full coords :use-ri? nil :recognized-coords-symbol :put-coords :draw? draw? :rotation-axis (list :z t t) :arm arm :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?))
  (ros::ros-info "drill-grasp-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (when (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
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
    ;; (if use-ri? (send *robot* :angle-vector (send *ri* :state :reference-vector)))
    (if drill-coords
        (progn (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform drill-coords :local))
               (send *drill* :put :grasp-coords
                     (make-cascoords
                      :coords (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords)
                      :parent *drill*)))
      (set-drill-default-coords)
      )

    ;(send (send *robot* *drill-main-arm* :end-coords) :assoc *drill*)
    (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
    ))

(defun fullbody-inverse-kinematics-for-drill-button
  (&key (x-offset 0) (y-offset 0) (z-offset 0) (draw? t) (self-colision-check t) (stop 150))
  (let* (ret)
    (with-move-target-link-list
     (mt ll *robot* (list *drill-main-arm* (get-opposite-arm *drill-main-arm*) :rleg :lleg))
     (setf (elt mt 0) (gethash *drill-main-arm* *arm-drill-grasp-coords*))
     (setf (elt mt 1) (send *robot* :get :support-arm-drill-button-coords))
     (setq tc
           (list
            (send (send *drill* :get :grasp-coords) :copy-worldcoords)
            ;(send *robot* *drill-main-arm* :end-coords :copy-worldcoords)
            (send (send (send (send *drill* :get :button-coords) :copy-worldcoords)
                        :translate (float-vector x-offset y-offset z-offset) :local) :rotate (deg2rad 0);150)
                        :z :local)

            (send *robot* :rleg :end-coords :copy-worldcoords)
            (send *robot* :lleg :end-coords :copy-worldcoords)))
     (setq ret
           (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
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
                        :stop stop
                        :centroid-thre 50
                        :debug-view (if draw? :no-message nil)
                        :warnp (if draw? t nil)
                        ))
                 (t
                  (send *robot*
                        :fullbody-inverse-kinematics tc
                        :move-target mt :link-list ll
                        :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.01 0.15 0.5 0.5)
                        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                        :translation-axis (list t t t t)
                        :rotation-axis (list t t t t)
                        :warnp (if draw? t nil)
                        :additional-weight-list
                        (list (list (send *robot* :torso :waist-r :child-link) 0.1)
                              (list (send *robot* :torso :waist-y :child-link) 0.1)
                              (list (send *robot* :torso :waist-p :child-link) 0.1))
                        :max (float-vector  500  200  25  10  20  10)
                        :min (float-vector  -500  -200  -200  -10  -20  -10)
                        :stop stop
                        :debug-view (if draw? :no-message nil)))
                 ))
    (if (and ret self-colision-check (send *robot* :self-collision-check))
        (progn
          (setq ret nil)
          (ros::ros-info (format nil "self collision found ~A" (send *robot* :self-collision-check)))
          )
        )
    (when ret
      (let (look-at-ret)
        (setq look-at-ret (send *robot* :look-at-target (send *drill* :worldcoords)))
        (when look-at-ret (setq ret look-at-ret))
        )
      )
     ret)
    )
  )

(defun generate-drill-button-motion
  (&key (draw? t) (fin? t))
  (let* (ik-results
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; push buton
    (send *robot* :angle-vector (get-pre-drill-button-pose))
    ;;(send *robot* :fix-leg-to-coords original-foot-midcoords)
    (send *robot* :fix-leg-to-coords (make-coords))
    (block drill-ik
    (let* (ik-ret
           )
      ;; pre-push1
      ;; (send *robot* :angle-vector (get-pre-drill-button-pose))
      ;; (send *robot* :fix-leg-to-coords (make-coords))
      ;; (setq ik-ret (send *robot* *drill-main-arm* :move-end-rot -20 :z :world))
      ;; place drill
      (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
             (if *drill-reverse-hand*
                 (cond ((equal *drill-button-point* :attachment-r)
                        (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (make-coords :pos (float-vector 500 0 300) :rpy (list 0 (deg2rad -30) pi/2)) :local)))
                       ((equal *drill-button-point* :attachment)
                        (send *drill* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform (make-coords :pos (float-vector 348.528 -110.496 1385.46) :rpy (list -2.963 0.413 -3.02)) :local)))
                       (t
                        (send *drill* :newcoords (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :transform (make-coords :pos (float-vector 348.528 0 1385.46) :rpy (list (deg2rad 240) (deg2rad 10) (deg2rad 240))) :local)))
                       )
               (send *drill* :newcoords (make-coords :pos (float-vector 250 -320 940) :rpy (list (deg2rad -40) 0 0)))
             ))
            (t
             (if *drill-reverse-hand*
                 (send *drill* :newcoords (make-coords :pos (float-vector 388 -110 1285) :rpy (list -2.828 0.17 -2.9)))
               (send *drill* :newcoords (send (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (make-coords :pos (float-vector 550 -100 100) :rpy (list 0 (deg2rad -10) (deg2rad 20))) :local) :rotate (deg2rad 20) :z))
               )
             )
            )
      ;; end place drill
      (if (equal *drill-main-arm* :larm) (mirror-drill))
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset -100 :draw? draw?))
      (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
      (push-back (list 0 (not (null ik-ret)) (send *robot* :angle-vector)) ik-results)
      (when (and fin? (not ik-ret)) (return-from drill-ik nil))
      ;; pre-push2
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset -50 :draw? draw?))
      (push-back (list 1 (not (null ik-ret)) ik-ret) ik-results)
      (when (and fin? (not ik-ret)) (return-from drill-ik nil))
      ;; pre-push3
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset 0 :draw? draw?))
      (push-back (list 2 (not (null ik-ret)) ik-ret) ik-results)
      (when (and fin? (not ik-ret)) (return-from drill-ik nil))
      ;; push
      (setq ik-ret (fullbody-inverse-kinematics-for-drill-button :x-offset 0 :y-offset 0 :z-offset 40 :draw? draw?))
      (push-back (list 3 (not (null ik-ret)) ik-ret) ik-results)
      (when (and fin? (not ik-ret)) (return-from drill-ik nil))
      )
    )
    (setq *drill-ik-results* ik-results)
    (send *robot* :fix-leg-to-coords (make-coords))
    )
  )

(defun mirror-drill
  ()
  (let* (
         (drill-coords (send *drill* :copy-worldcoords))
         (drill-pos (send drill-coords :pos))
         (drill-rpy (car (send drill-coords :rpy-angle)))
         )
    (send *drill* :newcoords (send (make-coords :pos (float-vector (elt drill-pos 0) (- (elt drill-pos 1)) (elt drill-pos 2)) :rpy (list (-(elt drill-rpy 0)) (+ (elt drill-rpy 1)) (- (elt drill-rpy 2)))) :rotate (deg2rad 134) :z))
    )
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
  (coords &key (real? t) (try-many? t))
  (let* ((drill-motion-av-list (get-drill-button-motion-full coords :draw? nil))
         (drill-motion-length (length drill-motion-av-list)))
    (when (not (= drill-motion-length 0))
      (when real?
        (set-default-impedance-param)
        (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
               (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 400)) ;; start imp hard
              (t
               (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000))))

      (when real? (send *ri* :set-ref-force #f(0 0 -20) 2000 *drill-main-arm*))
      ;; first try
      ;; pre-push1
      (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
             (cond ((equal *drill-button-point* :attachment)
                    (drill-full-grasp :arm :arms))
                   ((equal *drill-button-point* :finger)
                    (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
                           (send *robot* :hand (get-opposite-arm *drill-main-arm*) :angle-vector (float-vector 90.0 90.0 50.0 0.0 90.0 -40.0))
                           (send *robot* :hand *drill-main-arm* :angle-vector (float-vector 90.0 0.0 0.0 0.0 90.0 90.0))
                           (hand-model2real)
                           ))

                    )
                   ))
            (t ;; (or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
             (send *ri* :start-grasp *drill-main-arm* :gain 40)
             (send *ri* :stop-grasp (get-opposite-arm *drill-main-arm*))
             ))
      (send *robot* :angle-vector (elt drill-motion-av-list 0))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 5000))
      ;; pre-push2
      (send *robot* :angle-vector (elt drill-motion-av-list 1))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 1500))
      ;; pre-push3
      (send *robot* :angle-vector (elt drill-motion-av-list 2))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 1000))
      ;; push
      (send *robot* :angle-vector (elt drill-motion-av-list 3))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 1000))
      ;; pre-push3
      (send *robot* :angle-vector (elt drill-motion-av-list 4))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 1000))
      ;; pre-push2
      (send *robot* :angle-vector (elt drill-motion-av-list 5))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 1000))
      (setq req (instance drc_task_common::GetBoolStateRequest :init))
      (unix::sleep 4)
      (let ((request-service-name (if (equal *drill-main-arm* :rarm) "/rarm_get_button_state" "/larm_get_button_state")))
        (if (send (ros::service-call request-service-name req) :success)
            (progn (speak-en "button succeeded")
                   (return-from send-drill-button-motion :success) 
                   (speak-en "button failes"))
            )
        )
      ;; second try
      (when try-many?
        (dotimes (ix (if (or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT")) 3 5))
          (dotimes (iy 5)
            (let ((button-ik nil) (width 6) (width-x 15))
              (send *robot* :angle-vector (elt drill-motion-av-list 5))
              (send *robot* :fix-leg-to-coords (make-coords))
              (setq button-ik (fullbody-inverse-kinematics-for-drill-button :x-offset (+ 0 (* (- ix (/ (- ix 1.0) 2)) width-x)) :y-offset (+ 0 (* (- iy (/ (- iy 1.0) 2)) width)) :z-offset -35 :draw? nil))
              (send *ri* :wait-interpolation)
              (if button-ik (send *ri* :angle-vector button-ik 500)
                (send *ri* :angle-vector (elt drill-motion-av-list 5))
                )
              (setq button-ik (fullbody-inverse-kinematics-for-drill-button :x-offset (+ 0 (* (- ix (/ (- ix 1.0) 2)) width-x)) :y-offset (+ 0 (* (- iy (/ (- iy 1.0) 2)) width)) :z-offset 40 :draw? nil))
              (send *ri* :wait-interpolation)
              (if button-ik (send *ri* :angle-vector button-ik 750))
              )
            )
          )
        (send *ri* :wait-interpolation)
        )
      (send *robot* :angle-vector (elt drill-motion-av-list 5))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (when real? (model2real :time 2000))
      (unix::sleep 4)
      (let ((request-service-name (if (equal *drill-main-arm* :rarm) "/rarm_get_button_state" "/larm_get_button_state")))
        (if (send (ros::service-call request-service-name req) :success)
            (progn (speak-en "button succeeded")
                   (return-from send-drill-button-motion :success) 
                   (speak-en "button failes"))
          )
        )
      )
    ))
(defun publish-drill-button-motion
  (&key coords (step-time 400) (update? t) (draw? t))
  (when update?
    (get-drill-button-motion-full coords :use-ri? nil :draw? draw?))
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (publish-motion-joint-states *drill-motion* :step-time step-time)
  )

;; (defun send-drill-look-motion
;;   (&key (real? nil))
;;   ;; look
;;   (send *robot* :angle-vector (get-pre-drill-button-pose))
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

(defun get-center-coords
  (coords-list)
  (let (center (center-pos (float-vector 0 0 0)))
        (dotimes (i (length coords-list))
                (setq center-pos (v+ center-pos (send (elt coords-list i) :worldpos)))
                )
        (setq center (make-coords :pos (scale (/ 1.0 (length coords-list)) center-pos) :rot (send (elt coords-list 0) :rot))) ;; rot of first, because wall recognizing
        center
        )
  )
(defun get-relative-coords-list
  (center coords-list)
  (let ((relative-coords-list nil))
    (dotimes (i (length coords-list))
      (push-back (send center :transformation (send (elt coords-list i) :copy-worldcoords)) relative-coords-list)
      )
    relative-coords-list
    )
  )
(defun get-coords-list-with-center
  (center relative-coords-list)
  (let ((coords-list nil))
    (dotimes (i (length relative-coords-list))
      (push-back (send (send center :copy-worldcoords) :transform (send (elt relative-coords-list i) :copy-worldcoords)) coords-list)
      )
    coords-list
    )
  )
(defun generate-drill-wall-motion
  (&key (draw? t) (drill-type *default-drill-type*) (c-check t))
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
           :parent (send (send *robot* *drill-main-arm* :end-coords) :parent)))

    ;; wall drill
    (send *robot* :angle-vector (get-pierce-drill-pre-pose))
    (send *robot* :fix-leg-to-coords original-foot-midcoords)    ;; solve ik to wall drill
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
        (when (and c-check (c-check-for-drill)) (setq ik-ret nil))
        (push-back (list (+ i 1) (not (null ik-ret)) ik-ret) ik-results)
        (when (and fin? (not ik-ret)) (return-from drill-ik nil))
        (when (or (equal i 0) ;; (equal i (- dt-coords-length 1))
                  )
          (send *drill* :translate (float-vector 0 0 -130) :local)
          (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
          (when (and c-check (c-check-for-drill)) (setq ik-ret nil))
           (when ik-ret
             (setq ik-ret
                   (send *robot* :look-at-target (send *drill* :worldcoords)))))
          (if (equal i 0) (push (list 0 (not (null ik-ret)) ik-ret) ik-results))
          (if (or ;; (equal i (- dt-coords-length 1)) 
               (equal dt-coords-length 0)) (push-back (list (+ dt-coords-length 1) (not (null ik-ret)) ik-ret) ik-results))
          )
      (send *drill* :newcoords (get-center-coords dt-coords-list))
      (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
      (when (and c-check (c-check-for-drill)) (setq ik-ret nil))
      (when ik-ret
        (setq ik-ret
              (send *robot* :look-at-target (send *drill* :worldcoords))))
      (push-back (list (+ dt-coords-length 1) (not (null ik-ret)) ik-ret) ik-results)
      (send *drill* :translate (float-vector 0 0 40) :local)
      (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
      (when (and c-check (c-check-for-drill)) (setq ik-ret nil))
      (when ik-ret
        (setq ik-ret
              (send *robot* :look-at-target (send *drill* :worldcoords))))
      (send *drill* :translate (float-vector 0 0 -130) :local)
      (setq ik-ret (if *use-dual-arm-ik* (fullbody-inverse-kinematics-for-drill-grasp-dual :draw? draw?) (fullbody-inverse-kinematics-for-drill-grasp-single :draw? draw? :rotation-axis (if (equal drill-type :takenoko) '(:z t t) '(:z t t)) :arm-mt drill-edge-mt :arm-tc (send *drill* :get :edge-coords))))
      (when (and c-check (c-check-for-drill)) (setq ik-ret nil))
      (when ik-ret
        (setq ik-ret
              (send *robot* :look-at-target (send *drill* :worldcoords))))
      (push-back (list (+ dt-coords-length 3) (not (null ik-ret)) ik-ret) ik-results)      
      )
      ;; draw
      (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects :flush t)))
      ;; drill-reset-pose
                                        ;(send *robot* :angle-vector (get-pre-drill-button-pose))
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
(defun get-drill-wall-stand-coords-full
  (drill-coords-list)
  (let (drill-oriented-coords
        (mid-dist-pos (float-vector 0 0 0)))
    (if (equal (length drill-coords-list) 1)
        (setq mid-dist-pos (send (elt drill-coords-list 0) :pos))
      (progn
        (dotimes (i (- (length drill-coords-list) 1))
          (setq mid-dist-pos (v+ mid-dist-pos (send (elt drill-coords-list i) :pos))))
        (setq mid-dist-pos (scale (/ 1.0 (- (length drill-coords-list) 1)) mid-dist-pos))
        )
      )
    ;; (let ((theta (atan2 (elt mid-dist-pos 1)
    ;;                     (elt mid-dist-pos 0))))
    ;;   (let ((ux (float-vector (cos theta)
    ;;                           (sin theta)
    ;;                           0))
    ;;         (uz (float-vector 0 0 1)))
    ;;     (let ((uy (v* uz ux)))
    ;;       (let ((mat (make-matrix 3 3)))
    ;;         (setf (matrix-column mat 0) ux)
    ;;         (setf (matrix-column mat 1) uy)
    ;;         (setf (matrix-column mat 2) uz)
    ;;         (setq drill-oriented-coords
    ;;               (make-coords :pos (send (elt drill-coords-list 0) :worldpos)
    ;;                            :rot mat))))))
    
    (let ((ux (matrix-column (send (elt drill-coords-list 0) :rot) 2))
          (uz (float-vector 0 0 1)))
          ;(setq ux (scale -1.0 ux))
      (let ((uy (v* uz ux)))
        (let ((mat (make-matrix 3 3)))
          (setf (matrix-column mat 0) ux)
          (setf (matrix-column mat 1) uy)
          (setf (matrix-column mat 2) uz)
          (setq drill-oriented-coords
                    (make-coords :pos mid-dist-pos
                                 :rot mat)))))
    (setq drill-wall-coords (send (send drill-oriented-coords :copy-worldcoords)
                                  :transform (get-drill-wall-stand-coords)))
    (make-coords :pos (float-vector (elt (send drill-wall-coords :worldpos) 0) (elt (send drill-wall-coords :worldpos) 1) 0) :rpy (float-vector (elt (car (send drill-wall-coords :rpy-angle)) 0) 0 0))
    )
  )
(defun get-drill-wall-stand-coords ;; relative to drill
  (&key (drill-type *default-drill-type*))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (if (equal drill-type :gun)
             (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -680 0 0) :rpy (list (deg2rad 0) 0 0)) (make-coords :pos (float-vector -680 0 0) :rpy (list (deg2rad 0) 0 0)))
           (if *drill-reverse-hand*
               (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -680 -200 0) :rpy (list (deg2rad 40) 0 0))
                 (make-coords :pos (float-vector -680 -0 0) :rpy (list (deg2rad -25) 0 0)) ;; avoid wall
                 )
             (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -700 -265 0) :rpy (list (deg2rad 50) 0 0)) (make-coords :pos (float-vector -700 250 0) :rpy (list (deg2rad -65) 0 0)))
             )
           ))
        (t ;;JAXON
         (if *drill-reverse-hand*
             (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -850 200 0) :rpy (list (deg2rad 50) 0 0)) (make-coords :pos (float-vector -850 -250 0) :rpy (list (deg2rad -40) 0 0)))
           (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -780 -290 0) :rpy (list (deg2rad 50) 0 0)) (make-coords :pos (float-vector -780 -490 0) :rpy (list (deg2rad 0) 0 0)))
           ;; (if (equal *drill-main-arm* :rarm) (make-coords :pos (float-vector -780 -290 0) :rpy (list (deg2rad 50) 0 0)) (make-coords :pos (float-vector -650 -90 0) :rpy (list (deg2rad -20) 0 0)))
           )
         )
        )
  )
(defun get-drill-wall-motion-full
  (drill-coords-list &key (draw? nil) (use-ri? t) (drill-type *default-drill-type*) (recognized-coords-symbol :edge-coords)
  (overwrite-stand-coords? t) (stand-coords nil) (devide-num 3))
  (setq *drill-stand-coords* (or stand-coords (make-coords)))
  (when (equal (length drill-coords-list) 0) (ros::ros-info "wall coords are empty") (return-from nil nil))
  (set-drill-environment-full (send (elt drill-coords-list 0) :copy-worldcoords) :draw? draw? :use-ri? use-ri? :drill-type drill-type :recognized-coords-symbol recognized-coords-symbol)
  ;(send *drill* :put :destination-coords-list drill-coords-list)
  (let ((dest-coords-list nil) (dest-coords-list-filled nil))
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
    (dotimes (i (length dest-coords-list))
      (when (not (equal i 0))
          (dotimes (j (- devide-num 1))
            (push-back (midcoords (/ (float (+ j 1)) devide-num) (elt dest-coords-list (- i 1)) (elt dest-coords-list i)) dest-coords-list-filled)
            )
        )
      (push-back (elt dest-coords-list i) dest-coords-list-filled)
      )
    (send *drill* :put :destination-coords-list dest-coords-list-filled)
    )
  (send *robot* :fix-leg-to-coords *drill-stand-coords*)
  (setq *drill-stand-coords-forced* (get-drill-wall-stand-coords-full drill-coords-list))
  (let* ((drill-motion-av-list (if (and (equal overwrite-stand-coords? :force) (check-stand-coords *drill-stand-coords-forced* :pos-thre 0.1 :rpy-thre 4)) nil (get-drill-wall-motion :draw? draw?)))
         (drill-motion-length (length drill-motion-av-list)))
    (when (and (= drill-motion-length 0) overwrite-stand-coords?)

      (setq *drill-stand-coords* *drill-stand-coords-forced*)
      (ros::ros-warn "change stand position to ~a for drill motion.~%" *drill-stand-coords*)
      (send *robot* :fix-leg-to-coords *drill-stand-coords*)
      (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
      (setq drill-motion-av-list (get-drill-wall-motion :draw? draw?))
      )
    drill-motion-av-list
    ))



(defun send-drill-wall-motion
  (coords-list &key (real? t) (drill-type *default-drill-type*) (stand-coords nil) (overwrite-stand-coords? nil) (devide-num 3) (draw? nil))
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (let* ((drill-motion-av-list (get-drill-wall-motion-full coords-list :drill-type drill-type :recognized-coords-symbol :edge-coords :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords? :devide-num devide-num :draw? draw?))
         (drill-motion-length (length drill-motion-av-list)))
    (when (= drill-motion-length 0) (return-from send-drill-wall-motion))
    (when (equal drill-type :takenoko)
      ;; (send *robot* :angle-vector (get-reach-drill-pre-pose))
      ;; (when real? (model2real :time 5000))
      (send *robot* :angle-vector (get-pierce-drill-pre-pose))
      (when real? (model2real :time 5000))
      )

    ;(send *ri* :set-ref-force #f(0 0 -20) 2000 *drill-main-arm*)
    ;; walk
    (when (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
      (let* ((pos (scale 0.001 (send *drill-stand-coords* :worldpos)))
             (rpy (mapcar #'rad2deg (car (send *drill-stand-coords* :rpy-angle)))))
        (ros::ros-warn "go-pos to ~a ~a ~a for drill~%" (elt pos 0) (elt pos 1) (elt rpy 0))
        (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
        ))
    (set-default-impedance-param)
    ;; (when real?
    ;;
    (cond ((or (equal *robot-name* "HRP2JSKNTS") (equal *robot-name* "HRP2JSKNT"))
           ;(send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 -200) :k-p 1000 :d-p 400)) ;; start imp hard
          (send *ri* :start-impedance *drill-main-arm* :force-gain #f(0.1 0.1 1) :k-p 150 :d-p 700))
          (t
           (send *ri* :start-impedance *drill-main-arm* :force-gain #f(0.1 0.1 1) :moment-gain (float-vector 0 0 0) :k-p 700 :d-p 1500)))
    ;(send *ri* :start-impedance *drill-main-arm* :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 1500)
    ;; pre-reach

    (let* ((drill-dir (matrix-column (send (send (send (if stand-coords stand-coords (make-coords)) :copy-worldcoords) :transformation (send (elt coords-list 0) :copy-worldcoords)) :rot) 2))
          (ref-dir (v- (float-vector 0 0 -20) (scale (if (equal *robot-name* "JAXON") 5.0 3.0) (normalize-vector drill-dir)))))
      (send *ri* :set-ref-force ref-dir 2000 *drill-main-arm*)
      (ros::ros-info (format nil "ref force ~A ~A ~A" (elt ref-dir 0) (elt ref-dir 1) (elt ref-dir 2)))
      (speak-en (format nil "ref force ~A ~A ~A" (elt ref-dir 0) (elt ref-dir 1) (elt ref-dir 2)))
      )
    ;;todo
    (let ((i 0))
      (while (< i (length drill-motion-av-list))
        (cond ((or 
                (equal i 0) 
                (equal i 1)
                (equal i (- (length drill-motion-av-list) 3))
                (equal i (- (length drill-motion-av-list) 2))
                (equal i (- (length drill-motion-av-list) 1))
                )
               (send *robot* :angle-vector (elt drill-motion-av-list i))
               (when real? (model2real :time 5000))
               (setq i (+ i 1)))
              (t
               (send-angle-vector-sequence-with-time (subseq drill-motion-av-list i (+ i 3)) :time (/ 15000 devide-num))
               (setq i (+ i 3))
               )
              )
        )
      )
    ;; (dotimes (i (length drill-motion-av-list))
    ;;   (send *robot* :angle-vector (elt drill-motion-av-list i))
    ;;   (when real? (model2real :time (if (or (equal i 0) (equal i (- (length drill-motion-av-list) 1))) 5000 15000)))
    ;;   )
    (send *ri* :set-ref-force (float-vector 0 0 -20) 2000 *drill-main-arm*)

    (send *ri* :set-impedance-controller-param *drill-main-arm* :force-gain (float-vector 1 1 1))
    (when (equal drill-type :takenoko)
      (send *robot* :angle-vector (get-pierce-drill-pre-pose))
      (when real? (model2real :time 5000))
      ;; (send *robot* :angle-vector (get-reach-drill-pre-pose))
      ;; (when real? (model2real :time 5000))
      )
    )
  )

(defun publish-drill-wall-motion
  (&key coords-list (step-time 100) (update? t) (drill-type *default-drill-type*)
        (stand-coords nil) (overwrite-stand-coords? nil)
        (draw? nil)
        )
  (when (and stand-coords (not (check-stand-coords stand-coords)))
    (setq stand-coords nil)
    )
  (when update?
    (get-drill-wall-motion-full coords-list :use-ri? nil :drill-type drill-type :recognized-coords-symbol :edge-coords :draw? draw? :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?))
  (ros::ros-info "drill-wall-stand-coords: ~a~%" *drill-stand-coords*)
  (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
  (if (and *drill-stand-coords* (check-stand-coords *drill-stand-coords*))
    (move-robot-model *drill-stand-coords* "ground")
    (move-robot-model (make-coords) "ground"))

  (publish-motion-joint-states *drill-motion* :step-time step-time))


;; todo grasp-coords rotate
;; set-drill-environment
(defun generate-drill-rotate-motion
  (&key (draw? t) (devide-num 20) (grasp-up-dist 100))
  (let* (ik-results
         ik-results-before
         ik-results-after
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (original-angle-vector (send *robot* :angle-vector))
         (original-drill-coords (send *drill* :copy-worldcoords))
         )
    (when *drill-reverse-hand* (setq grasp-up-dist (- grasp-up-dist)))
    (let* (ik-ret-out ik-ret-in ik-ret-up)
      (block reverse
        (dotimes (index (/ devide-num 4))
          (let ((i (* -1 index)))
            (send *robot* :angle-vector original-angle-vector)
            (send *robot* :fix-leg-to-coords original-foot-midcoords)
            (send *drill* :newcoords (send original-drill-coords :copy-worldcoords))
            (send *drill* :rotate (* i (deg2rad (/ 360.0 devide-num))) :z)
            (setq ik-ret-in (fullbody-inverse-kinematics-for-drill-grasp-single :rotation-axis (list t t t) :stop 30))
            (if ik-ret-in
                (setq ik-ret-in (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from reverse nil)
              )
            (setq ik-ret-out (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *drill* :get :grasp-coords) :copy-worldcoords) :translate (float-vector -100 0 0) :local) :rotation-axis (list t t t) :stop 30 :draw? draw?))
            (if ik-ret-out
              (setq ik-ret-out (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from reverse nil))
            (setq ik-ret-up (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *drill* :get :grasp-coords) :copy-worldcoords) :translate (float-vector 0 0 grasp-up-dist)) :rotation-axis (list t t t) :stop 30 :draw? draw?))
            (if ik-ret-up
                (setq ik-ret-up (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from reverse nil))
            (when (not (and ik-ret-out ik-ret-in ik-ret-up))
                       (return-from reverse nil))
            (push-back (list i (not (null (and ik-ret-out ik-ret-in ik-ret-up))) ik-ret-out ik-ret-in ik-ret-up) ik-results-before)
            ;(when draw? (send *irtviewer* :draw-objects :flush t))
            )
          )
        )
      (block front
        (dotimes (index (/ devide-num 4))
          (let ((i (+ (* +1 index) 1)))
            (send *robot* :angle-vector original-angle-vector)
            (send *robot* :fix-leg-to-coords original-foot-midcoords)
            (send *drill* :newcoords (send original-drill-coords :copy-worldcoords))
            (send *drill* :rotate (deg2rad (* i (/ 360.0 devide-num))) :z)
            (setq ik-ret-in (fullbody-inverse-kinematics-for-drill-grasp-single :rotation-axis (list t t t) :stop 30 :draw? draw?))
            (if ik-ret-in
                (setq ik-ret-in (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from front nil))
            (setq ik-ret-out (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *drill* :get :grasp-coords) :copy-worldcoords) :translate (float-vector -100 0 0)) :rotation-axis (list t t t) :stop 30 :draw? draw?))
            (if ik-ret-out
                (setq ik-ret-out (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from front nil))
            (send *robot* :angle-vector original-angle-vector)
            (send *robot* :fix-leg-to-coords original-foot-midcoords)
            (setq ik-ret-up (fullbody-inverse-kinematics-for-drill-grasp-single :arm-tc (send (send (send *drill* :get :grasp-coords) :copy-worldcoords) :translate (float-vector 0 0 grasp-up-dist)) :rotation-axis (list t t t) :stop 30))
            (if ik-ret-up
                (setq ik-ret-up (send *robot* :look-at-target (send *drill* :worldcoords)))
              (return-from front nil))
            (when (not (and ik-ret-out ik-ret-in ik-ret-up))
              (return-from front nil))
            (push-back (list i (not (null (and ik-ret-out ik-ret-in ik-ret-up))) ik-ret-out ik-ret-in ik-ret-up) ik-results-after)
            (when draw? (when (boundp '*irtviewer) (send *irtviewer* :draw-objects :flush t)))
            )
          )
        )
      )
    (send *robot* :angle-vector original-angle-vector)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    (send *drill* :newcoords original-drill-coords)
    (setq *drill-ik-results* (append (reverse ik-results-before) ik-results-after))
    )
  )

(defun get-drill-rotate-motion
  (&key (draw? t) (devide-num 20))
  (generate-drill-rotate-motion :draw? draw? :devide-num devide-num)
  (let* (angle-vector-ret-out angle-vector-ret-in angle-vector-ret-up)
    (dotimes (i (length *drill-ik-results*))
      (push-back (elt (elt *drill-ik-results* i) 2) angle-vector-ret-out)
      (push-back (elt (elt *drill-ik-results* i) 3) angle-vector-ret-in)
      (push-back (elt (elt *drill-ik-results* i) 4) angle-vector-ret-up)
      )
    (setq *drill-motion-in* angle-vector-ret-in)
    (setq *drill-motion-out* angle-vector-ret-out)
    (setq *drill-motion-up* angle-vector-ret-up)
    (list angle-vector-ret-in angle-vector-ret-out angle-vector-ret-up)
    )
  )

(defun get-drill-rotate-motion-full
  (drill-coords plane-coords &key (draw? t) (use-ri? t) (devide-num 20) (grasp-up-dist 100) (thre 20) (angle-vector nil)) ;; tmp draw
  (let ((diff-deg (get-needed-rotate-angle drill-coords :use-ri? use-ri?)) ; it initialize too!
        resolution
        ik-length
        rotate-length
        )
    (send (send *drill* :get :grasp-coords) :rotate (deg2rad diff-deg) :z)
    (send *drill* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform (get-drill-coords-with-symbol plane-coords :put-coords) :local))
    (setq resolution (/ 360.0 devide-num))
    (setq diff-length (+ (round (abs (+ (/ diff-deg resolution) 0.5))) 1));ik length needed to rotate
    (when (< diff-length 2) (return-from get-drill-rotate-motion-full (list :no-need-to-rotate 0)))
    (when angle-vector 
      (ros::ros-info "default angle set")
      (send *robot* :angle-vector angle-vector)
      (send *robot* :fix-leg-to-coords (make-coords))
      )
    (when (not (fullbody-inverse-kinematics-for-drill-grasp-single))
      (ros::ros-info "first ik failes")
      (return-from get-drill-rotate-motion-full (list :need-rotate-but-ik-fails 0))
      )
    (set-drill-default-coords)
    (get-drill-rotate-motion :draw? draw?)
    (list :rotate diff-deg)
    (setq ik-length (length *drill-motion-up*))
    (ros::ros-info (format nil "ik-length: ~A" ik-length))
    (setq rotate-length (min ik-length diff-length))
    (when (< ik-length 2) (return-from get-drill-rotate-motion-full (list :need-rotate-but-ik-fails 0)))
    (ros::ros-info (format nil "rotate-length: ~A" rotate-length))
    (when (< diff-deg 0)
      (setq *drill-motion-up* (reverse *drill-motion-up*))
      (setq *drill-motion-in* (reverse *drill-motion-in*))
      (setq *drill-motion-out* (reverse *drill-motion-out*))
      )
    (setq *drill-motion-up* (subseq *drill-motion-up* 0 rotate-length))
    (setq *drill-motion-in* (subseq *drill-motion-in* 0 rotate-length))
    (setq *drill-motion-out* (subseq *drill-motion-out* 0 rotate-length))
    (list :rotate ik-length diff-length diff-deg)
    )
  )


(defun get-body-relative-coords
  (coords &key)
  (send (send (send *robot* :body_lk) :copy-worldcoords) :transformation (send coords :copy-worldcoords))
  )

(defun send-drill-rotate-motion
  (drill-coords plane-coords &key (real? t) (drill-rotate-num 1) (draw? nil) (mid-pose? t))
  (setq *drill-motion-in* nil)
  (setq *drill-motion-up* nil)
  (setq *drill-motion-out* nil)
  (let* 
      ((rotate-result (get-drill-rotate-motion-full drill-coords plane-coords :draw? draw?)))
    (when (or (atom rotate-result) (not (equal (elt rotate-result 0) :rotate)))
      (ros::ros-info "diff deg 0 or ik failed")
      (return-from send-drill-rotate-motion (elt rotate-result 0))
      )
    (let
        ((ik-length (elt rotate-result 1))
         (diff-length (elt rotate-result 2))
         (diff-deg (elt rotate-result 3)))
      (while (and (< 0 drill-rotate-num) (< 1 diff-length))
        (let ((rotate-length (min diff-length ik-length)))
          ;; check-coords-distance
          (when mid-pose?
            (send *ri* :angle-vector (get-reach-drill-pre-pose))
            )
          (send *robot* :angle-vector (elt *drill-motion-up* 0))
          (when real? (model2real :time 3000))
          (send *ri* :set-ref-force #f(0 0 0) 2000 *drill-main-arm*)
          (send *robot* :angle-vector (elt *drill-motion-in* 0))
          (when real? (model2real :time 2000))
          (when real? (drill-hook))
          (send-angle-vector-sequence-with-time (subseq *drill-motion-out* 0 rotate-length) :time 2500)
          (send *robot* :angle-vector (elt *drill-motion-in* (- rotate-length 1)))
          (when real? (model2real :time 5000))
          (when real? (drill-air-grasp))
          (send *robot* :angle-vector (elt *drill-motion-up* (- rotate-length 1)))
          (when real? (model2real :time 2000))
          (when real? (drill-full-grasp))
                                        ;(send *ri* :set-ref-force #f(0 0 -20) 2000 *drill-main-arm*)
          (if (not (set-ref-force-with-condition)) (return-from send-drill-rotate-motion nil))
          (send (send *drill* :get :grasp-coords) :rotate (deg2rad (- (* (if (> diff-deg 0) 18.0 -18.0) (- rotate-length 1)))) :z)
                                        ;(rotate-drill-horizontal)
          (set-drill-default-coords)
          (setq drill-rotate-num (- drill-rotate-num 1))
          (setq diff-length (- diff-length (- rotate-length 1)))
          )
        )
      ))
  )
(defun get-needed-rotate-angle
  (drill-coords &key (use-ri? nil) (draw? nil))
  (set-drill-environment-full drill-coords :draw? draw? :use-ri? use-ri?)
  (check-diff-angle)
  )
(defun publish-drill-rotate-motion
  (&key drill-coords plane-coords (step-time 100) (update? t) (publish? t) (use-ri? nil) angle-vector (move-robot-model? t) (draw? nil))
  (let ((ret (get-drill-rotate-motion-full drill-coords plane-coords :use-ri? use-ri? :angle-vector angle-vector :draw? draw?)))
    (when (not (equal (elt ret 0) :rotate))
      (when (and (consp ret) (boundp '*app*))
        (send *app* :show-text (format nil "result, ~A" (elt ret 0)))
        )
      (return-from publish-drill-rotate-motion ret)
      )
    (ros::ros-info "drill-ik-results: ~a~%" *drill-ik-results*)
    (when (and publish? move-robot-model?) (move-robot-model (make-coords) "ground")
          (publish-motion-joint-states (append *drill-motion-in* *drill-motion-up*) :step-time step-time))
    
    (when (and (consp ret) (boundp '*app*))
      (send *app* :show-text (format nil "result, ~A" (elt ret 0)))
      (when (equal (elt ret 0) :rotate)
        (send *app* :show-text (format nil "needed angle:~A, can rotate:~A" (- (elt ret 2) 1) (- (elt ret 1) 1))
              )
        )
      )
    ret
    )
  )


(defun check-diff-angle
  ()
  (* (rad2deg (elt (car (send (send  (send (gethash *drill-main-arm* *arm-drill-grasp-coords*) :copy-worldcoords) :transformation (send (send *drill* :get :grasp-coords-true) :copy-worldcoords)) :rpy-angle)) 0)) (if *drill-reverse-hand* -1 1))
  )

(defun check-reachability
  (&key (width 50) (arm *drill-main-arm*))
  (let ((result-list nil) (current-av (send *robot* :angle-vector)))
    (dotimes (ix 3)
      (dotimes (iy 3)
        (dotimes (iz 3)
          (if (send *robot* arm :move-end-pos (float-vector (* (- ix 1) width) (* (- iy 1) width) (* (- iz 1) width)) :local :debug-view :no-message)
              (push-back t result-list) (push-back nil result-list)
            )
          (send *robot* :angle-vector current-av)
          )))
    result-list
    )
  )

;; drill util ;;;;

(defun rotate-drill-horizontal (&key (use-current? t) (real? t))
  (let* (
        (original-foot-coords (send *robot* :foot-midcoords))
        (original-angle-vector (send *robot* :angle-vector))
        )
    (if (and use-current? real?) (send *robot* :angle-vector (send *ri* :state :reference-vector)))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq ret
          (send *robot* :rarm :inverse-kinematics (make-coords :pos (send (send (send *robot* *drill-main-arm* :end-coords) :copy-worldcoords) :pos) :rpy (list 0 0 (* pi/2 (if *drill-reverse-hand* 1 -1) (if (equal *drill-main-arm* :rarm) -1 1))))))
    (when (and ret real?)
      (model2real :time 3000)
      )
    (send *robot* :angle-vector original-angle-vector)
    (send *robot* :fix-leg-to-coords original-foot-coords)
    ret
    )
  )
(defun send-drill-throw-motion ()
  (set-drill-environment)
  (send *robot* :reset-pose)
  (model2real :time 10000)
  (send *robot* :angle-vector
        #f(3.44994 -19.6749 -76.4333 112.505 -44.7788 8.57772 0.0 3.58847 -18.7016 -72.8647 110.545 -46.3422 7.71206 0.0 12.3847 45.076 -37.7442 35.7111 -27.222 -8.17595 -10.9602 -22.3436 -0.390404 -20.1614 -2.54513 15.0 11.0358 14.2058 -7.98962 -34.5693 7.37697 2.53959 -5.48105 -15.0)
        )
  (if (equal *drill-main-arm* :larm)
      (send *robot* :angle-vector (get-mirror-angle-vector))
    )
  (model2real :time 20000)
  (drill-hook)
  (send *ri* :set-ref-force (float-vector 0 0 0) 2000 *drill-main-arm*)
  (send *robot* :reset-pose)
  (model2real :time 10000)
  )
(defun get-reach-drill-pre-pose ()
  (let ((angle-ret
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (if *drill-reverse-hand*
             #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -54.9114 -41.9176 63.6546 -114.997 82.9781 15.3017 22.7922 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
             #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -43.1188 -36.5194 14.7778 -106.994 -32.1181 -12.396 18.5842 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
             )
         )
        (t (send *robot* :reset-manip-pose))
        )))
    (send *robot* :angle-vector angle-ret)
    (when (equal *drill-main-arm* :larm) (setq angle-ret (get-mirror-angle-vector)))
    angle-ret
    )
  )
(defun get-pierce-drill-pre-pose ()
  ;; (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 1.19212 -20.9065 -5.36626 -98.3812 50.0 6.28459 -83.1406 60.0 -17.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
  (let ((angle-ret
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (if *drill-reverse-hand*
             #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -15.8902 -24.0028 13.0297 -91.879 5.61774 -4.8117 -73.0948 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
             #f(-2.38692 3.44954 -31.3438 59.374 -27.6708 -1.8005 0.0 -2.38708 3.47463 -32.0978 59.782 -27.3249 -1.82558 0.0 10.0735 8.75398 -17.663 20.7846 -37.6601 10.0 29.6919 -68.3126 -97.3465 -3.31063 -19.3263 60.0 29.0864 29.9297 2.98779 -118.271 21.1947 0.852295 -11.3978 -60.0)
             ))
        (t
         (if *drill-reverse-hand*
             (send *robot* :reset-pose)
             #f(1.8474 -2.65775 -26.6226 45.4555 -22.888 1.96534 1.84848 -2.64244 -25.6546 44.674 -23.0745 1.95 4.97106 9.06265 14.0558 -32.0 30.0 5.44788 41.1171 -56.4215 -5.75946 -108.854 -115.061 15.0282 27.2784 1.11288 44.8267 41.7962 -9.91826 -101.245 33.2679 14.7361 -39.3661)
         ))
        )))
    (send *robot* :angle-vector angle-ret)
    (when (equal *drill-main-arm* :larm) (setq angle-ret (get-mirror-angle-vector)))
    angle-ret
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
(defun get-pre-drill-button-pose ()
  (let ((angle-ret
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         ;; #f(-1.73786 12.7998 -28.717 58.3273 -27.2775 -3.19394 0.0 -1.75078 13.1089 -32.0242 64.2536 -29.8937 -3.50327 0.0 -18.0177 40.9471 -15.5394 -20.0 -71.9257 -1.63271 9.57461 -68.7006 -18.7947 12.8132 13.4518 60.0 -67.3083 63.2485 -56.2882 -89.6456 -32.2721 16.8311 -17.2177 -60.0))
         ;; #f(-3.18801 14.0246 -30.3389 64.3646 -30.1868 -5.8666 0.0 -3.20998 14.342 -32.5966 67.7287 -31.2878 -6.18463 0.0 -6.39784 15.1167 -23.2174 -16.7713 -58.3745 -17.3128 21.0908 -111.942 -21.179 -21.2009 34.1876 60.0 -64.9891 55.4053 -56.0059 -114.503 -79.2957 19.8623 67.9176 -60.0)) ;; 04/05
         #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -23.2174 -16.7713 -63.8998 -26.7475 19.7525 -93.797 -60.1496 14.1419 -8.73053 60.0 -64.9891 55.4053 -56.0059 -114.503 -79.2957 19.8623 67.9176 -60.0))
        ((equal *robot-name* "JAXON")
         (if *drill-reverse-hand*
             #f(-0.177859 0.641193 -17.7005 39.4727 -20.9405 -0.24116 -0.177872 0.642071 -18.0592 40.1005 -21.2096 -0.242039 -1.32326 0.0 -0.214599 0.0 0.0 20.3307 32.0308 -36.6623 -1.24795 -118.487 -16.2352 -2.10621 -25.4482 -20.5155 51.3195 74.2961 41.72 -99.0289 15.0662 -31.9513 -59.7661)
             (send *robot* :reset-pose)
             )
         ;; send *robot* :larm :move-end-pos (float-vector -100 0 0) :local ;;from drill-button-first-pose
         )
        (t (send *robot* :reset-manip-pose))
        )))
    (send *robot* :angle-vector angle-ret)
    (when (equal *drill-main-arm* :larm) (setq angle-ret (get-mirror-angle-vector)))
    angle-ret
    )
  )
(defun get-drill-reset-pose (&key (arm *drill-main-arm*) (drill-type *default-drill-type*))
  (let ((angle-ret
    (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
      (cond
       ((not *drill-reverse-hand*)
        (send *robot* :reset-manip-pose))
       (t
        #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 20.0 49.2654 -15.6889 -8.04143 -131.198 78.4518 -8.73041 1.4443 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)
        ;; #f(-1.77229 1.82573 -25.6231 51.2333 -25.6803 -0.988256 0.0 -1.77228 1.83528 -26.1327 51.4649 -25.4022 -0.9978 0.0 10.8349 4.38163 -40.7868 34.4397 -34.2213 -48.3344 76.6297 -106.708 104.568 54.9403 38.4528 60.0 44.7247 30.079 9.16563 -119.921 23.9102 2.91034 -18.3553 -60.0)
        ))
      )
        (t
         (send *robot* :reset-manip-pose))
        )
  ))
    (send *robot* :angle-vector angle-ret)
    (when (equal arm :larm) (setq angle-ret (get-mirror-angle-vector)))
    angle-ret
    )
  )


(defun get-watch-drill-pose ()
  (let ((angle-ret
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (if *drill-reverse-hand*
             #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -10.6249 -19.0 -105.319 -10.0296 34.741 -43.3425 -106.0 43.5744 65.5498 60.0 40.0869 75.2885 11.7308 -117.757 -18.2568 65.0121 0.945275 -60.0)
             #f(-0.173437 9.3105 -27.542 65.2878 -34.038 -0.18 -0.324 -0.182812 9.432 -30.568 71.6022 -37.316 -0.3528 -0.036 -34.1485 9.39775 -23.2875 20.0 -29.6116 -18.9393 21.046 -113.361 -66.5677 -36.78 20.4044 60.0 -61.8932 48.1149 -54.1074 -126.033 -9.9503 -35.1643 0.847727 -60.0)
             )
         )
        ((equal *robot-name* "JAXON")
         (if *drill-reverse-hand*
             (send *robot* :reset-manip-pose)
           #f(2.03272 -1.10746 -18.4222 37.4121 -20.1829 0.277957 2.03297 -1.09553 -17.1498 35.9931 -20.0362 0.266026 -0.035991 0.125934 -0.881805 -29.3714 35.9221 17.203 10.8884 -75.6167 -16.7206 -90.815 -46.7448 77.8398 -46.7473 -14.7322 55.1311 42.536 20.5925 -113.521 111.075 -25.9983 27.7244)
           )
         )
        (t
         (send *robot* :reset-manip-pose))
        )
  ))
    (send *robot* :angle-vector angle-ret)
    (when (equal *drill-main-arm* :larm) (setq angle-ret (get-mirror-angle-vector)))
    angle-ret
    )
  )

(defun test-drill-button-motion-reachability
  ()
  (set-drill-button-environment-full nil)
  (setq *answer-list* nil)
  (dotimes (i 36)
    (send (send *drill* :get :grasp-coords) :rotate (deg2rad 10) :z)
    (send *ri* :angle-vector (send *robot* :reset-pose))
    (set-drill-default-coords)
    (setq *answer-for-grasp-test* (get-drill-button-motion-full (get-body-relative-coords (send *drill* :copy-worldcoords)) :draw? t :use-ri? t))
    (if *drill-motion*
        (push (* i 10) *answer-list*)
      )
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
(defun test-init
  ()
  (if (not (boundp '*robot-name*))
      (setq *robot-name* "HRP2JSKNTS")
    )
  (generate-robot *robot-name* :generate-ri? t)
  (send *robot* :fix-leg-to-coords (make-coords))
  )
(defun test-drill-grasp-motion (&key (high? nil) (rotation-axis (list :z t t)) (overwrite-stand-coords? :force) (standcoords (make-coords)))
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (test-init)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (get-body-relative-coords (make-coords :pos (float-vector 1200 0 (if high? 1100 830)) :rpy (list (deg2rad 90) 0 0))) :draw? t :drill-type :takenoko :rotation-axis rotation-axis :overwrite-stand-coords? overwrite-stand-coords?))
  (if (equal *robot-name* "HRPJSKNT")
      (send *robot* :hand :arms :hook-pose))
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-publish-drill-grasp-motion ()
  (setq *answer-for-grasp-test* (publish-drill-grasp-motion :coords (make-coords :pos (float-vector 1200 0 130) :rpy (list (deg2rad 90) 0 0)) :publish? nil :drill-rotate-num 1 :use-ri? t))
  )
(defun test-send-drill-grasp-motion (&key (rotate-drill? nil) (high? nil))
  (test-init)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send-drill-grasp-motion (get-body-relative-coords (make-coords :pos (float-vector 900 0 (if high? 1100 830)) :rpy (list (deg2rad 90) 0 0))) :drill-rotate-num 2 :draw? t)
  )
(defun test-send-drill-grasp-motion-with-rot ()
  (send-drill-grasp-motion (make-coords :pos (float-vector 700 0 200) :rpy (list 0 0 0)) :drill-rotate-num 3 :draw? t)
  )

(defun test-send-drill-rotate-motion ()
  (test-send-drill-grasp-motion)
  (send-drill-rotate-motion (get-body-relative-coords (send *drill* :copy-worldcoords)) (get-body-relative-coords (send *drill* :copy-worldcoords)))
  )

(defun test-send-drill-rotate-motion2 ()
  (set-drill-environment-full (make-coords :pos (float-vector 500 -150 100) :rpy (list (deg2rad 0) 0 0)))
  (objects (list *robot* *drill*))
  (send *drill* :rotate (deg2rad 180) :z)
  (send *robot* :angle-vector (get-drill-reset-pose))
  (fullbody-inverse-kinematics-for-drill-grasp-single :rotation-axis (list :z t t))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *drill* :rotate (deg2rad 190) :z)
  (send-drill-rotate-motion (get-body-relative-coords (send *drill* :copy-worldcoords)) (get-body-relative-coords (send *drill* :copy-worldcoords)))
  )
(defun test-send-drill-rotate-motion3
  (&key (standcoords (make-coords)))
  (test-drill-grasp-motion :rotation-axis (list :z t t) :standcoords standcoords)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  ;(send *drill* :rotate (deg2rad 130) :z)
  (send-drill-rotate-motion (get-body-relative-coords (send *drill* :copy-worldcoords)) (get-body-relative-coords (send (send *drill* :copy-worldcoords) :translate (float-vector 0 0 -100))) :drill-rotate-num 100)
  )

(defun test-drill-rotate-motion
  ()
  (set-drill-environment)
  (send *drill* :newcoords (make-coords :pos (float-vector 450.0 102.104 1090.459) :rpy (list -2.297 0.727 -0.472)))
  (send *robot* :angle-vector 
        (cond ((equal *robot-name* "HRP2JSKNTS")
               #f(-5.91298 5.03177 -37.4717 71.2432 -32.7995 -2.72015 0.0 -5.91474 5.13619 -39.0504 71.5181 -31.4957 -2.82452 0.0 -13.6276 10.7601 23.9568 24.2259 -111.192 10.0 91.991 -98.4731 23.5707 35.1984 80.7096 60.0 -52.165 26.6558 -37.6794 -125.311 63.7804 76.3645 37.7435 -60.0)
               )
              (t
               (send *robot* :reset-pose)
               )
              )
        )
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *drill-motion* nil)
  (let ((drill-support-grasp-mt
    (make-cascoords
     :coords
     (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 70 0 0) :local)
     :parent (send (send *robot* :larm :end-coords) :parent))))
    (push (fullbody-inverse-kinematics-for-drill-grasp-dual :support-arm-coords drill-support-grasp-mt :rotation-axis (list :z t t t)) *drill-motion*)
    (send *drill* :rotate (deg2rad 30) :z)
    (push (fullbody-inverse-kinematics-for-drill-grasp-dual :support-arm-coords drill-support-grasp-mt :rotation-axis (list :z t t t)) *drill-motion*)
    (send *drill* :rotate (deg2rad 30) :z)
    (push (fullbody-inverse-kinematics-for-drill-grasp-dual :support-arm-coords drill-support-grasp-mt :rotation-axis (list :z t t t)) *drill-motion*)
    (send *drill* :rotate (deg2rad 30) :z)
    )
  )

(defun test-drill-rotate-motion-reachability-single
  (dist deg)
  (set-drill-environment)
  (send *drill* :newcoords (make-coords :pos (float-vector dist 0 830)))
  (send *robot* :angle-vector (get-reach-drill-pre-pose))
  (send *robot* :fix-leg-to-coords (make-coords :rpy (list (deg2rad deg) 0 0)))
  (get-drill-grasp-motion)
  (get-drill-rotate-motion)
  (and *drill-motion* (length *drill-motion-in*))
  )
(defun test-drill-rotate-motion-reachability
  ()
  (let (answer-list)
    (dotimes (i 5)
      (dotimes (j 10)
        (let (
              (dist (+ 500 (* j 20)))
              (deg (+ -60 (* i 6)))
              )
          (push (list dist deg (test-drill-rotate-motion-reachability-single dist deg)) answer-list)
          )
        )
      )
    answer-list
    )
  )

(defun test-drill-put-motion ()
  ;(get-drill-grasp-motion-full (make-coords :pos (float-vector 700 -150 800) :rpy (list 0 0 0)) :draw? t)
  (setq *answer-for-grasp-test* (get-drill-grasp-motion-full (make-coords :pos (float-vector 1000 -150 200) :rpy (list 0 0 (deg2rad 180))) :draw? t :drill-type :takenoko :recognized-coords-symbol :put-coords :rotation-axis (list :z t t)))
  ;(send *robot* :hand :arms :hook-pose)
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-send-drill-put-motion ()
  (send-drill-put-motion (make-coords :pos (float-vector 800 -150 200) :rpy (list 0 0 0)) :drill-type :takenoko :real? t)
  )

(defun test-drill-button-motion ()
  (set-drill-button-environment-full nil)
  (setq *answer-for-grasp-test* (get-drill-button-motion-full nil :draw? t :use-ri? t))
  )
(defun test-send-drill-button-motion ()
  (send-drill-button-motion nil :real? t)
  )
(defun test-drill-wall-motion ()
  (test-init)
  ;; (setq *drill-coords-list* (list (make-coords :pos (float-vector 1000 -150 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 500) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -250 400) :rpy (list 0 pi/2 0)) (make-coords :pos (float-vector 1000 -50 500) :rpy (list 0 pi/2 0))))
  (setq *drill-coords-list* (list (get-body-relative-coords (make-coords :pos (float-vector 1000 -200 1000) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 200 1000) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 200 1400) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 -200 1400) :rpy (list 0 pi/2 0)))))
  ;(setq *drill-coords-list* (reverse *drill-coords-list*))
  (push-back (elt *drill-coords-list* 0) *drill-coords-list*)

  (setq *answer-for-grasp-test* (get-drill-wall-motion-full *drill-coords-list* :draw? t :drill-type :takenoko :recognized-coords-symbol :edge-coords))
  ;(send *robot* :hand :arms :hook-pose)
  ;(play-motion-on-eus-viewer *drill-motion*)
  )
(defun test-send-drill-wall-motion ()
  (test-init)
  (setq *drill-coords-list* (list (get-body-relative-coords (make-coords :pos (float-vector 1000 -100 1000) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 100 1000) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 100 1200) :rpy (list 0 pi/2 0))) (get-body-relative-coords (make-coords :pos (float-vector 1000 -100 1200) :rpy (list 0 pi/2 0)))))
  (send-drill-wall-motion *drill-coords-list* :real? t :overwrite-stand-coords? t :draw? t)
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
        (t (send *ri* :stop-grasp arm :wait t))
        )
  )
(defun drill-hook2 (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 65.0 -10.0 0.0 -20.0 -20.0))
         (hand-model2real)
         )
        (t (send *ri* :stop-grasp arm :wait t))
        )
  )
(defun drill-hook-after (&key (arm *drill-main-arm*)) ;; test code
  (unix::sleep 5)
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 -40.0))
         (hand-model2real)
         )
        (t (send *ri* :stop-grasp arm :wait t))
        )
  )
(defun drill-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 90.0 -10.0 0.0 -40.0 90.0))
         (hand-model2real)
         )
        (t (send *robot* :start-grasp arm :wait t))
        )
  )

(defun drill-air-grasp (&key (arm *drill-main-arm*))
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 40.0 -10.0 0.0 -40.0 45.0))
         (hand-model2real)
         )
        (t
         ;(send *ri* :move-gripper arm 0.7 :effort 20 :wait t)
         (send *ri* :start-grasp arm :wait t)
           )
        )
  )

(defun drill-t-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 -10.0 0.0 -40.0 -40.0))
         (hand-model2real)
         )
        (t (send *ri* :start-grasp arm :wait t))
        )
  )
(defun drill-i-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90 90.0 0.0 0.0 90.0 90.0))
         (hand-model2real)
         )
        (t (send *ri* :start-grasp arm :wait t))
        )
  )

(defun drill-full-grasp (&key (arm *drill-main-arm*)) ;; test code
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 0.0 0.0 90.0 90.0))
         (hand-model2real)
         )
        (t (send *ri* :start-grasp arm :wait t))
        )
  )
(defun drill-full-grasp2 (&key (arm *drill-main-arm*)) ;; grasp without index finger
  (cond ((or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (send *robot* :hand arm :angle-vector (float-vector 90.0 0.0 -10.0 0.0 -40.0 90.0))
         (hand-model2real)
         )
        (t (send *ri* :start-grasp arm :wait t))
        )
  )

;; (defun get-hrp3-hand-potentio-vector
;;   ()
;;   (let (res)
;;     (setq res (ros::service-call "/HRP3HandControllerServiceROSBridge/getRobotState" (instance jsk_hrp2_ros_bridge::OpenHRP_HRP3HandControllerService_getRobotStateRequest :init)))
;;     (if res
;;         (send res :potentio)
;;       nil
;;       )
;;     )
;;   )

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

(defun do-takenoko-motion-for-jaxon
  (&key (real? nil) (draw? t))
  (setq *av-list* nil)
  (dotimes (i 10)
    (let* ((x 500)
           (y -700)
           (z-max 1300)
           (z-min 975)
           (z (+ (* (- z-max z-min) (/ i 10.0)) z-min))
           )
      (solve-ik-for-do-takenoko-motion-for-jaxon x y z)
      ))
  (dotimes (i 10)
    (let* ((x 500)
           (z 1300)
           (y-max -500)
           (y-min -700)
           (y (+ (* (- y-max y-min) (/ i 10.0)) y-min))
           )
      (solve-ik-for-do-takenoko-motion-for-jaxon x y z)
      ))
  (dotimes (i 10)
    (let* ((x 500)
           (y -500)
           (z-max 1000)
           (z-min 1300)
           (z (+ (* (- z-max z-min) (/ i 10.0)) z-min))
           )
      (solve-ik-for-do-takenoko-motion-for-jaxon x y z)
      ))
  (dotimes (i 10)
    (let* ((x 500)
           (z 1000)
           (y-max -750)
           (y-min -500)
           (y (+ (* (- y-max y-min) (/ i 10.0)) y-min))
           )
      (solve-ik-for-do-takenoko-motion-for-jaxon x y z)
      ))
  (dolist (av *av-list*)
    (send *robot* :angle-vector av)
    (when draw? (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects)))
    (when real?
      (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation))
    )
  )

(defun solve-ik-for-do-takenoko-motion-for-jaxon
  (x y z)
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos (float-vector x y z) :rpy (list -pi/2 0 -pi/2))
        :rotation-axis :z)
  (push (send *robot* :angle-vector) *av-list*)
  ;;(send (make-coords :pos (float-vector x y z) :rpy (list -pi/2 0 -pi/2)) :draw-on :flush t :size 200 :width 5)
  ;;(when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  )

;; (defun tmp-drill
;   ()
;;   ;; generate
;;   (set-drill-environment)
;;   (get-drill-grasp-motion)
;;   ;; grasp
;;   (send *robot* :angle-vector (get-grasp-drill-default-pose))
;;   ;;(send *robot* :angle-vector (elt *drill-motion* 1))
;;   (send *robot* :hand *drill-main-arm* :angle-vector (get-grasp-drill-hand-pose))
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   ;; look
;;   (send *robot* :angle-vector (get-pre-drill-button-pose))
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
;;   (send *robot* :hand (get-opposite-arm *drill-main-arm*) :angle-vector (get-button-drill-hand-pose))
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
;;       (send *ri* :reach-until-touch (get-opposite-arm *drill-main-arm*) (float-vector 0 0 1) :coords-system :local :thre 1.0 :ik-mode :fullbody :dist-step 5 :max-dist 120 :time-step 200 :debug t)
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
;;    (mt ll *robot* (list *drill-main-arm* (get-opposite-arm *drill-main-arm*) :rleg :lleg))
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
;;    (mt ll *robot* (list *drill-main-arm* (get-opposite-arm *drill-main-arm*) :rleg :lleg))
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

;; temporary ;;for jaxon drikk mnotion
;; pre push angle
;;#f(0.391807 0.32324 -17.5873 39.6374 -21.0175 -0.106618 0.391797 0.323802 -17.63 39.9478 -21.2852 -0.107182 -0.800099 0.0169 -0.60547 0.0 0.0 27.9269 27.1869 -33.6532 1.5015 -98.4913 -15.1283 -3.00702 -38.3624 -41.2965 36.1851 47.8688 14.3972 -103.814 -64.6653 48.1323 -27.4725)
;; push angle
;l#f(0.391807 0.32324 -17.5873 39.6374 -21.0175 -0.106618 0.391797 0.323802 -17.63 39.9478 -21.2852 -0.107182 -0.800099 0.0169 -0.60547 0.0 0.0 27.9269 27.1869 -33.6532 1.5015 -98.4913 -15.1283 -3.00702 -38.3624 -44.3288 33.6429 46.331 13.9252 -107.334 -68.2965 41.7428 -23.3786)
;;jaxon impedance
;;(send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 1500)
;; jaxon


;; change log
;; button
;; deg  30 -> 15 -> 22
;; ix   30 -> 15 
;; wall
;; impdedance 700 -> 500
