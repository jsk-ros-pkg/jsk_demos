(require "package://drc_task_common/euslisp/robot-util.l")
(require "package://drc_task_common/euslisp/util.l")

;; hose grasp ;;;;

(defun set-hose-grasp-environment
  (&key (hose-coords (make-coords :pos (float-vector 200 500 800) :rpy (list 0 0 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; attach handle to robot
  (send *robot* :put :larm-hose-grasp-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 5.0 0.0) :local)
         :parent (send (send *robot* :larm :end-coords) :parent)))
  (send *robot* :put :rarm-hose-grasp-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 25.0 -5.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
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
    (when hose-coords
      (send *hose* :newcoords hose-coords)
      ;; treat the difference of origin between eus and recognition program
      (send *hose* :translate (float-vector 0 0 (* -0.5 hose-h)) :local))
    (setq *hose-stand-coords* nil)
    )
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
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
    (set-hose-grasp-environment :hose-coords nil :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *hose* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform hose-coords :local))
    ;; treat the difference of origin between eus and recognition program
    (send *hose* :translate (float-vector 0 0 (* -0.5 (elt (send *hose* :body-type) 2))) :local)
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun fullbody-inverse-kinematics-for-hose-grasp
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

(defun generate-hose-grasp-motion
  (&key (draw? t))
  (let* ((reach-dist 200)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; grasp hose
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    ;; solve ik to grasp hose
    (let* (ik-ret
           tc)
      (when draw? (send (send *hose* :get :grasp-coords) :draw-on :flush t))
      ;; solve ik for grasp
      (setq tc
            (list
             (send *hose* :get :grasp-coords)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (with-move-target-link-list
       (mt ll *robot* '(:larm :rleg :lleg))
       (setf (elt mt 0) (send *robot* :get :larm-hose-grasp-coords))
       (setq ik-ret (fullbody-inverse-kinematics-for-hose-grasp tc mt ll :draw? draw?))
       (when ik-ret
         (setq ik-ret
               (send *robot* :look-at-target (send *hose* :worldcoords)))))
      (push-back (list 1 (not (null ik-ret)) ik-ret) ik-results)
      ;; solve ik for pre grasp
      (setq tc
            (list
             (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) reach-dist 0) :local)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (with-move-target-link-list
       (mt ll *robot* '(:larm :rleg :lleg))
       (setf (elt mt 0) (send *robot* :get :larm-hose-grasp-coords))
       (setq ik-ret (fullbody-inverse-kinematics-for-hose-grasp tc mt ll :draw? draw?))
       (when ik-ret
         (setq ik-ret
               (send *robot* :look-at-target (send *hose* :worldcoords)))))
      (push (list 0 (not (null ik-ret)) ik-ret) ik-results)
      ;; draw
      (when draw? (send *irtviewer* :draw-objects :flush t))
      ;; hose-reset-pose
      (send *robot* :angle-vector (get-hose-reset-pose))
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (push-back (list 2 t (send *robot* :angle-vector)) ik-results)
      (setq *hose-ik-results* ik-results)
      ))
  )

(defun get-hose-grasp-motion
  (&key (draw? nil))
  (generate-hose-grasp-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*))
      (setq angle-vector-ret (mapcar #'(lambda (x) (elt x 2)) *hose-ik-results*)))
    (setq *hose-motion* angle-vector-ret)
    angle-vector-ret
    ))

(defun get-hose-grasp-stand-coords ;; relative to hose
  ()
  (make-coords :pos (float-vector -690 -245 0) :rpy (list 0 0 0)) ;; dependent on arm
  )

(defun get-hose-grasp-motion-full
  (hose-coords &key (draw? nil) (use-ri? t))
  (set-hose-grasp-environment-full hose-coords :draw? draw? :use-ri? use-ri?)
  ;; change stand position
  (let* ((hose-coords-stand-rot (send (send *hose* :copy-worldcoords) :rotate pi :y))
         (stand-coords (send (send hose-coords-stand-rot :copy-worldcoords) :transform (get-hose-grasp-stand-coords) :local))
         (stand-pos (send stand-coords :worldpos))
         (stand-rpy (elt (send stand-coords :rpy-angle) 0))
         )
    (setq *hose-stand-coords* (make-coords :pos (float-vector (elt stand-pos 0) (elt stand-pos 1) 0) :rpy (list (elt stand-rpy 0) 0 0)))
    )
  (ros::ros-warn "change stand position to ~a for hose-grasp motion.~%" *hose-stand-coords*)
  (send *robot* :fix-leg-to-coords *hose-stand-coords*)
  (when draw? (send *irtviewer* :draw-objects))
  (get-hose-grasp-motion :draw? draw?)
  )

(defun send-hose-grasp-motion
  (coords &key (real? t))
  (let* ((hose-motion-av-list (get-hose-grasp-motion-full coords))
         (hose-motion-length (length hose-motion-av-list)))
    (when (not (= hose-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :reset-manip-pose)
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
      ;; walk
      (when *hose-stand-coords*
        (let* ((pos (scale 0.001 (send *hose-stand-coords* :worldpos)))
               (rpy (mapcar #'rad2deg (car (send *hose-stand-coords* :rpy-angle)))))
          (ros::ros-warn "go-pos to ~a ~a ~a for hose~%" (elt pos 0) (elt pos 1) (elt rpy 0))
          (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
          ))
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
  (&key coords (step-time 200) (update? t))
  (when update?
    (get-hose-grasp-motion-full coords :use-ri? nil))
  (ros::ros-info "hose-stand-coords: ~a~%" *hose-stand-coords*)
  (ros::ros-info "hose-ik-results: ~a~%" *hose-ik-results*)
  (cond (*hose-stand-coords*
         (move-robot-model *hose-stand-coords* "ground"))
        (t
         (move-robot-model-to-current-pos)))
  (publish-motion-joint-states *hose-motion* :step-time step-time))


;; hose connect ;;;;

(defun set-hose-connect-environment
  (&key (hose-coords (make-coords :pos (float-vector 0 0 800) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; attach handle to robot
  (send *robot* :put :larm-hose-connect-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 55.0 -25.0 0.0) :local)
         :parent (send (send *robot* :larm :end-coords) :parent)))
  (send *robot* :put :rarm-hose-connect-coords
        (make-cascoords
         :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 55.0 25.0 0.0) :local)
         :parent (send (send *robot* :rarm :end-coords) :parent)))
  ;; hose setting
  (let* ((hose-r 30) (hose-h 50))
    ;; generate hose
    (setq *hose* (make-cylinder hose-r hose-h))
    (send *hose* :set-color (float-vector 1 1 0))
    ;; atattch handle to hose
    (send *hose* :put :connect-coords
          (make-cascoords
           :coords (make-coords :pos (float-vector 0 0 0))
           :parent *hose*))
    ;; place hose to hose-coords
    (when hose-coords
      (send *hose* :newcoords hose-coords)
      ;; treat the difference of origin between eus and recognition program
      (send *hose* :translate (float-vector 0 0 (* -0.5 hose-h)) :local))
    (setq *hose-stand-coords* nil)
    )
  ;; draw
  (when draw?
    (setq *ground* (make-cube 1000 1000 1))
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
    (set-hose-connect-environment :hose-coords nil :draw? draw? :generate-ri? use-ri?)
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

(defun fullbody-inverse-kinematics-for-hose-connect
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
        :thre (list 10 1 1)
        :rthre (list (deg2rad 5) (deg2rad 1) (deg2rad 1))
        :centroid-thre 10
        :cog-gain 5.0
        )
  )

(defun generate-hose-connect-motion
  (&key (draw? t) (try-all-ik? nil))
  (let* ((reach-dist 50)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords))
         (connect-interpolation-num 4)
         (connect-minus-distance -150)
         (connect-plus-distance 80)
         (connect-distance (- connect-plus-distance connect-minus-distance)))
    (when draw? (send (send *hose* :get :connect-coords) :draw-on :flush t))
    ;; loop
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords original-foot-midcoords)
    (dotimes (i connect-interpolation-num)
      ;; solve ik to connect hose
      (let* (ik-ret
             tc
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
         (setf (elt mt 0) (send *robot* :get :larm-hose-connect-coords))
         (setq ik-ret (fullbody-inverse-kinematics-for-hose-connect tc mt ll :draw? draw?))
         (when ik-ret
           (setq ik-ret
                 (send *robot* :look-at-target (send *hose* :worldcoords)))))
        (push-back (list i (not (null ik-ret)) ik-ret) ik-results) ;; connect pose
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        ))
    (when (or try-all-ik? (every #'eval (mapcar #'(lambda (x) (elt x 1)) ik-results)))
      (let* ((ik-results-orig (copy-seq ik-results)))
        (dotimes (i connect-interpolation-num)
          ;; solve ik for hose connector handle
          (let* (ik-ret
                 tc)
            (when (elt (elt (reverse ik-results-orig) i) 1)
              (send *robot* :angle-vector (elt (elt (reverse ik-results-orig) i) 2))
              (send *robot* :fix-leg-to-coords original-foot-midcoords)
              (setq tc
                    (list
                     (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) reach-dist 0) :local)
                     (send *robot* :rleg :end-coords :copy-worldcoords)
                     (send *robot* :lleg :end-coords :copy-worldcoords)))
              (with-move-target-link-list
               (mt ll *robot* '(:larm :rleg :lleg))
               (setq ik-ret (fullbody-inverse-kinematics-for-hose-connect tc mt ll :draw? draw?))
               (when ik-ret
                 (setq ik-ret
                       (send *robot* :look-at-target (send *hose* :worldcoords))))))
            (push-back (list i (not (null ik-ret)) ik-ret) ik-results) ;; connect pose
            ;; draw
            (when draw? (send *irtviewer* :draw-objects :flush t))
            )))
      )
    (setq *hose-ik-num* connect-interpolation-num)
    (setq *hose-ik-results* ik-results))
  )

(defun get-hose-connect-motion
  (&key (draw? nil))
  (generate-hose-connect-motion :draw? draw?)
  (let* (angle-vector-ret)
    (when (every #'eval (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*))
      ;; hose-reset-pose
      (push-back (get-hose-reset-pose) angle-vector-ret)
      ;; connect angle-vector
      (dolist (av (mapcar #'(lambda (x) (elt x 2)) *hose-ik-results*))
        (push-back av angle-vector-ret))
      ;; reset-manip-pose pre1
      (send *robot* :torso :waist-y :joint-angle 20 :relative t) ;; dependent on arm
      (push-back (send *robot* :angle-vector) angle-vector-ret)
      ;; reset-manip-pose pre2
      (send *robot* :larm :angle-vector #f(50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)) ;; dependent on arm
      (push-back (send *robot* :angle-vector) angle-vector-ret)
      ;; reset-manip-pose
      (push-back (send *robot* :reset-manip-pose) angle-vector-ret))
    (setq *hose-motion* angle-vector-ret)
    angle-vector-ret))

(defun get-hose-connect-stand-coords ;; relative to hose
  ()
  (make-coords :pos (float-vector -405 -525 0) :rpy (list 0 0 0)) ;; dependent on arm
  )

(defun get-hose-connect-motion-full
  (hose-coords
   &key (draw? nil) (use-ri? t))
  (set-hose-connect-environment-full hose-coords :draw? draw? :use-ri? use-ri?)
  ;; change stand position
  (let* ((x-vec (send (send (send *hose* :worldcoords) :copy-worldcoords) :rotate-vector (float-vector 0 0 1)))
         (z-vec (float-vector 0 0 1))
         (y-vec (v* z-vec x-vec))
         (hose-coords-stand-rot (make-coords :pos (send (send *hose* :copy-worldcoords) :worldpos) :rot (transpose (matrix x-vec y-vec z-vec))))
         (stand-coords (send (send hose-coords-stand-rot :copy-worldcoords) :transform (get-hose-connect-stand-coords) :local))
         (stand-pos (send stand-coords :worldpos))
         (stand-rpy (elt (send stand-coords :rpy-angle) 0))
         )
    (setq *hose-stand-coords* (make-coords :pos (float-vector (elt stand-pos 0) (elt stand-pos 1) 0) :rpy (list (elt stand-rpy 0) 0 0)))
    )
  (ros::ros-warn "change stand position to ~a for hose-connect motion.~%" *hose-stand-coords*)
  (send *robot* :fix-leg-to-coords *hose-stand-coords*)
  (when draw? (send *irtviewer* :draw-objects))
  (get-hose-connect-motion :draw? draw?)
  )
;;send (send (send *hose* :worldcoords) :copy-worldcoords) :transform a :local

(defun send-hose-connect-motion
  (coords &key (real? t))
  (let* ((hose-motion-av-list (get-hose-connect-motion-full coords))
         (hose-motion-length (length hose-motion-av-list))
         (hose-ik-num *hose-ik-num*))
    (when (not (= hose-motion-length 0))
      (when real? (send *ri* :start-impedance :larm :moment-gain (float-vector 1 1 1) :k-p 200 :d-p 100 :k-r 10 :d-r 5 :m-r 1)) ;; start imp
      ;; hose-reset-pose
      (send *robot* :angle-vector (elt hose-motion-av-list 0))
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      ;; walk
      (when *hose-stand-coords*
        (let* ((pos (scale 0.001 (send *hose-stand-coords* :worldpos)))
               (rpy (mapcar #'rad2deg (car (send *hose-stand-coords* :rpy-angle)))))
          (ros::ros-warn "go-pos to ~a ~a ~a for hose~%" (elt pos 0) (elt pos 1) (elt rpy 0))
          (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
          ))
      ;; connect first pose
      (send *robot* :angle-vector (elt hose-motion-av-list 1))
      (when real? (model2real :time 5000))
      ;; connect
      (when real? (send-angle-vector-sequence-with-time (cdr (subseq *hose-motion* 1 (+ *hose-ik-num* 1))) :time 2000 :wait? t))
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      ;; release
      (when real? (send-angle-vector-sequence-with-time (subseq *hose-motion* (+ *hose-ik-num* 1) (+ *hose-ik-num* *hose-ik-num* 1)) :time 2000 :wait? t))
      ;; reset-manip-pose pre1
      (send *robot* :angle-vector (elt hose-motion-av-list (- hose-motion-length 3)))
      (when real? (model2real :time 5000))
      ;; reset-manip-pose pre2
      (send *robot* :angle-vector (elt hose-motion-av-list (- hose-motion-length 2)))
      (when real? (model2real :time 5000))
      ;; reset-manip-pose
      (send *robot* :angle-vector (elt hose-motion-av-list (- hose-motion-length 1)))
      (when real? (model2real :time 5000))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 800 :d-p 400)) ;; set default imp param
      ))
  )

(defun publish-hose-connect-motion
  (&key coords (step-time 200) (update? t))
  (when update?
    (get-hose-connect-motion-full coords :use-ri? nil))
  (ros::ros-info "hose-stand-coords: ~a~%" *hose-stand-coords*)
  (ros::ros-info "hose-ik-results: ~a~%" *hose-ik-results*)
  (cond (*hose-stand-coords*
         (move-robot-model *hose-stand-coords* "ground"))
        (t
         (move-robot-model-to-current-pos)))
  (publish-motion-joint-states *hose-motion* :step-time step-time)
  )


;; hose util ;;;;

(defun get-hose-reset-pose ()
  (float-vector 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 -11.8504 28.6898 -41.2121 -109.707 -61.224 -14.2529 43.9426 -60.0))

(defun test-hose-grasp-motion1
  ()
  (get-hose-grasp-motion-full (make-coords :pos (float-vector 700 250 350) :rpy (list pi 0 pi)) :draw? nil :use-ri? t)
  (when *hose-stand-coords*
    (send *robot* :fix-leg-to-coords *hose-stand-coords*))
  (play-motion-on-eus-viewer *hose-motion*)
  )

(defun test-hose-connect-motion1
  ()
  (get-hose-connect-motion-full (make-coords :pos (float-vector 400 550 405) :rpy (list 0.0 1.571 0.0)) :draw? nil :use-ri? t)
  (when *hose-stand-coords*
    (send *robot* :fix-leg-to-coords *hose-stand-coords*))
  (play-motion-on-eus-viewer *hose-motion*)
  )
