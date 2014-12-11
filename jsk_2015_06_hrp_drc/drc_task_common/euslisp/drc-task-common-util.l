(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")

(defmethod rtm-ros-robot-interface
  (:reach-until-touch
   (limb dir
         &key (thre 10) (max-dist 150) (dist-step 10) (time-step 200)
         (coords-system :world) (rotation-axis t) (debug nil))
   (unless (consp limb) (setq limb (list limb)))
   (unless (consp dir) (setq dir (list dir)))
   (let* ((tmp-robot (copy-object robot))
          (ndir (mapcar #'normalize-vector dir))
          (initial-force (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb))
          (finish-condition (make-list (length limb) :initial-element nil)))
     ;; 1. initialize
     (send tmp-robot :angle-vector (send self :state :potentio-vector))
     (send tmp-robot :fix-leg-to-coords (make-coords))
     (send self :set-interpolation-mode ":linear")
     (when debug (warn ";; ndir: ~a initial-force: ~a~%" ndir initial-force))
     ;; 2. loop
     (dotimes (i (round (/ max-dist dist-step)))
      ;; 2.1 check whether end condition is satisfied
      (let* ((current-force (mapcar #'(lambda (l) (send self :state :absolute-force-vector l)) limb))
             (diff-force (mapcar #'v- current-force initial-force))
             (diff-dired-force (mapcar #'(lambda (df nd) (- (v. df nd))) diff-force ndir)))
        (setq finish-condition (mapcar #'(lambda (l fc ddf) (or fc (when (> ddf thre) (warn "limb ~a touched~%" l) t)))
                                       limb finish-condition diff-dired-force))
        (when debug (warn ";; diff-dired-force: ~a thre: ~a finish-cond: ~a~%" diff-dired-force thre finish-condition))
        (unless (some #'not finish-condition)
          (return-from nil nil)))
      ;; 2.2 solve ik and move real robot
      (let* ((ik-res (mapcar #'(lambda (l nd fc) (unless fc (send tmp-robot l :move-end-pos (scale dist-step nd) coords-system))) limb ndir finish-condition)))
        (unless ik-res
          (warn ";; ik failed~%")
          (return-from nil nil)))
      (send self :angle-vector (send tmp-robot :angle-vector) time-step)
      (unix::usleep (round (* (* time-step 1000) 0.8)))
      )
     ;; 3. send final posture again
     ;;(send self :angle-vector (send self :state :potentio-vector) 500)
     (send self :wait-interpolation)
     (unix::usleep (* 100 1000))
     (send self :set-interpolation-mode ":hoffarbib")
     )
   ))

(defmethod hrp2jsknts-robot
  (:reach-drill-walk-pose
   ()
   (send self :angle-vector
         #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 10.7134 -42.5314 0.849741 -134.051 50.0635 22.293 25.1515 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
  (:reach-drill-pre-pose
   ()
   (send self :angle-vector
         #f(-8.144307e-06 -0.000153 -36.5591 70.8929 -34.3338 0.000178 0.0 8.144307e-06 0.000153 -36.5591 70.8929 -34.3338 -0.000178 0.0 40.0 0.0 0.0 40.0 -56.1706 -73.4951 92.0 -127.965 106.0 37.3389 20.1853 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
  (:pierce-drill-pre-pose
   ()
   (send self :angle-vector
         #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 1.19212 -20.9065 -5.36626 -98.3812 -17.2023 6.28459 -83.1406 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)))
  )

;; (send *ri* :reach-until-touch :rarm #f(0 0 1) :coords-system :local :thre 10)
