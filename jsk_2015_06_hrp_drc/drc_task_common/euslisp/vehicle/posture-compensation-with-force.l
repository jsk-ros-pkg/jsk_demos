#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")

(setq *robot* nil)
(setq *old-ref-force* #f(0 0 0))

(defun init
  (&optional (limb :rarm))
  (staro-init)
  (setq *robot* *staro*)
  (reset :rarm)
  (objects (list *robot*))
  )

(defun reset
  (&optional (limb :rarm))
  ;; (send *ri* :stop-impedance limb)
  (setq *old-ref-force* #f(0 0 0))
  (send *ri* :set-ref-force #f(0 0 0) 20000 limb)
  )

(defun sitting-pose
  (&key (real nil))
  (send *staro* :angle-vector #f(43.6687 -27.625 28.2562 178.606 -74.3625 -51.025 13.3875 112.494 -10.0437 -60.0375 89.9625 -90.025 59.95 0.0 0.0 45.0 0.048 0.024 -70.068 40.068 -15.024 0.0 -0.06 -0.012 -70.116 40.02 -10.008 0.0 -0.03125 0.00625 30.0375 0.01875))
  (when real
    (send *ri* :angle-vector (send *staro* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  )

(defun grasp-handle
  (&key (limb :rarm) (real nil))
  (send *ri* :hand limb :open)
  (send *staro* :angle-vector #f(28.1687 8.125 39.7687 148.762 -83.4187 -23.4062 10.5625 99.9125 -10.0562 -60.0625 89.95 -90.0312 59.9437 0.00625 0.0 45.0 0.072 0.048 -70.068 40.092 -15.048 -0.01195 -0.084 -0.024 -70.152 40.032 -10.008 -0.01195 -0.0375 0.0125 30.0875 0.01875))
  (when real
    (send *ri* :angle-vector (send *staro* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  (send *ri* :start-impedance :limb :M-p 80 :D-p 300 :K-p 500)
  )

(defun fix-end-coords-to-ground
  (&key (limb :rarm) (real nil))
  (send *ri* :state)  ;; update
  (send *robot* :newcoords (send (car (send *ri* :robot :links)) :copy-worldcoords))  ;; adapt
  (send *robot* limb :inverse-kinematics (make-coords) :translation-axis nil) ;; ik to fix ground
  (when real
    (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    )
  ;; (send *robot* limb :move-end-pos #f(0 0 -200) :world)  ;; approacch
  ;; (when real
  ;;   (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  ;;   (send *ri* :wait-interpolation)
  ;;   )
  )

(defun get-current-ypr
  ()
  (let (body-coords current-ypr)
    (send *robot* :angle-vector (send *ri* :state :potentio-vector)) ;; update-status
    (setq body-coords (send *robot* :newcoords (send (car (send *ri* :robot :links)) :copy-worldcoords)))
    (setq current-ypr (mapcar #'rad2deg (car (rpy-angle (send body-coords :worldrot)))))
    current-ypr
    )
  )

(defun update-ref-force
  (&key (limb :rarm) (gain 0.5) (tm 5000) (ref-roll 0.0) (max-force 150))
  (let* ((ypr (get-current-ypr))
         (roll (elt ypr 2))
         (ref-force-vector #f(0 0 0))
         )
    (setf (elt ref-force-vector 2) (min (+ (elt *old-ref-force* 2) (* gain (- ref-roll roll))) max-force))
    (send *ri* :set-ref-force ref-force-vector tm limb)
    (unix::usleep (* tm 1000))
    (format t "ref-force-vector : ~A~%ypr : ~A~%" ref-force-vector ypr)
    (setq *old-ref-force* ref-force-vector)
    ref-force-vector
    )
  )

(defun main
  ()
  ;; (send *ri* :start-impedance :rarm :M-p 80 :D-p 300 :K-p 500)
  (do-until-key
   (update-ref-force :limb :rarm :gain 0.5 :tm 2000 :ref-roll 0.0 :max-force 120)
   (send *irtviewer* :draw-objects :flush t)
   (x::window-main-one)
   )
  (if (y-or-n-p "~%reset (ref-force 0 in 20s)?~%y: ref-force 0, n: ref-force unchanged~%")
      (progn
	(print "reset~%")
	(reset :rarm)
	)
    )
  (print "end main~%")
  )

;; (progn
;;   (do-until-key
;;    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
;;    (let* ((ref-roll (rad2deg 0))
;;           (K 0.5)
;;           (body-coords (send *robot* :newcoords (send (car (send *ri* :robot :links)) :copy-worldcoords)))
;;           (ypr (mapcar #'rad2deg (car (rpy-angle (send body-coords :worldrot)))))
;;           (roll (elt ypr 2))
;;           (tm 5000)
;;           )
;;      (setf (elt *ref-force-vector* 2) (min (+ *ref-force-norm* (* K (- ref-roll roll))) 150))
;;      (send *ri* :set-ref-force *ref-force-vector* tm :rarm)
;;      (unix::usleep (* tm 1000))
;;      (format t "*ref-force-norm* : ~A~%*ref-force-vector* : ~A~%ypr : ~A~%" *ref-force-norm* *ref-force-vector* ypr)
;;      )
;;    (setq *ref-force-norm* (elt *ref-force-vector* 2))
;;    (send *irtviewer* :draw-objects :flush t)
;;    (x::window-main-one)
;;    )
;;   ;; (send *ri* :set-ref-force #f(0 0 0) 10000 :rarm)
;;   )
;; (progn
;;   ;; update                                                                                                             
;;   (send *ri* :state)
;;   ;; adapt                                                                                                              
;;   (send *staro* :newcoords (send (car (send *ri* :robot :links)) :copy-worldcoords))
;;   ;; ik                                                                                                                 
;;   (send *staro* :rarm :inverse-kinematics (send (make-coords) :rotate -pi/2 :y) :translation-axis nil)
;;   ;; approacch                                                                                                          
;;   (send *staro* :rarm :move-end-pos #f(0 -200 0) :world)
;;   )



