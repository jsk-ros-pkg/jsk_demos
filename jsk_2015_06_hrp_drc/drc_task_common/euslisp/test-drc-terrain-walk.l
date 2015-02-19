;;;;;;;;;;;;;;;;
;; utilities
;;;;;;;;;;;;;;;;
(defun walking-pose ;; this should be method
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose))
  "Generate and set walking pose.
   default-pose-method is initial pose, reset-manip-pose by default.
   Generated pose is near from default-pose-method pose.
   root-link-height-offset is root height offset [mm] from default-pose-method.
   root-link-pitch-offset is root pitch offset [deg] from default-pose-method."
  (send robot default-pose-method)
  (send robot :fix-leg-to-coords fix-coords)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (send robot :move-coords
          (send
           (send (send (car (send robot :links)) :copy-worldcoords)
                 :translate (float-vector 0 0 root-link-height-offset))
           :rotate (deg2rad root-link-pitch-offset) :y)
          (car (send robot :links)))
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (send robot :angle-vector)
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Stair walking functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun test-stair-walking-common
  (robot stair-height
         &key (fwd-stride 250) (stair-once nil) (y-or-n-p nil)
              (initial-support-leg :lleg))
  (let ((counter-leg (case initial-support-leg (:rleg :lleg) (:lleg :rleg))))
    (if stair-once
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send robot initial-support-leg :end-coords :copy-worldcoords)
                            :name initial-support-leg)
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               (make-coords :coords (send (send robot initial-support-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name initial-support-leg)))
      (progn
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send robot initial-support-leg :end-coords :copy-worldcoords)
                            :name initial-support-leg)
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               ))
        (if y-or-n-p
            (y-or-n-p ";; next step?"))
        (send *ri* :set-foot-steps
              (list
               (make-coords :coords (send (send robot counter-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name counter-leg)
               (make-coords :coords (send (send robot initial-support-leg :end-coords :copy-worldcoords) :translate (float-vector fwd-stride 0 stair-height))
                            :name initial-support-leg)
               ))))
    ))

;; thin 40[mm] wooden bar
(defun test-stair-walk-0-up
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 40.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-0-down
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -40.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; thick 88[mm] wooden bar
(defun test-stair-walk-1-up
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 88.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-1-down
  (robot &key (fwd-stride 250) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -88.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block 150[mm], thin side
(defun test-stair-walk-2-up
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 150.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-2-down
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -150.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block, thick side, 190[mm] Qualification block
(defun test-stair-walk-3-up
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot 190.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-3-down
  (robot &key (fwd-stride 270) (stair-once nil) (initial-support-leg :lleg))
  (test-stair-walking-common robot -190.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block, thick side, 20[mm] Qualification block
(defun test-stair-walk-4-up
  (robot &key (fwd-stride 270) (stair-once t) (initial-support-leg :lleg))
  (test-stair-walking-common robot 200.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

(defun test-stair-walk-4-down
  (robot &key (fwd-stride 270) (stair-once t) (initial-support-leg :lleg))
  (test-stair-walking-common robot -200.0 :fwd-stride fwd-stride :stair-once stair-once :initial-support-leg initial-support-leg))

;; concrete block stair in TerrainFloor.wrl
(defun test-stair-walk-5-up
  (robot &key (fwd-stride 250) (stair-once nil))
  (test-stair-walking-common robot (* 1e3 0.1524) :fwd-stride fwd-stride :stair-once stair-once))

(defun test-stair-walk-5-down
  (robot &key (fwd-stride 250) (stair-once nil))
  (test-stair-walking-common robot (* -1e3 0.1524) :fwd-stride fwd-stride :stair-once stair-once))

;;;;;;;;
;; demo functions
;;;;;;;;
(defun setup-terrain-walk-hrp2jsknts
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (objects (list *robot*))
  (walking-pose *robot* :root-link-height-offset -70)
  )
(warn ";; (setup-terrain-walk-hrp2jsknts)~%")

(defun setup-terrain-walk-real-hrp2jsknts
  ()
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :set-gait-generator-param
        :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :swing-trajectory-delay-time-offset 0.25
        :default-orbit-type :stair
        :default-step-time 1.3
        :default-double-support-ratio 0.16
        :default-step-height 0.03)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 5 30 0) (float-vector 5 -30 0)))
  )
(warn ";; (setup-terrain-walk-real-hrp2jsknts)~%")

(defun test-15cm-block-hrp2jsknts (&key (stair-once t))
  (print ";; up")
  (test-stair-walk-2-up *robot* :stair-once stair-once)
  (print ";; go pos fwd")
  (read-line)
  (send *ri* :go-pos 0.2 0 0)
  (print ";; down")
  (read-line)
  (test-stair-walk-2-down *robot* :stair-once stair-once :fwd-stride 240)
  )

