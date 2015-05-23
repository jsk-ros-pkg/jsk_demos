(require "package://drc_task_common/euslisp/robot-util.l")
(require "package://drc_task_common/euslisp/util.l")

(defun set-valve-environment
  (&key (valve-r 200)
        (valve-coords (make-coords :pos (float-vector 0 0 1200) :rpy (list 0 pi/2 0)))
        (valve-coords-system :world)
        (stand-coords nil)
        (generate-ri? t) (draw? t) (use-ri? t)
        (arm :rarm)
        (robot-name "HRP2JSKNTS"))
  ;; generate robot
  (generate-robot robot-name :generate-ri? generate-ri?)
  ;; place robot to origin ;; this should be done before attaching handle to robot
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve-arm* arm)
  ;; attach handle to robot
  (attach-handle-to-robot-for-valve)
  ;; generate valve
  (let* ((valve-sr (* 0.1 valve-r)))
    (setq *valve* (make-ring valve-r valve-sr :segments 16))
    (send *valve* :set-color (float-vector 1 1 0))
    )
  ;; atattch handle to valve
  (attach-handle-to-object-for-valve valve-r)
  ;; place valve to valve-coords ;; this should be done after attaching handle to valve
  (when valve-coords
    (cond ((equal valve-coords-system :world)
           (send *valve* :newcoords valve-coords)
           )
          (t ;; (equal valve-coords-system :body)
           (cond (use-ri?
                  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
                 (t
                  (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
           (send *robot* :fix-leg-to-coords (make-coords))
           (send *valve* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform valve-coords :local))
           )))
  ;; set robot stand-coords
  (setq *valve-stand-coords* (if stand-coords stand-coords (make-coords)))
  (send *robot* :fix-leg-to-coords *valve-stand-coords*)
  (setq *valve-motion* nil)
  (setq *valve-ik-results* nil)
  ;; draw
  (setq *ground* (make-cube 1000 1000 1))
  (when (or draw? (boundp '*irtviewer*))
    (objects (list *robot* *valve* *ground*)))
  (when draw?
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )

(defun attach-handle-to-robot-for-valve
  ()
  (let* (larm-translate-offset
         rarm-translate-offset
         larm-z-rotate-offset
         rarm-z-rotate-offset)
    ;; handle for valve edge
    (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
           (setq larm-translate-offset (float-vector -10.0 0.0 0.0))
           (setq rarm-translate-offset (float-vector -10.0 0.0 0.0))
           (setq larm-z-rotate-offset (deg2rad 0))
           (setq rarm-z-rotate-offset (deg2rad -0))
           )
          (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
           (setq larm-translate-offset (float-vector 25.0 5.0 0.0))
           (setq rarm-translate-offset (float-vector 25.0 -5.0 0.0))
           (setq larm-z-rotate-offset 0)
           (setq rarm-z-rotate-offset 0)))
    (send *robot* :put :larm-valve-coords
          (make-cascoords
           :coords (send (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate larm-translate-offset :local) :rotate pi :x :local) :rotate larm-z-rotate-offset :z :local)
           :parent (send (send *robot* :larm :end-coords) :parent)))
    (send *robot* :put :rarm-valve-coords
          (make-cascoords
           :coords (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate rarm-translate-offset :local) :rotate rarm-z-rotate-offset :z :local)
           :parent (send (send *robot* :rarm :end-coords) :parent)))
    ;; handle for valve center
    (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
           (setq larm-translate-offset (float-vector -10.0 0.0 0.0))
           (setq rarm-translate-offset (float-vector -10.0 0.0 0.0))
           (setq larm-z-rotate-offset (deg2rad 0))
           (setq rarm-z-rotate-offset (deg2rad -0))
           )
          (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
           (setq larm-translate-offset (float-vector 40.0 -15.0 0.0))
           (setq rarm-translate-offset (float-vector 40.0 15.0 0.0))
           (setq larm-z-rotate-offset (deg2rad -55))
           (setq rarm-z-rotate-offset (deg2rad 55))))
    (send *robot* :put :larm-valve-coords-for-center
          (make-cascoords
           :coords (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate larm-translate-offset :local) :rotate larm-z-rotate-offset :z :local)
           :parent (send (send *robot* :larm :end-coords) :parent)))
    (send *robot* :put :rarm-valve-coords-for-center
          (make-cascoords
           :coords (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate rarm-translate-offset :local) :rotate rarm-z-rotate-offset :z :local)
           :parent (send (send *robot* :rarm :end-coords) :parent)))
    )
  )

(defun attach-handle-to-object-for-valve
  (valve-r)
  (send *valve* :put :grasp-coords
        (make-cascoords
         :coords
         (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
                (send (make-coords :pos (float-vector valve-r 0 0) :rpy (list pi/2 -pi/2 0)) :rotate (deg2rad 10) :z :local))
               (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
                (make-coords :pos (float-vector valve-r 0 0) :rpy (list pi/2 -pi/2 0))))
         :parent *valve*))
  (send *valve* :put :center-grasp-coords
        (make-cascoords
         :coords (make-coords :pos (float-vector 0 0 0) :rpy (list pi/2 -pi/2 0))
         :parent *valve*))
  )

(defun reset-valve-angle
  ()
  (let* ((handle-height-min 1e10)
         valve-rot-min)
    (dotimes (i 360)
      (send *valve* :rotate (deg2rad 1) :z :local)
      (when (> handle-height-min (elt (send (send *valve* :get :grasp-coords) :worldpos) 2))
        (setq handle-height-min (elt (send (send *valve* :get :grasp-coords) :worldpos) 2))
        (setq valve-rot-min (send (send (send *valve* :worldcoords) :copy-worldcoords) :worldrot))
        )
      )
    (send *valve* :newcoords (make-coords :pos (send (send *valve* :worldcoords) :worldpos) :rot valve-rot-min))
    )
  )


(defun fullbody-inverse-kinematics-for-valve
  (tc mt ll &key (draw? nil))
  (cond ((equal *robot-name* "JAXON")
         (fullbody-inverse-kinematics-for-valve-for-jaxon tc mt ll :draw? draw?))
        ((equal *robot-name* "STARO")
         (fullbody-inverse-kinematics-for-valve-for-staro tc mt ll :draw? draw?))
        (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (fullbody-inverse-kinematics-for-valve-for-hrp2 tc mt ll :draw? draw?))
        ))

(defun fullbody-inverse-kinematics-for-valve-for-hrp2
  (tc mt ll &key (draw? nil))
  (send *robot*
        :fullbody-inverse-kinematics tc
        :move-target mt :link-list ll
        :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
        :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
        :translation-axis (list t t t)
        :rotation-axis (list t t t)
        :warnp (if draw? t nil)
        :additional-weight-list
        (list (list (send *robot* :rleg :toe-p :child-link) 0)
              (list (send *robot* :lleg :toe-p :child-link) 0))
        :max (float-vector  500  500  0  20  20  10)
        :min (float-vector  -500  -500  -200  -20  -20  -10)
        :stop 20
        :debug-view (if draw? :no-message nil)
        ))

(defun fullbody-inverse-kinematics-for-valve-for-jaxon
  (tc mt ll &key (draw? nil))
  (let (ret)
    (setq ret
          (send *robot*
                :fullbody-inverse-kinematics tc
                :move-target mt :link-list ll
                :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                :translation-axis (list t t t)
                :rotation-axis (list t t t)
                :warnp (if draw? t nil)
                :root-link-virtual-joint-weight (float-vector 0.15 0.15 0.02 0.15 0.5 0.5)
                :additional-weight-list
                (list (list (send *robot* :torso :waist-r :child-link) 0.01)
                      (list (send *robot* :torso :waist-y :child-link) 0.1)
                      (list (send *robot* :torso :waist-p :child-link) 0.1)
                      (list (send *robot* :rarm :shoulder-p :child-link) 0)
                      (list (send *robot* :larm :shoulder-p :child-link) 0)
                      )
                :max (float-vector  500  200  15  10  20  10)
                :min (float-vector  -500  -200  -200  -10  -20  -10)
                :stop 50
                :debug-view (if draw? :no-message nil)
                :avoid-collision-distance 100
                :avoid-collision-null-gain 5.0
                :avoid-collision-joint-gain 0.8
                :collision-avoidance-link-pair
                (list (list (elt (send *robot* *valve-arm* :links) 2) (send *robot* :link "CHEST_LINK2")))
                ))
    (when (and ret (remove-if  #'(lambda (l) (or (and (eq (car l) (send *robot* :link "CHEST_LINK2")) (eq (cdr l) (send *robot* :link "HEAD_LINK1"))) (and (eq (cdr l) (send *robot* :link "CHEST_LINK2")) (eq (car l) (send *robot* :link "HEAD_LINK1"))))) (send *robot* :self-collision-check)))
      (setq ret nil))
    ret)
  )

(defun fullbody-inverse-kinematics-for-valve-for-staro
  (tc mt ll &key (draw? nil))
  (let (ret)
    (setq ret
          (send *robot*
                :fullbody-inverse-kinematics tc
                :move-target mt :link-list ll
                :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                :translation-axis (list t t t)
                :rotation-axis (list t t t)
                :warnp (if draw? t nil)
                :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5)
                :additional-weight-list
                (list (list (send *robot* :torso :waist-y :child-link) 0)
                      (list (send *robot* :torso :waist-p :child-link) 0))
                :max (float-vector  500  200  25  10  20  10)
                :min (float-vector  -500  -200  -200  -10  -20  -10)
                :stop 50
                :debug-view (if draw? :no-message nil)
                :avoid-collision-distance 100
                :avoid-collision-null-gain 5.0
                :avoid-collision-joint-gain 3.0
                :collision-avoidance-link-pair
                (list (list (caddr (send *robot* :lleg :links))
                            (car (last (send *robot* :larm :links)))))
                ))
    (when (and ret (send *robot* :self-collision-check)) (setq ret nil))
    ret)
  )

(defun generate-valve-motion
  (&key (draw? t) (grasp-center? nil) (ccw? t))
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* ((devide-num 10)
         (reach-dist 100)
         (ik-results nil)
	 (valve-handle-name :grasp-coords)
	 )
    (when grasp-center?
      (setq valve-handle-name :center-grasp-coords))
    ;; rotate valve
    (dotimes (i devide-num)
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords *valve-stand-coords*)
      ;; solve ik to rotate valve
      (let* (ik-ret-in
             ik-ret-out
             tc)
        (when draw? (send (send *valve* :get valve-handle-name) :draw-on :flush t))
        ;; solve ik for valve handle
        (setq tc
              (list
               (send *valve* :get valve-handle-name)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (with-move-target-link-list
         (mt ll *robot* (list *valve-arm* :rleg :lleg))
         (setf (elt mt 0) (send *robot* :get (read-from-string (format nil "~a-valve-coords~a" *valve-arm* (if grasp-center? "-for-center" "")))))
         (setq ik-ret-in (fullbody-inverse-kinematics-for-valve tc mt ll :draw? draw?))
         (when ik-ret-in
           (setq ik-ret-in (send *robot* :look-at-target (send *valve* :worldcoords)))))
        ;; solve ik for valve reaching point
        (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
               (setq tc
                     (list
                      (send (send (send *valve* :get valve-handle-name) :copy-worldcoords) :translate (float-vector (* (- reach-dist) 2.0) 0 0) :local)
                      (send *robot* :rleg :end-coords :copy-worldcoords)
                      (send *robot* :lleg :end-coords :copy-worldcoords))))
              (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
               (setq tc
                     (list
                      (cond (grasp-center?
                             (send (send (send *valve* :get valve-handle-name) :copy-worldcoords) :translate (float-vector (- reach-dist) 0 0) :local))
                            (t ;; (not grasp-center?)
                             (send (send (send *valve* :get valve-handle-name) :copy-worldcoords) :translate (float-vector (- reach-dist) (- reach-dist) 0) :local)))
                      (send *robot* :rleg :end-coords :copy-worldcoords)
                      (send *robot* :lleg :end-coords :copy-worldcoords)))))
        (when ik-ret-in
          (with-move-target-link-list
           (mt ll *robot* (list *valve-arm* :rleg :lleg))
           (setf (elt mt 0) (send *robot* :get (read-from-string (format nil "~a-valve-coords~a" *valve-arm* (if grasp-center? "-for-center" "")))))
           (setq ik-ret-out (fullbody-inverse-kinematics-for-valve tc mt ll :draw? draw?))
           (when ik-ret-out
             (setq ik-ret-out (send *robot* :look-at-target (send *valve* :worldcoords))))))
        (push-back (list i (not (null (and ik-ret-in ik-ret-out))) ik-ret-in ik-ret-out) ik-results)
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        )
      (setq *valve-ik-results* ik-results)
      ;; rotate valve
      (send *valve* :rotate (* (if ccw? -1 1) (deg2rad (/ 360.0 devide-num))) :z :local)
      )
    ))

(defun search-rotatable-range
  (&key (arm *valve-arm*) (check-continuous? t))
  (let* ((ik-results-double (append *valve-ik-results* *valve-ik-results*))
         (test-point 0)
         (rotatable-point nil))
    (dotimes (i (/ (length ik-results-double) 2))
      (let (j)
        (setq test-point 0)
        (setq j (+ i 1))
        (when (car (cdr (nth i ik-results-double)))
          (incf test-point)
          (while (and
                  (car (cdr (nth j ik-results-double)))
                  (or (not check-continuous?)
                      (check-angle-vector-continuous (elt (elt ik-results-double (1- j)) 3) (elt (elt ik-results-double j) 3) arm)
                      ;; (and
                      ;;  (check-angle-vector-continuous (elt (elt ik-results-double (1- j)) 2) (elt (elt ik-results-double j) 2) arm)
                      ;;  (check-angle-vector-continuous (elt (elt ik-results-double (1- j)) 3) (elt (elt ik-results-double j) 3) arm))
                      ))
            (incf test-point)
            (incf j)))
        (push-back test-point rotatable-point)
        ))
    (unless rotatable-point
      (setq rotatable-point (list 0)))
    rotatable-point
    ))

(defun get-rotatable-angle
  ()
  (* 360 (/ (float (apply #'max (search-rotatable-range))) (length *valve-ik-results*))))

(defun get-valve-motion
  (&key (draw? nil) (grasp-center? nil))
  (generate-valve-motion :draw? draw? :grasp-center? grasp-center?)
  (let* ((valve-rotatable-range (search-rotatable-range))
         (rotatable-point-max (apply #'max valve-rotatable-range))
         (rotatable-point-idx (position (apply #'max valve-rotatable-range) valve-rotatable-range))
         angle-vector-ret angle-vector-ret-release)
    (dotimes (i rotatable-point-max)
      (push-back (elt (elt *valve-ik-results* rotatable-point-idx) 2) angle-vector-ret)
      (push-back (elt (elt *valve-ik-results* rotatable-point-idx) 3) angle-vector-ret-release)
      (setq rotatable-point-idx (mod (+ rotatable-point-idx 1) (length valve-rotatable-range)))
      )
    (setq *valve-motion* (append angle-vector-ret (reverse angle-vector-ret-release)))
    *valve-motion*
    ))

(defun get-valve-stand-coords ;; relative to valve
  (&key (grasp-center? nil))
  (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
         (cond ((equal *valve-arm* :larm)
                (make-coords :pos (float-vector -600 -500 0) :rpy (list (deg2rad -10) 0 0)))
               ((equal *valve-arm* :rarm)
                (make-coords :pos (float-vector -600 +500 0) :rpy (list (deg2rad +10) 0 0))))
         )
        (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
         (cond (grasp-center?
                (cond ((equal *valve-arm* :larm)
                       (make-coords :pos (float-vector -400 -500 0) :rpy (list (deg2rad 0) 0 0)))
                      ((equal *valve-arm* :rarm)
                       (make-coords :pos (float-vector -400 500 0) :rpy (list (deg2rad 0) 0 0)))))
               (t
                (cond ((equal *valve-arm* :larm)
                       (make-coords :pos (float-vector -500 -120 0) :rpy (list (deg2rad 0) 0 0)))
                      ((equal *valve-arm* :rarm)
                       (make-coords :pos (float-vector -500 120 0) :rpy (list (deg2rad 0) 0 0))))
                ))
         ))
  )

(defun move-robot-with-stand-coords-for-valve
  (stand-coords)
  (let* ((x-vec (send (send (send *valve* :worldcoords) :copy-worldcoords) :rotate-vector (float-vector 0 0 1)))
         (z-vec (float-vector 0 0 1))
         (y-vec (v* z-vec x-vec))
         (valve-coords-stand-rot (make-coords :pos (send (send *valve* :copy-worldcoords) :worldpos) :rot (transpose (matrix x-vec y-vec z-vec))))
         (tmp-stand-coords (send (send valve-coords-stand-rot :copy-worldcoords) :transform stand-coords :local))
         (stand-pos (send tmp-stand-coords :worldpos))
         (stand-rpy (elt (send tmp-stand-coords :rpy-angle) 0))
         )
    (setq *valve-stand-coords* (make-coords :pos (float-vector (elt stand-pos 0) (elt stand-pos 1) 0) :rpy (list (elt stand-rpy 0) 0 0)))
    )
  )

(defun get-valve-motion-full
  (valve-r
   valve-coords
   &key (valve-coords-system :body) (arm :rarm) (grasp-center? nil) (stand-coords nil)
   (overwrite-stand-coords? nil) (overwrite-arm-side? nil) (draw? nil) (draw-environment? nil) (use-ri? t)
   (robot-name "HRP2JSKNTS")
   )
  (let* ((rotatable-angle-thre 100))
    (set-valve-environment
     :valve-r valve-r :valve-coords valve-coords :valve-coords-system valve-coords-system
     :stand-coords stand-coords :arm arm :draw? (or draw? draw-environment?)
     :use-ri? use-ri? :generate-ri? use-ri? :robot-name robot-name)
    ;; overwrite of grasp-mode is not implemented yet
    (cond (grasp-center?
           (setq *valve-grasp-mode* :center-grasp))
          (t
           (setq *valve-grasp-mode* :edge-grasp)))
    ;; first trial
    (when (not (equal overwrite-stand-coords? :force))
      (get-valve-motion :grasp-center? grasp-center? :draw? draw?))
    ;; change arm
    (when (and overwrite-arm-side? (not (equal overwrite-stand-coords? :force)) (<= (get-rotatable-angle) rotatable-angle-thre))
      (setq *valve-arm* (get-opposite-arm *valve-arm*))
      (ros::ros-warn "change arm side to ~a for valve motion. (stand position is ~a.) ~%" *valve-arm* *valve-stand-coords*)
      (get-valve-motion :grasp-center? grasp-center? :draw? draw?)
      )
    ;; revert arm, change stand position
    (when (and overwrite-stand-coords? (<= (get-rotatable-angle) rotatable-angle-thre))
      (when (and overwrite-arm-side? (not (equal overwrite-stand-coords? :force)))
        (setq *valve-arm* (get-opposite-arm *valve-arm*)))
      (move-robot-with-stand-coords-for-valve (get-valve-stand-coords :grasp-center? grasp-center?))
      (ros::ros-warn "change stand position to ~a for valve motion. (arm side is ~a.)~%" *valve-stand-coords* *valve-arm*)
      (get-valve-motion :grasp-center? grasp-center? :draw? draw?)
      )
    ;; change arm, change stand position
    (when (and overwrite-arm-side? overwrite-stand-coords? (<= (get-rotatable-angle) rotatable-angle-thre))
      (when overwrite-arm-side? (setq *valve-arm* (get-opposite-arm *valve-arm*)))
      (move-robot-with-stand-coords-for-valve (get-valve-stand-coords :grasp-center? grasp-center?))
      (ros::ros-warn "change arm side to ~a and stand position to ~a for valve motion.~%" *valve-arm* *valve-stand-coords*)
      (get-valve-motion :grasp-center? grasp-center? :draw? draw?)
      )
    *valve-motion*))

(defun send-valve-motion
  (radius
   coords
   &key (arm :rarm) (overwrite-arm-side? nil)
   (stand-coords nil) (overwrite-stand-coords? nil)
   (grasp-center? nil) (overwrite-grasp-mode? nil)
   (real? t) (test-mode? nil) (av-list nil))
  (let* ((rotate-num 3)
         (valve-motion-av-list
          (if av-list
              av-list
            (get-valve-motion-full
             radius coords
             :valve-coords-system :body
             :arm arm :overwrite-arm-side? overwrite-arm-side?
             :stand-coords stand-coords :overwrite-stand-coords? overwrite-stand-coords?
             :grasp-center? grasp-center? :draw-environment? test-mode?
             )))
         (valve-motion-length (length valve-motion-av-list)))
    (when (not (= valve-motion-length 0))
      ;; reset-manip-pose
      (hand-stop-grasp :real? real?)
      (send *robot* :reset-manip-pose)
      (unless (check-angle-vector-distance)
        (when real? (model2real :time 3000)))
      (when real?
        (set-default-impedance-param)
        (cond ((or (equal *robot-name* "JAXON") (equal *robot-name* "STARO"))
               (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 300 :d-p 500))
              (t ;; (or (equal *robot-name* "HRP2JSKNT") (equal *robot-name* "HRP2JSKNTS"))
               (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 800 :d-p 400)
               )))
      ;; walk
      (when *valve-stand-coords*
        (let* ((pos (scale 0.001 (send *valve-stand-coords* :worldpos)))
               (rpy (mapcar #'rad2deg (car (send *valve-stand-coords* :rpy-angle))))
               )
          (when (check-stand-coords *valve-stand-coords*)
            (ros::ros-warn "go-pos to ~a ~a ~a for valve~%" (elt pos 0) (elt pos 1) (elt rpy 0))
            (when real? (send *ri* :go-pos (elt pos 0) (elt pos 1) (elt rpy 0)))
            )
          ))
      ;; pre grasp
      (send *robot* :angle-vector (car (last valve-motion-av-list)))
      (when real? (model2real :time 5000))
      (dotimes (i rotate-num)
        ;; grasp
        (send *robot* :angle-vector (car valve-motion-av-list))
        (when real? (model2real :time 3000))
        (hand-start-grasp :real? real?)
        ;; rotate
        (when real? (send-angle-vector-sequence-with-time (cdr (subseq valve-motion-av-list 0 (/ valve-motion-length 2))) :time 1000 :wait? t))
        ;; relese
        (hand-stop-grasp :real? real?)
        (send *robot* :angle-vector (car (subseq valve-motion-av-list (/ valve-motion-length 2))))
        (when real? (model2real :time 3000))
        (cond ((= i (1- rotate-num))
               ;; last releasing
               (send *robot* :reset-manip-pose)
               (when real? (model2real :time 5000)))
              (t
               ;; reverse
               (when real? (send-angle-vector-sequence-with-time (cdr (subseq valve-motion-av-list (/ valve-motion-length 2))) :time 1000 :wait? t))
               ))
        )))
  )

(defun publish-valve-motion
  (radius
   coords
   &key (arm :rarm) (overwrite-arm-side? nil)
   (stand-coords nil) (overwrite-stand-coords? nil)
   (grasp-center? nil) (overwrite-grasp-mode? nil)
   (step-time 200) (update? t) (test-mode? nil))
  (when update?
    (get-valve-motion-full
     radius coords
     :valve-coords-system :body :arm arm
     :stand-coords stand-coords :grasp-center? grasp-center?
     :overwrite-stand-coords? overwrite-stand-coords? :overwrite-arm-side? overwrite-arm-side?
     :use-ri? (if test-mode? t nil)))
  (ros::ros-info "valve-stand-coords: ~a~%" *valve-stand-coords*)
  (ros::ros-info "valve-ik-results: ~a~%" *valve-ik-results*)
  (when *valve-stand-coords*
    (move-robot-model *valve-stand-coords* "ground"))
  (publish-motion-joint-states *valve-motion* :step-time step-time)
  )
