(load "door-open.l")
(init :real? t)
(warn "please init-ri when you start first. autobalancer, st, impedance will be start")


(sys::make-thread 10)
;;prepare for multi thread

(defun init-ri (&key (st nil))
  (model2real :time 5000)
  (print "start-auto-balancer")
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :larm))
  (send *ri* :wait-interpolation)
  ;;  (send *ri* :calibrate-inertia-sensor)
  ;;  (print "calibrate-inertia-sensor")
  ;;  (send *ri* :wait-interpolation)
  ;;  (print "wait-interpolation")
  (cond (st (print "start-st")
            (send *ri* :start-st)
            (send *ri* :wait-interpolation)))
  (print "impedance")
  (send *ri* :start-impedance :arms :k-p 800 :d-p 300 :m-p 7.5 :m-r 0.5 :d-r 8 :k-r 20 
        :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0))
  )


(defun demo(&key (time1 1000) (time2 2000))
  (init-ri)
  ;;  (setq *door-angle-thre* -15)
  (model2real :time (* 3 time1))
  (door-reach0)
  (warn "prepare reaching door-knob. if ok, please enter.")
  (cond (t;;(equal (read-line) "") 
         (model2real :time (* 3 time1))
         (handle-reach)
         (warn "reach door-knob. if ok, please enter.")
         (cond (t ;;(equal (read-line) "")
                (calibrate-door-angle)
                (model2real :time (* 2 time1))
                (warn "unlock door. if ok, please enter.")
                (cond (t ;;(equal (read-line) "")
                       (unlock-door :real? t :time 2000)
                       (warn "open door. if ok, please enter.")
                       (cond (t ;;(equal (read-line) "")
                              ;; (door-reach :real? t :knob-offset #f(0 0 0) :door-angle (* 1.5 *door-angle-thre* )
                              ;;             :rot-knob-z (* 0.68 (- *door-angle-thre*)))
                              (open-door-with-constant-velocity-prepare :start-angle -5)
                              (change-params)
                              (open-door-with-constant-velocity-execute :time time2)
                              )))))))))

(defun open-door-with-constant-velocity-prepare
  (&key (start-angle -8) (stop-angle -60))
  (let ((step 3))
    (setq *avs* (mapcar #'(lambda (door-angle)
                          (door-reach :real? nil :knob-offset #f(0 0 0) 
                                      :door-angle door-angle :target-only-pos nil :rot-knob-z (* 0.68 (- door-angle))))
                      (count-up-step (+ start-angle (/ (- stop-angle start-angle) (1- step)))
                                     (/ (- stop-angle start-angle) (1- step))
                                     (1- step))
                      ))
    ))

(defun open-door-with-constant-velocity-execute (&key(time 10000))
  (setq *tvs* (count-up-step (/ time (length *avs*)) (/ time (length *avs*)) (length *avs*)))
  (send *ri* :angle-vector-sequence *avs* *tvs*)
  )

(defun open-door-with-constant-velocity
  (&key (real? nil) (time 10000) (start-angle -8) (stop-angle -60))
  (open-door-with-constant-velocity-prepare :time 10000 :start-angle -8 :stop-angle -60)
  (when real?
    (open-door-with-constant-velocity-exec))
  )

(defun estimate-robot (&key (draw? nil))
  (let (av)
    "update *robot_estm* acoording to potentio-vector of *ri*."
    (send *robot_estm* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot_estm* :fix-leg-to-coords (send *robot* :foot-midcoords))
    
    (send *robot_ref* :angle-vector (send *ri* :state :reference-vector))
    (send *robot_ref* :fix-leg-to-coords (send *robot* :foot-midcoords))
    
    (when draw? (send *irtviewer* :draw-objects))))
(defun start-estimate-robot()
  (let (av)
    "make instance of *robot_estm* and start estimate-robot."
    (create-robot-instance-core *robot_estm* *init-robot-coords_estm* :robot "hrp2jsknt")
    (create-robot-instance-core *robot_ref* *init-robot-coords_ref* :robot "hrp2jsknt")
    (estimate-robot)
    (objects (cons *robot_ref* (cons *robot_estm* (objects))))
    ))
(start-estimate-robot)


(defun calibrate-door-angle ()
  "set current door-angle ZERO[deg]. Substitute current hand coords in *init-hand-coords*."
  (estimate-robot :draw? t)
  (setq *init-hand-coords* (send (send *robot_estm* :larm :end-coords) :copy-worldcoords))
  (format t "set current door-angle ~A deg.~%" (estimate-door-angle))
  )


(defun estimate-door-angle ()
  "calculate door-angle from difference between current hand position and *init-hand-coords* set in calibrate-door-angle function"
  (let* ();;(hand-coords x0 y0 x1 y1 th0 th1)
    (estimate-robot :draw? t)
    (setq hand-coords (send (send *robot_estm* :larm :end-coords) :copy-worldcoords))
    (setq x0 (elt (send *init-hand-coords* :pos) 0)
          y0 (elt (send *init-hand-coords* :pos) 1)
          x1 (elt (send hand-coords :pos) 0)
          y1 (elt (send hand-coords :pos) 1)
          th0 (atan (/ y0 x0))
          th1 (atan (/ y1 x1))
          *door-angle_estm* (rad2deg (cond ((< x1 0) (- (abs (- th1 th0))))
                                           ((=> x1 0) (abs (- th1 th0)))))
          )
    (format t "estimated door-angle is ~A[deg] ~%" *door-angle_estm*)
    )
  *door-angle_estm*
  )

(defun nearly-equal-coords (coords1 coords2 er)
  (let (err)
    (setq err (norm (v- (send (send coords1 :copy-worldcoords) :pos)
                        (send (send coords2 :copy-worldcoords) :pos))))
    (cond ((< err er)
           (print err)
           t)
          (t (print err)  nil)
          )))

(defun judge-if-follow-angle-vector(er)
  (estimate-robot)
  (nearly-equal-coords
   (send *robot_ref* :larm :end-coords)
   (send *robot_estm* :larm :end-coords)
   er)
  )


(defun change-params()
  (let ()
    (estimate-door-angle)

    (while (not (judge-if-follow-angle-vector 50))
      (unix::usleep 100000)
      (estimate-door-angle)
      )
    (estimate-door-angle)
    (send *ri* :start-impedance :arms :k-p 800 :d-p 300 :m-p 7.5 :m-r 0.5 :d-r 8 :k-r 10 
          :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0))
    (send *ri* :set-ref-force (float-vector 0 0 0) 1000 :rarm)
    (setq zmpoff (list #f(50 50 0) #f(50 50 0)))
    (send *ri* :set-auto-balancer-param :default-zmp-offsets zmpoff)
    (print "change-param-of-impedance")
    ))