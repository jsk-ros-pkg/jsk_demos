(require "create-robot-instance.l")
(load "math.l")
(require "package://drc_task_common/euslisp/test-drc-door-task.l")
(require "~/prog/euslib/jsk/gnuplotlib.l")
(require "~/prog/euslib/rbrain/virtual_interpolator.l")
(require "models/arrow-object.l")
;; (mapcar #'(lambda (x) (elt x 0))
;;         (cadr (memq :data (pos-list-interpolation
;;                            (list (float-vector 0) (float-vector 70))
;;                            (list 2.0)
;;                            0.5))))

;; drc box2 door param
;;   door width : 700mm
;;   hinge -> knob-axis : 640mm
;;   knob depth : 60mm
;;   knob width : 100mm
;;   knob height : 20mm
;;   knob pos z : 950mm
;;(defun make-my-door ()
;;  "Door on drc box2."
;;  (setq *door* (instance param-door :init 700 60 100 20
;;                         (float-vector 0 -640 950)
;;                         (float-vector -60 -600 950)
;;                         :handle-l/r :right)))

(defun make-my-door ()
  (setq *door* (instance param-door :init 875 50 110 20
                         (float-vector 0 -820 980)
                         (float-vector -140 -790 980)
                         :use-leg nil
                         :handle-l/r :right)))
;;init :robot-dir 30 :robot-pos #f(-700 -900 0)
;;init :robot-dir 30 :robot-pos #f(-750 -850 0);;open-legs-moti 100
;;init :robot-dir 30 :robot-pos #f(-850 -850 0);
(defun init
  (&key (real? nil) 
        ;;(robot-pos #f(-700 -900 0))
        (robot-pos #f(-900 -800 0));;open-legs 100
        ;;(robot-pos #f(-800 -900 0));;standard
        (robot-dir 0);;30 is good
        (robot "hrp2jsknt")
        (draw? t)
        )
                                        ;if use real machine
  (setq *robot-pos* robot-pos)
  (create-robot-instance :real? real? :robot-pos robot-pos :robot-dir robot-dir 
                         :robot robot)
  (make-my-door)
  (when draw? (objects (list *robot* *door*)))
  (setq *init-robot-coords* (send (send *robot* :worldcoords) :copy-worldcoords))
  (send *robot* :chest_joint1 :max-angle 30)
  (send *robot* :chest_joint1 :min-angle -2)
  )

(defun rot-target (my-coords theta axis
                              &key
                              (translate #f(120 10 -10)) (show nil))
  (setq end-coords (send my-coords :copy-worldcoords))
  (setq center-coords (send (send end-coords :copy-worldcoords):translate translate))
  (when show
    (send center-coords :draw-on :flush t)
    (send end-coords :draw-on :flush t)
    )
  (send end-coords :transform (send center-coords :inverse-transformation) :world)
  (send center-coords :rotate (deg2rad theta) axis :world)
  (send end-coords :transform center-coords :world)
  (when show
    (send center-coords :draw-on :flush t)
    (send end-coords :draw-on :flush t)
    )
  end-coords
  )

(defun draw-knob-target-coords (&key(door-angle 0) (knob-angle 0) (rot-knob-z 0) (knob-offset #f(0 0 0)) (lhand-pos-to-knob #f(0 0 0)))
  (let (knob-target-coords temp-door-angle temp-knob-angle)
    (setq temp-door-angle (send (send *door* :joint :door-hinge-joint) :joint-angle))
    (setq temp-knob-angle (send (send *door* :joint :door-knob-joint) :joint-angle))
    (send (send *door* :joint :door-hinge-joint) :joint-angle door-angle)
    (send (send *door* :joint :door-knob-joint) :joint-angle knob-angle)
    (setq knob-target-coords
          (rot-target
           (send (send (send (send *door* :handle :knob-handle) :copy-worldcoords)
                       :translate knob-offset)
                 :translate lhand-pos-to-knob :world)
           rot-knob-z :z)
          )
    (setq *arrow* (arrow))
    (send *arrow* :newcoords knob-target-coords)
    (objects (cons *arrow* (objects)))
    (send (send *door* :joint :door-hinge-joint) :joint-angle temp-door-angle)
    (send (send *door* :joint :door-knob-joint) :joint-angle temp-knob-angle)
    (send *irtviewer* :draw-objects)
    ))




(defun door-reach
  (&key (real? nil) (time 3000) (door-angle 0) (knob-angle 0)
        (knob-offset #f(0 0 0)) (rot-knob-z 0)
        (lhand-pos-to-knob #f(0 0 0)) (target-only-pos nil)
        (rleg-translate #f(0 0 0)) (lleg-translate #f(0 0 0))
        (reset? t) (rthre (list 1 1 1)) (draw? t)
        (target-centroid-pos
         (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :pos)
         )
        (knob-target-coords nil)
        )
  (let (init-robot-coords lleg-end-coords rleg-end-coords target-coords ik-ret)
  (block door-reach
    (setq init-robot-coords (send (send *robot* :worldcoords) :copy-worldcoords))
    (setq lleg-end-coords (send (send *robot* :lleg :end-coords) :copy-worldcoords))
    (setq rleg-end-coords (send (send *robot* :rleg :end-coords) :copy-worldcoords))
    (when reset?
      (reset-manip-pose-only-rarm)
      (send (send *door* :joint :door-hinge-joint) :joint-angle door-angle);door-angle
      (send (send *door* :joint :door-knob-joint) :joint-angle knob-angle)
      )
    (when (eq knob-target-coords nil)
      (setq knob-target-coords
            (rot-target
             (send (send (send (send *door* :handle :knob-handle) :copy-worldcoords)
                         :translate knob-offset)
                   :translate lhand-pos-to-knob :world)
             rot-knob-z :z)
            ))
    (let* ((move-target
            (list (send *robot* :lleg :end-coords)
                  (send *robot* :rleg :end-coords)
                  (send *robot* :larm :end-coords)
                  ))
           (target-coords)
           (offset-from-knob-coords)
           )
      (setq target-coords
            (list
             lleg-end-coords
             rleg-end-coords
             knob-target-coords
             ))
      (setq ik-ret
            (send *robot*
                  :fullbody-inverse-kinematics target-coords
                  :move-target move-target
                  :link-list (mapcar #'(lambda (limb)
                                         (send *robot* :link-list (send limb :parent))) move-target)
                  :translation-axis (list t t t)
                  :rotation-axis (list t t (not target-only-pos))
                  :rthre rthre
                  :target-centroid-pos target-centroid-pos
                  :warnp nil
                  :stop 20
                  ))
      (when (not ik-ret) (return-from door-reach nil))
      (cond ;; ((collision-check-for-door) 
            ;;  (warn "collision!!!")
            ;;  nil)
            (t
             (when draw? (send *irtviewer* :draw-objects))
             (when real?
               (send *ri* :angle-vector (send *robot* :angle-vector) time))
             (send *robot* :angle-vector)
             )
            )
      ))))
(defun reset-manip-pose-only-rarm ()
  (setq current-robot-pos (send (send *robot* :foot-midcoords) :copy-worldcoords))
  (setq lleg-end-coords (send (send *robot* :lleg :end-coords) :copy-worldcoords))
  (setq rleg-end-coords (send (send *robot* :rleg :end-coords) :copy-worldcoords))

  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords current-robot-pos)
  (setq rleg-pos (v- (send rleg-end-coords :worldpos) (send (send *robot* :rleg :end-coords) :worldpos)))
  (setq lleg-pos (v- (send lleg-end-coords :worldpos) (send (send *robot* :lleg :end-coords) :worldpos)))
  (send *robot* :rleg :move-end-pos rleg-pos)
  (send *robot* :lleg :move-end-pos lleg-pos)

  (send *robot* :rarm-shoulder-p :joint-angle 50)
  (send *robot* :rarm-shoulder-r :joint-angle -30)
  (send *robot* :rarm-shoulder-y :joint-angle -10)
  (send *robot* :HEAD_JOINT1 :joint-angle 0)
  (send *robot* :rarm-wrist-r :joint-angle -5)
  (send *robot* :rarm-wrist-p :joint-angle -20)
  (send *robot* :rarm-wrist-y :joint-angle -25)
  (send *robot* :rarm-elbow-p :joint-angle -120)
  )
(defun door-reach2
  (&key (real? nil) (time 3000) (door-angle 0) (knob-angle 0)
        (knob-offset #f(0 0 0))
        (lhand-pos-to-knob #f(0 0 0)) (target-only-pos nil)
        (rleg-translate #f(0 0 0)) (lleg-translate #f(0 0 0))
        (reset? t) (rthre (list 1 1 1)) (draw? t)
        (target-centroid-pos
         (send (send (send *robot* :foot-midcoords) :copy-worldcoords) :pos))
        )
    (setq k 0)
    (setq flag t)
    (while flag
      (setq rot-knob-z (* k (if (< door-angle 0) 5 -5)))
      (setq ret
            (door-reach :real? nil :door-angle door-angle
                        :rot-knob-z rot-knob-z :draw? nil
                        :lleg-translate lleg-translate
                        :rleg-translate rleg-translate
                        :target-centroid-pos target-centroid-pos
                        )
            )
      (when (not (equal rleg-translate #f(0 0 0))) (setq rleg-translate #f(0 0 0)))
      (when (not (equal lleg-translate #f(0 0 0))) (setq lleg-translate #f(0 0 0)))
      (setq k (1+ k))
      (if ret (setq flag nil))
      (if (> (abs rot-knob-z) (abs door-angle)) (setq flag nil))
      )
    (if draw? (send *irtviewer* :draw-objects))
    (format t "rot-knob-z is ~A~%" rot-knob-z)
    ret
    )
(defun door-reach0 (&key (real? nil))
  (door-reach :real? nil :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(-100 0 100))
  (send *irtviewer* :draw-objects)
  (when real? (send *ri* :angle-vector (send *robot* :angle-vector)))
  )

(defun hand0 ()
  (send (send *robot* :hand :larm) :reset-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 1000)
  )

(defun hand1 ()
  (send *robot* :hand :larm :l_indexpip_r :joint-angle 110)
  (send *robot* :hand :larm :l_indexpip_r :joint-angle 110)
  (send *robot* :hand :larm :l_middlepip_r :joint-angle 30)
  (send *irtviewer* :draw-objects)
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send *robot* :hand :arms :angle-vector)) 1000)
 )

(defun unlock-door (&key (real? nil) (time 10000))
  ;start
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 0) :knob-angle 0)
  (send *irtviewer* :draw-objects)
  (when real? (model2real :time time :sleep t))
  ;turn down knob
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 -30) :knob-angle -30)
  (send *irtviewer* :draw-objects)
  (when real? (model2real :time time :sleep t))
  ;open the door a little
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 0) :knob-angle -30 :door-angle -5
              :rot-knob-z (* 0.68 5)
              )
  (send *irtviewer* :draw-objects)
  (when real? (model2real :time time :sleep t))
  ;stop turn down the knob, keeping the door open
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 0) :knob-angle 0 :door-angle -5
              :rot-knob-z (* 0.68 5))
  (send *irtviewer* :draw-objects)
  )

(defun unreach-door
  (&key (real? nil) (time 3000) (door-angle 0) (knob-angle 0)
        (knob-offset #f(0 0 0)) (rot-knob-z 0)
        (lhand-pos-to-knob #f(0 0 0)) (target-only-pos nil)
        (rleg-translate #f(0 0 0)) (lleg-translate #f(0 0 0))
        (reset? nil)
        )
  (door-reach :lhand-pos-to-knob #f(0 0 50) 
              :door-angle (send (send *door* :joint :door-hinge-joint) :joint-angle)
              :target-only-pos t
              :rot-knob-z (* 0.68 (- (send (send *door* :joint :door-hinge-joint) :joint-angle)))
              :reset? nil)
  (send *robot* :chest_joint1 :joint-angle (+ (send *robot* :chest_joint1 :joint-angle) -20))
  (send *robot* :larm-wrist-p :joint-angle -40)
  (send *irtviewer* :draw-objects)
  (send *robot* :angle-vector)
  )

(defun open-door
  (&key (real? nil) (time 10000) (slow t) (ik-map nil))
  (setq *open-door-avs* (mapcar #'(lambda (door-angle)
                        (door-reach2 :real? nil :knob-offset #f(0 0 0) 
                                    :door-angle door-angle :target-only-pos nil
                                    ;;:rot-knob-z (* 0.68 (- (send (send *door* :joint :door-hinge-joint) :joint-angle)))
                                    ))
                    ;; (mapcar #'(lambda (x) (elt x 0))
                    ;;         (cadr (memq :data (pos-list-interpolation
                    ;;                            (list (float-vector -5) (float-vector -60))
                    ;;                            (list 2.5)
                    ;;                            0.5))))
                    (list -5.0 -7.1856 -12 -35 -60 -70)
                    ))
  (setq *open-door-tvs* (cons 0 (count-up-step (/ time (length *open-door-avs*)) (/ time (length *open-door-avs*)) (1- (length *open-door-avs*)))))
  (when real? t
        (cond (slow
               (dolist (avs_i avs)
                 (print "model2real?")
                 (read-line)
                 (model2real :angle-vector avs_i :time (floor (/ time (length avs))))
                 ))
              (t
               (send *ri* :angle-vector-sequence avs tvs)))
  ))




(defun open1 (&key (time 1000))
  ;;(send *robot* :reset-manip-pose)
  (model2real :time (* 3 time) :sleep t)
  (door-reach0)
  (model2real :time (* 3 time) :sleep t)
  (handle-reach)
  (model2real :time (* 3 time) :sleep t)
  )

(defun prepare-open2(&key (time 3000))
  (open-door :real? nil :time time)
  (dump-loadable-structure "./open2-avs.l" *open-door-avs* *open-door-tvs*)
  )

(defun open2 (&key(time 10000) (slow nil) (wait t) (log? nil))
  (unlock-door :real? t :time 2000)
  (print "model2real?")
  (load "./open2-avs.l")
  (read-line)
  (cond (slow
         (dolist (avs_i avs)
           (print "model2real?")
           (read-line)
           (model2real :angle-vector avs_i :time (floor (/ time (length avs))))
           ))
        (t
         (send *ri* :angle-vector-sequence *open-door-avs* *open-door-tvs*)))
  (model2real :sleep t :send? nil)
  (when log? (send *ri* :save-log "/tmp/open-door"))
  )

(defun lhand-down ()
  (send *robot* :larm :move-end-pos #f(0 0 -100) :world)  
  )

(defun lhand-up ()
  (send *robot* :larm :move-end-pos #f(0 0 100) :world)  
   (send *irtviewer* :draw-objects)
   )

(defun impedance (&key (gazebo? nil) (soft nil))
  (cond 
   (soft (send *ri* :start-impedance :larm :k-p 1000 :d-p 500 :m-p 7.5 :m-r 5 :d-r 50 :k-r 50 
               :force-gain (float-vector 0.01 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))
  (t
   (send *ri* :start-impedance :larm :k-p 1000 :d-p 500 :m-p 7.5 :m-r 5 :d-r 50 :k-r 50 
         :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))))



;; open the legs agains auto-balancer
;(defun open-legs (&key (robot *robot*) (real? t))
 ; (let* (footstep-list open-legs-motion)
;;    (objects (append (list robot) footstep-list))
;    (setq footstep-list 
;;  (list (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
;;(make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) 
;;                           :translate #f(-100 0 0) :world) :name :lleg)
;;))
;    (send *irtviewer* :draw-objects)
;    
;    (defun calc-open-legs (robot footstep-list)
;      (send robot :calc-walk-pattern-from-footstep-list footstep-list 
;	    :debug-view :no-message
;	    :default-step-height 50)
;      )
    
;    (defun get-angle-vector (motion-pattern) 
;      (cond ((eq (cdr motion-pattern) nil) 
;	     (print (cadar motion-pattern))
;	     (send *ri* :angle-vector (cadar motion-pattern) 100)
;	     (send *ri* :wait-interpolation)
;	     )
;	    (t (print (cadar motion-pattern))
;	       (send *ri* :angle-vector (cadar motion-pattern) 100)
;	       (send *ri* :wait-interpolation)
;	       (get-angle-vector (cdr motion-pattern))
;	       )
;	));
    
   ; (cond (real? (setq open-legs-motion (calc-open-legs robot footstep-list))
;		 (get-angle-vector open-legs-motion))
;	  (t (calc-open-legs robot footstep-list)))      
;    (print 'done)
;    ))


(defun open-legs (&key (robot *robot*) (real? nil) (position #f(100 0 0)))
  (let* (footstep-list)
    ;; (objects (append (list robot) footstep-list))
    (setq footstep-list 
          (list (make-coords :coords (send robot :lleg :end-coords :copy-worldcoords) :name :lleg)
                (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords)
                                           :translate position) :name :rleg)
                ))
;;    (send *robot* :set-foot-steps
    (send robot :calc-walk-pattern-from-footstep-list footstep-list
          :debug-view :no-message
          :default-step-height 50)
    (when real?
      (send *ri* :set-gait-generator-param :default-step-height 0.05)
      (send *ri* :wait-foot-steps)
      (send *ri* :set-foot-steps footstep-list)
      (send *ri* :wait-foot-steps))
    (print "  setFootSteps()=>OK")
    t))

(defun move-cog-foward (&key (robot *robot*) (real? t))
  (let* (footstep-list)
    ;; (objects (append (list robot) footstep-list))
    (setq footstep-list 
          (list (make-coords :coords (send robot :lleg :end-coords :copy-worldcoords) :name :lleg)
                (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
                ))
;;    (send *robot* :set-foot-steps
    (send robot :calc-walk-pattern-from-footstep-list footstep-list
          :debug-view :no-message
          :default-step-height 0)
    (send *ri* :wait-foot-steps)
    (send *ri* :set-gait-generator-param :default-step-height 0)
    (send *ri* :set-foot-steps footstep-list)
    (send *ri* :wait-foot-steps)
    (print "  setFootSteps()=>OK")
    t))

(defun close-legs (&key (robot *robot*) (real? t))
  (let* ()
    (setq footstep-list
          (list (make-coords :coords (send robot :lleg :end-coords :copy-worldcoords) :name :lleg)
                (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) 
                                           :translate #f(0 -210 0)) :name :rleg)
                )
          )
    (send *robot* :calc-walk-pattern-from-footstep-list footstep-list
          :debug-view :nomessage
          :default-step-height 50)
    (send *ri* :set-gait-generator-param :default-step-height 0.05)
    (send *ri* :wait-foot-steps)
    (send *ri* :set-foot-steps footstep-list)
    (send *ri* :wait-foot-steps)
    (print "  setFootSteps()=>OK")
    t))




  (defun walk-motion
  (robot)
  "walking motion using preview-control, gait-generator, and fullbody ik"
  (let* ((is-small-robot
          (< (abs (elt (send (send (car (send robot :links));;get root link 
                                   :transformation ;;get root link position in foot-link coords
                                   (apply #'midcoords 0.5 (send robot :legs :end-coords))) 
                             :worldpos) 2)) 400)) ;;compare the height of root-link with 400mm
         (default-step-height (if is-small-robot 10 50)))
                                        ;determin the step hight according to the robot size
  (send robot :reset-pose)
  (send robot :fix-leg-to-coords (make-coords))
  (objects (list robot))
  (warn ";; test1 ;; specify footstep-list~%")
  (let* ((test1-scale (if is-small-robot 0.25 1.0))
        (footstep-list
         (list (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)
                                        ;make copy of right foot coords named :rleg 
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) 
                                          :translate (scale test1-scale #f(100 0 0))) :name :lleg)
                                        ;make target coords named lleg
               (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) 
                                          :translate (scale test1-scale #f(200 0 0))) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(300 0 0))) :name :lleg)
               (make-coords :coords (send (send robot :rleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(400 0 0))) :name :rleg)
               (make-coords :coords (send (send robot :lleg :end-coords :copy-worldcoords) :translate (scale test1-scale #f(500 0 0))) :name :lleg))))
    (objects (append (list robot) footstep-list));show robot and coords
    ;; footstep-list is coordinates list.
    ;; This should have :l/r method or :l/r name.
    (send robot :calc-walk-pattern-from-footstep-list
          footstep-list :debug-view :no-message
          :default-step-height default-step-height)
    )
  (warn ";; test2 ;; calc footstep-list from go pos param~%")
  (objects (list robot
                 (apply #'midcoords 0.5 (send robot :legs :end-coords))
                 (send (send (apply #'midcoords 0.5 (send robot :legs :end-coords)) :translate (float-vector 500 150 0)) :rotate (deg2rad 45) :z)))
  (if is-small-robot (send robot :gen-footstep-parameter :ratio 0.3))
  (send robot :calc-walk-pattern-from-footstep-list
        (send robot :go-pos-params->footstep-list
              500 150 45) ;; x[mm] y[mm] th[deg]
        :debug-view :no-message
        :default-step-height default-step-height)
  ))







(defun collision-check-for-door ()
  (if (> (reduce #'+
                 (cons
                  (pqp-collision-check (elt (send *robot* :hand :larm :links) 0) (send *door* :link :door-panel))
                  (mapcar #'(lambda (link) (pqp-collision-check link (send *door* :link :door-panel))) (send *robot* :links))
                  )
                 )
         0) t nil)
