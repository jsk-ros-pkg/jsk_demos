(require "create-robot-instance.l")
(load "math.l")
(require "package://drc_task_common/euslisp/test-drc-door-task.l")
(require "~/prog/euslib/jsk/gnuplotlib.l")
(require "~/prog/euslib/rbrain/virtual_interpolator.l")
(require "models/arrow-object.l")


;; [How To Demo]
;; With JAXON_RED
;; (init :robot "jaxon_red" :robot-pos #f(-800 -800 0))
;; (open1 :real? t) ;;reach-door
;; (open2 :real? t) ;;unlock the door & open-door
;;
;; With HRP2
;; (init :robot "hrp2jsknt" :robot-pos #f(-700 -900 0))
;; (open1 :real? t) ;;reach-door
;; (open2 :real? t) ;;unlock the door & open-door


(defun make-my-door ()
  (let (robot-name)
  (setq robot-name (send *robot* :name))
  (cond ((equal robot-name "JAXON_RED")
         (setq *door* (instance param-door :init 875 50 110 20
                                (float-vector 0 -820 980)
                                (float-vector -50 -760 980)
                                :use-leg nil
                                :handle-l/r :right))
         )
        ((equal robot-name "HRP2JSKNT")
         (setq *door* (instance param-door :init 875 50 110 20
                                (float-vector 0 -820 980)
                                (float-vector -140 -790 980)
                                :use-leg nil
                                :handle-l/r :right))))))

(defun make-drc-box2-door (&key (use-leg nil))
  "Door on drc box2."
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -160 -580 970) 
                         :use-leg use-leg 
                         :handle-l/r :right)))

(defun init
  (&key (real? nil) 
        (robot-pos #f(-800 -800 0))
        (robot-dir 0)
        (robot "hrp2jsknt")
        (draw? t)
        )
  (setq *robot-pos* robot-pos)
  (create-robot-instance :real? real? :robot-pos robot-pos :robot-dir robot-dir 
                         :robot robot)
  (send *robot* :reset-pose)
  (send *robot* :head :neck-p :joint-angle 0)
  (make-drc-box2-door)
  (when draw? (objects (list *robot* *door*)))
  (when real? 
    (send *ri* :start-auto-balancer)
    (send *ri* :start-st)
    (impedance)
    )
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
      (send (send *door* :joint :door-hinge-joint) :joint-angle door-angle)
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
                  :stop 50
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

  (send *robot* :rarm-shoulder-p :joint-angle -10)
  ;; (send *robot* :rarm-shoulder-p :joint-angle 50)
  ;; (send *robot* :rarm-shoulder-r :joint-angle -30)
  ;; (send *robot* :rarm-shoulder-y :joint-angle -10)
  ;; (send *robot* :rarm-wrist-r :joint-angle -5)
  ;; (send *robot* :rarm-wrist-p :joint-angle -20)
  ;; (send *robot* :rarm-wrist-y :joint-angle -25)
  ;; (send *robot* :rarm-elbow-p :joint-angle -120)
  (send *robot* :HEAD_JOINT1 :joint-angle 0)
  (send *robot* :head :neck-p :joint-angle 0)
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
  (door-reach :real? nil :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(-30 0 200))
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
  (when real? (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (send *ri* :wait-interpolation))
  ;turn down knob
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 -30) :knob-angle -30)
  (send *irtviewer* :draw-objects)
  (when real? (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (send *ri* :wait-interpolation))
  ;open the door a little
  (door-reach :real? real? :knob-offset #f(0 0 0) :lhand-pos-to-knob #f(0 0 0) :knob-angle -30 :door-angle -5
              :rot-knob-z (* 0.68 5)
              )
  (send *irtviewer* :draw-objects)
  (when real? (send *ri* :angle-vector (send *robot* :angle-vector) time)
        (send *ri* :wait-interpolation)
        )
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
  (&key (real? nil) (time 30000) (slow t) (ik-map nil))
  (setq *open-door-avs* (mapcar #'(lambda (door-angle)
                        (door-reach2 :real? nil :knob-offset #f(0 0 0) 
                                    :door-angle door-angle :target-only-pos nil
                                    ))
                    (mapcar #'(lambda (x) (elt x 0))
                            (cadr (memq :data (pos-list-interpolation
                                               (list (float-vector -5) (float-vector -60))
                                               (list 2.5)
                                               0.5))))
                    ))
  (setq *open-door-tvs* (cons 0 (count-up-step (/ time (length *open-door-avs*)) (/ time (length *open-door-avs*)) (1- (length *open-door-avs*)))))
  (when real? t
        (send *ri* :angle-vector-sequence *open-door-avs* *open-door-tvs*)))


(defun open1 (&key (time 10000) (real? nil) (adjust? t))
  ;;(send *robot* :reset-manip-pose)
  (send *robot* :larm :angle-vector #f(50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (when real?
    (when adjust? (adjust-door))
    (print "OK?")
    (if (equal (read-line) "y")
        (send *ri* :angle-vector (send *robot* :angle-vector) time))
    (send *ri* :wait-interpolation))
  (door-reach0)
  (when real? 
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    (send *ri* :wait-interpolation))
  (door-reach)
  (when real? 
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    (send *ri* :wait-interpolation))
  )

(defun open2 (&key(time 10000) (real? nil))
  (unlock-door :real? real? :time 10000)
  (print "Do you want to opne the door?")
  (read-line)
  (open-door :real? real? :time time)
  )

(defun impedance (&optional (option nil))
  (cond
   ((equal option "soft")
    (print "set impedance params soft mode")
    (send *ri* :start-impedance :larm :k-p 1000 :d-p 500 :m-p 7.5 :m-r 5 :d-r 50 :k-r 50 
          :force-gain (float-vector 0.01 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))
   ((equal  option "free")
    (print "set impedance params to free mode")
    (send *ri* :start-impedance :larm :k-p 1000 :d-p 500 :m-p 7.5 :m-r 5 :d-r 50 :k-r 50 
          :force-gain (float-vector 0.01 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))
   (t
    (print "set impedance params defaul mode")
    (send *ri* :start-impedance :larm :k-p 2000 :d-p 1000 :m-p 20 :m-r 5 :d-r 50 :k-r 100 
          :force-gain (float-vector 1 1 1) :moment-gain (float-vector 1.0 1.0 1.0)))))

(defun collision-check-for-door ()
  (let (robot-name hand-collision-flag)
    (setq robot-name (send *robot* :name))
    ;;for hrp3 hand
    (cond ((equal robot-name "HRP2JSKNT")
           (setq hand-collision-flag (pqp-collision-check (elt (send *robot* :hand :larm :links) 0) 
                                                          (send *door* :link :door-panel))))
          ((equal robot-name "JAXON_RED")
           (setq hand-collision-flag 0)))
    ;;for thk_hand
    (if (> (reduce #'+
                   (cons
                    hand-collision-flag
                    (mapcar #'(lambda (link) (pqp-collision-check link (send *door* :link :door-panel))) (send *robot* :links))
                    )
                   )
           0) t nil)))

(load "subscribe_checkerboard.l")
(defun diff-pos-rot (coords1 coords2)
  (setq pos2 (send coords2 :worldpos))
  (setq unit2 (send coords2 :rotate-vector #f(1 0 0)))
  
  (setq pos1 (send coords1 :worldpos))
  (setq unit1 (send coords1 :rotate-vector #f(1 0 0)))
  
  (setq pos-diff (v- pos2 pos1))
  (setq rot-diff (v* unit1 unit2))
  (setf (elt rot-diff 0) (asin (elt rot-diff 0)))
  (setf (elt rot-diff 1) (asin (elt rot-diff 1)))
  (setf (elt rot-diff 2) (asin (elt rot-diff 2)))  
  
  (list pos-diff rot-diff)
  )

(defun adjust-door ()
  (let (door-real-knob-coords door-model-knob-coords diff door-knob-pos-dif rot-z)
    (dolist (adjust-door-counter (list 1 2 3))
      (setq door-real-knob-coords (get-target-coords))
      (print "real")
      (print door-real-knob-coords)
      
      (setq door-model-knob-coords (send (send *door* :handle :knob-handle) :worldcoords))
      (print "model")
      (print door-model-knob-coords)
      
      
      (setq diff (diff-pos-rot door-real-knob-coords door-model-knob-coords))
      
      (setq door-knob-pos-diff (elt diff 0))
      (setq rot-z (elt (elt diff 1) 2))
      
      (send *robot* :newcoords (send (send *robot* :worldcoords) :translate door-knob-pos-diff :world))
      (send *robot* :newcoords (send (send *robot* :worldcoords) :rotate rot-z :z :world))
      (print "robot")
      (print (send *robot* :worldcoords))
      )
    (send *irtviewer* :draw-objects)
    ))

(defun walk-to-target (&key (target-coords *init-robot-coords*) (real? nil))
  (let (current-coords diff pos-diff rot-z loacl-pos-diff dx dy rot-z-deg)
    (adjust-door)
    (unix::sleep 1)
    (adjust-door)
    (setq current-coords (send (send *robot* :worldcoords) :copy-worldcoords))
    (setq diff (diff-pos-rot current-coords target-coords))
    
    (setq pos-diff (elt diff 0))
    (setq rot-z (elt (elt diff 1) 2))
  
    (setq local-pos-diff (send (send (send *robot* :worldcoords) :inverse-transformation) :rotate-vector pos-diff))
    
    (send (send (send (send (send *robot* :foot-midcoords) :copy-worldcoords) 
                      :translate local-pos-diff) :rotate rot-z :z) :draw-on :flush t)
    
    (setq dx (* 0.001 (elt local-pos-diff 0)))
    (setq dy (* 0.001 (elt local-pos-diff 1)))
    (setq rot-z-deg (rad2deg rot-z))
  
    (when real? 
      (print "walk OK?")
      (if (equal (read-line) "y")
          (send *ri* :go-pos dx dy rot-z-deg)))
    ))
