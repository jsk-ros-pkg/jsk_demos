#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(load "utils.l")
(load "add-co.l")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*

;; show in irtviewer
(objects (list *fetch* *scene*))
;; show in "fetch Kinematics Simulator"
(send *ri* :objects (send *scene* :objects))


;; check 20201026-soft3-model, the code to focus on the robot model
(send *irtviewer* :look-all
      (send (geo::make-bounding-box
             (flatten (send-all (send *fetch* :bodies) :vertices)))
            :grow 2.0))

;; reset-pose
(reset-pose)

;; move to in front of 73A3
(move-to-place "/eng2/7f/room73A3-front")
(send *ri* :wait-interpolation)


; door knob recognition
(setq *door-knob* (send (send (send (send (send (send (send *scene* :object "/eng2/7f/73a3") :object "room73a3-door-right") 
  :handle-rear-knob) :copy-worldcoords) :locate #f(200 0 0)) :rotate pi :z) :rotate -pi/2 :y))
(setq *door-knob-base* 
  (send (send (send (send *door-knob* :copy-worldcoords) :rotate pi/2 :x) :rotate -pi/2 :z) :locate (float-vector -750 0 ( - (elt (send *door-knob* :worldpos) 2 )))))
(move-to-coord *door-knob-base*)
; ;; or 
; (load "move-knob-front.l")


; right of the fetch
(setq *box* (send (send (send (send *fetch* :copy-worldcoords) :locate #f(-200 -500 1000)) :rotate pi/2 :z) :rotate pi/2 :y))
(setq *bento* (send (send *box* :copy-worldcoords) :locate #f(650 0 0)))
(if (boundp '*co*)
  (add-box -200 -540 0))

;; set coords
(setq *door-knob2-right* 
  (send (send (send (send *fetch* :copy-worldcoords) 
    :transform (make-coords :pos (float-vector 500 -200 (+ (elt (send *door-knob* :worldpos) 2) 150)))) :rotate pi/2 :z) :rotate -pi/2 :x))
(setq *door-knob3-right* (send (send 
    (send *door-knob2-right* :copy-worldcoords) :locate #f(0 0 -200)) :rotate (/ pi 6) :y))
(setq *door-knob3* (send (send 
    (send *door-knob2-right* :copy-worldcoords) :locate #f(200 0 -200)) :rotate (/ pi 6) :y))
(setq *door-knob4* (send (send (send *door-knob3* :copy-worldcoords) :rotate (/ pi 6) :z) :locate #f(-70 0 0)))
(if (boundp '*co*)
  (add-door (+ (- (elt (send *fetch* :worldpos) 0) (elt (send *door-knob3* :worldpos) 0)) 50)))


; grasp bento
(look-at-coord *bento* 3000 t)
(move-waist 100.0)
(ik-coord *box* 5000)
(ik-coord *bento* 3000)
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(ik-coord *box* 2000)

; hang on the knob
(look-at-coord *door-knob3* 2000)
(ik-coord *door-knob2-right* 4000)
(ik-coord *door-knob3-right* 2000)
(unix:sleep 1)
(ik-coord *door-knob3* 3000)
(send *ri* :stop-grasp :effort 60)
(unix:sleep 2)
(ik-coord *door-knob4* 2000)
(ik-coord *door-knob2-right* 2000)

(move-waist 100.0)
(reset-pose 7000)
(if (boundp '*co*)
  (progn (del-door)
      (del-box)))