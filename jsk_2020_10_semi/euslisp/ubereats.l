#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(load "utils.l")

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

;; initialize
(base-position)

;; move to in front of 73A3
(move-to-place "/eng2/7f/room73A3-front")

; door knob recognition
(setq *door-knob* (send (send (send (send (send (send (send *scene* :object "/eng2/7f/73a3") :object "room73a3-door-right")
 :handle-rear-knob) :copy-worldcoords) :locate #f(200 0 0)) :rotate -pi/2 :x) :rotate -pi/2 :z))
; (send *door-knob* :draw-on :flush t :size 1000 :color #f(0 1 0)))
(setq *door-knob-base* (send (send (send *door-knob* :copy-worldcoords) :rotate -pi/2 :z) :locate (float-vector -500 -200 ( - (elt (send *door-knob* :worldpos) 2 )))))
; (send *door-knob-base* :draw-on :flush t :size 1000 :color #f(1 0 0)))
; (objects (list *fetch* *scene* *door-knob* *door-knob-base*))
; (send *ri* :objects (send *scene* :objects))

;; move to in front of door knob
(move-to-coord *door-knob-base*)

;; move arms toward the door knob
(send *fetch* :rarm :inverse-kinematics *door-knob* :debug-view t)
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)

;; back to original position
(base-position)

; ; (progn (setq *door-knob* (send (send (send *scene* :spot "/eng2/7f/room73A3-front") :copy-worldcoords) :locate #f(1000 500 600)))
; ;     (send *door-knob* :draw-on :flush t :size 1000 :color #f(1 0 0)))
; ; (send *fetch* :move-to *door-knob* :world) ;; moved in irtviewer
; ; (send *irtviewer* :draw-objects) ;; update viewer
; ; (send *ri* :move-to *door-knob*) ;; moved real robot

; ;(send *irtviewer* :look1 (send *door-knob-tmp* :worldpos) 0 90) ;change irtview from fetch's above