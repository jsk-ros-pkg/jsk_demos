(load "utils.l")

;; reset-pose
(reset-pose)
(send *ri* :stop-grasp :effort 60)

(format t "reset-pose")

;;
; ;; move arms toward the door knob
; right of the fetch
(setq *box* (send (send (send (send *fetch* :copy-worldcoords) :locate #f(-200 -500 1000)) :rotate pi/2 :z) :rotate pi/2 :y))
(setq *bento* (send (send *box* :copy-worldcoords) :locate #f(650 0 0)))
(if (boundp '*co*)
  (add-box -200 -540 0))

; behind the fetch 
; (setq *box* (send (send (send (send *fetch* :copy-worldcoords) :locate #f(-300 -200 600)) :rotate pi/2 :z) :rotate -pi/2 :x)) ;fail

; ;; in front of the fetch 
; (setq *box* (send (send (send (send *fetch* :copy-worldcoords) :locate #f(300 -200 800)) :rotate pi/2 :z) :rotate -pi/2 :x))
; (setq *bento* (send (send *box* :copy-worldcoords) :locate #f(200 0 0)))
;;

; ;; set coords by absolute value
; (setq *door-knob2-right* 
;   (send (send (send (send *fetch* :copy-worldcoords) 
;     :transform (make-coords :pos #f(500 0 900))) :rotate pi/2 :z) :rotate -pi/2 :x))
; (setq *door-knob2* (send (send *door-knob2-right* :copy-worldcoords) :locate #f(200 0 0)))
;; set coords by *door-knob*
(setq *door-knob* (send (send (send (send (send (send (send *scene* :object "/eng2/7f/73a3") :object "room73a3-door-right") 
  :handle-rear-knob) :copy-worldcoords) :locate #f(200 0 0)) :rotate pi :z) :rotate -pi/2 :y))
(setq *door-knob2-right* 
  (send (send (send (send *fetch* :copy-worldcoords) 
    :transform (make-coords :pos (float-vector 500 -200 (+ (elt (send *door-knob* :worldpos) 2) 150)))) :rotate pi/2 :z) :rotate -pi/2 :x))
(setq *door-knob3-right* (send (send 
    (send *door-knob2-right* :copy-worldcoords) :locate #f(0 0 -200)) :rotate (/ pi 6) :y))
(setq *door-knob3* (send (send 
    (send *door-knob2-right* :copy-worldcoords) :locate #f(200 0 -200)) :rotate (/ pi 6) :y))
(setq *door-knob4* (send (send (send *door-knob3* :copy-worldcoords) :rotate (/ pi 6) :z) :locate #f(-70 0 0)))
; (if (boundp '*co*)
;   (add-door (+ (- (elt (send *fetch* :worldpos) 0) (elt (send *door-knob3* :worldpos) 0)) 50)))
(if (boundp '*co*)
  (add-door (+ 700 50)))


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
; (look-at-coord *door-knob3* 3000)
(ik-coord *door-knob3-right* 2000)
(unix:sleep 1)
(ik-coord *door-knob3* 3000)
(send *ri* :stop-grasp :effort 60)
(unix:sleep 2)
; (look-at-coord *door-knob3-right* 3000 t)
(ik-coord *door-knob4* 2000)
(ik-coord *door-knob2-right* 2000)

(move-waist 100.0)
(reset-pose 7000)
(if (boundp '*co*)
  (progn (del-door)
      (del-box)))