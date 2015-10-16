(load "door-open.l")

(defun generate-stand-point-for-door ()
  (setq *map-min-point* (list -400 -400 0))
  (setq *map-max-point* (list 400 400 0))
  (setq *map-step-dist* (float-vector 100 100 0))
  ;; (setq *map-step-dist* (float-vector 400 400 0))
  (setq *map-point-list* nil)
  (setq *max-list-all* nil)
  (setq *success-list* nil)
  (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) 
                         (elt *map-step-dist* 0))))
    (dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) 
                           (elt *map-step-dist* 1))))
      (let ((max-point 0)
            (map-point
             (float-vector
              (+ -800 (+ (elt *map-min-point* 0) (* ix (elt *map-step-dist* 0))))
              (+ -800 (+ (elt *map-min-point* 1) (* iy (elt *map-step-dist* 1))))
              0
              )))
        (send *robot* :fix-leg-to-coords (make-coords :pos map-point))
        (warning-message 2 "before :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (send *robot* :reset-manip-pose)
        (send *robot* :fix-leg-to-coords  (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (block opening-roop
          (dolist (x (list 0 -10 -20 -30 -40 -50 -60 -70))
            (if (door-reach2 :draw? nil :door-angle x)
                (push t *success-list*)
              (return-from opening-roop)
              )))
        (warning-message 2 "after :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *door* ))
        (push map-point *map-point-list*)
        (push (count t *success-list*) *max-list-all*)
        (setq *success-list* nil))
      )
    )
  )


(defun load-log
  ()
  (load "./map-door-with-cage-grasp.l"))

(defun save-log
  ()
  (dump-loadable-structure "./map-door-with-cage-grasp.l" *map-point-list* *max-list-all*))


(defun draw-inverse-rv-map-all
  ()
  (setq *c-list* nil)
  
  ;; (dotimes (i (length *max-list*))    
  ;;   (when (equal (nth i *max-list*) 20)
  ;;     (setq (nth i *max-list*) 10)))
  
  (dotimes (i (length *max-list-all*))
    (setq ci (make-cube 90 90 2))
    (send ci :locate (nth i *map-point-list*))
    ;; (send ci :set-color (float-vector (/ (nth i *max-list-all*) 8.0) 0 (- 1.0 (/ (nth i *max-list-all*) 8.0))))
    (if (= (nth i *max-list-all*) 8) (send ci :set-color #f(1 0 0)))
    (push ci *c-list*)
    )
  (objects (append (list *door*) *c-list*))
  )  




