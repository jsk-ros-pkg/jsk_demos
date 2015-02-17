(load "package://euslib/demo/otsubo/hrp2-debri-task/generate-debri-motion.l")


(defun generate-stand-point
  (&key (debri-r 40) (debri-h 1000)
        (debri-coords (make-coords :pos #f(500 -100 0) :rpy (list 0 0 0.3)))
        (handle-ratio 0.8)
        (generate-ri? t) (draw? t))
  
  
  (set-debri-environment)
cx
  (setq *map-min-point* (list -300 -300 0))
  (setq *map-max-point* (list 300 300 0))
  (setq *map-step-dist* (float-vector 50 50 0))
  (setq *map-point-list* nil)
  (setq *max-list* nil)

  (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) (elt *map-step-dist* 0))))
    (dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) (elt *map-step-dist* 1))))
      (let* ((max-point 0)
             (map-point
              (float-vector
               (+ (elt *map-min-point* 0) (* ix (elt *map-step-dist* 0)))
               (+ (elt *map-min-point* 1) (* iy (elt *map-step-dist* 1)))
               0
               )))
        
        (send *robot* :fix-leg-to-coords (make-coords :pos map-point))
        (warning-message 2 "before :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (send *robot* :reset-manip-pose)
        (send *robot* :fix-leg-to-coords  (send (send *robot* :foot-midcoords) :copy-worldcoords))
        
        (generate-debri-motion)
        (get-max-point)
        (send *debri* :newcoords (make-coords :pos (float-vector 500 -100 0) :rpy (list 0 0 0.3))) 
        (warning-message 2 "after :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (push map-point *map-point-list*)
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *debri* *ground*))
        (setq *debri-map-point-list* (reverse *map-point-list*))
        (print (length *map-point-list*))
        )))
  (setq *debri-max-list* (reverse *max-list*))
  )


(defun load-log
  ()
  (load "package://euslib/demo/otsubo/log-map-0.l"))

(defun draw-inverse-rv-map
  ()
  (setq *c-list* nil)

  ;; (dotimes (i (length *max-list*))                                                                                                    
  ;;   (when (equal (nth i *max-list*) 20)                                                                                               
  ;;     (setq (nth i *max-list*) 10)))                                                                                                  

  (dotimes (i (length *max-list*))
    (setq ci (make-cube 40 40 2))
    (send ci :locate (nth i *map-point-list*))
    (send ci :set-color (float-vector (/ (nth i *max-list*) 9.0) 0 (- 1.0 (/ (nth i *max-list*) 9.0))))
    (push ci *c-list*)
    )
  (objects (append (list *debri* *robot* *ground*) *c-list*))
  )

(defun get-max-point
  ()
  (let* ((max-point 0))
    (dotimes (i (length *debri-ik-results*))
      (when (car (cdr (nth i *debri-ik-results*)))
        (incf max-point))
      (push max-point *max-list*)
      ))
  )
