;;(load "package://euslib/demo/otsubo/hrp2-hose-task/generate-hose-motion.l")
(load "./generate-hose-motion.l")

(defun generate-stand-point-for-connect
  (&key (hose-coords (make-coords :pos (float-vector 500 0 800) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t))
  
  
  (set-hose-connect-environment)
  (setq *map-min-point* (list -600 -600 0))
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
        (reset-hose-position)
        (send *robot* :fix-leg-to-coords (make-coords :pos map-point))
        (Warning-message 2 "before :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (send *robot* :reset-manip-pose)
        (send *robot* :fix-leg-to-coords  (send (send *robot* :foot-midcoords) :copy-worldcoords))

        (generate-hose-connect-motion)
        (get-max-point)
        ;;(send *hose* :newcoords (make-coords :pos (float-vector 500 -100 0) :rpy (list 0 0 0.3)))
        (warning-message 2 "after :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (push map-point *map-point-list*)
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *hose* *ground*))
        (setq *hose-map-point-list* (reverse *map-point-list*))
        (print (length *map-point-list*))
        (send (send *irtviewer* :viewer :viewsurface) :write-to-image-file (format nil "./pictures/~a-~a.png" ix iy)
        )))
  (setq *hose-max-list* (reverse *max-list*))
  )
)

(defun reset-hose-position
  ()
  (send *hose* :newcoords (make-coords :pos (float-vector 500 0 800) :rpy (list 0 pi/2 0)))
  )
 

(defun load-log
  ()
  (load "package://euslib/demo/otsubo/log-map-0.l"))

(defun draw-inverse-rv-map
  ()
  (setq *c-list* nil)

  ;; (dotimes (i (length *max-list*)
  ;;   (when (equal (nth i *max-list*) 20)                                                                                               
  ;;     (setq (nth i *max-list*) 10)))                                                                                                  

  (dotimes (i (length *max-list*))
    (setq ci (make-cube 40 40 2))
    (send ci :locate (nth i *map-point-list*))
    (send ci :set-color (float-vector (/ (nth i *max-list*) 8.0) 0 (- 1.0 (/ (nth i *max-list*) 8.0))))
    (push ci *c-list*)
    )
  (objects (append (list *hose* *robot* *ground*) *c-list*))
  )

(defun get-max-point
  ()
  (let* ((max-point 0))
    (dotimes (i (length *hose-ik-results*))
      (when (car (cdr (nth i *hose-ik-results*)))
        (incf max-point)))
      (push max-point *max-list*)
      )
)
 

(defun test-hose-connect-motion
  ()
  ;;(set-hose-connect-environment)
  ;;(send *robot* :fix-leg-to-coords (make-coords :pos #f(-500 0 0)))
  (generate-hose-connect-motion :draw? t :try-all-ik? t)
  (warning-message 2 "ik results list: ~a~%ik success num: ~a~%" (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*) (count t (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*)))
  )



(defun generate-stand-point-for-connect2
  (&key (hose-coords (make-coords :pos (float-vector 500 0 800) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t)) 

  (set-hose-connect-environment)
  (setq *max-list* nil)
  (send *robot* :fix-leg-to-coords (make-coords :pos #f(-500 0 0)))
  (setq *map-min-point* (list -600 -600 0))
  (setq *map-max-point* (list 100 300 0))
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
        ;;(reset-hose-position)
        (send *robot* :reset-manip-pose)
        (send *robot* :fix-leg-to-coords (make-coords :pos map-point))
        (warning-message 2 "before :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (send *robot* :reset-manip-pose)
        (send *robot* :fix-leg-to-coords  (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (send (send *irtviewer* :viewer :viewsurface) :write-to-image-file (format nil "./pictures/before/~a-~a-b.png" ix iy))
        ;;(unix::sleep 5)
        (test-hose-connect-motion)
        ;;(generate-hose-connect-motion)
        ;;(get-max-point)
        ;;(send *hose* :newcoords (make-coords :pos (float-vector 500 -100 0) :rpy (list 0 0 0.3)))
        (warning-message 2 "after :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (push map-point *map-point-list*)
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *hose* *ground*))
        (setq max-point (count t (mapcar #'(lambda (x) (elt x 1)) *hose-ik-results*)))
        (push max-point *max-list*)
        ;;(setq *hose-map-point-list* (reverse *map-point-list*))
        ;;(print (length *map-point-list*))
        (send (send *irtviewer* :viewer :viewsurface) :write-to-image-file (format nil "./pictures/after/~a-~a.png" ix iy))
        ))
    (setq *hose-max-list* (reverse *max-list*))
    )
  )

(defun save-rv-map
  ()
  (dump-loadable-structure "./hogehoge.l" *map-step-dist* *map-point-list* *max-list*))


