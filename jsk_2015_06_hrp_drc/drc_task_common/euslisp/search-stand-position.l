(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://euslib/demo/otsubo/generate-valve-motion.l")


(defun generate-rotable-point
  (&key (valve-r 200) (valve-coords (make-coords :pos #f(500 0 1200) :rpy (list 0 pi/2 0))))

  (set-valve-enviroment)

  (send *obj* :rotate (deg2rad 20) :z :world)
  (setq *map-min-point* (list -300 -300 0))
  (setq *map-max-point* (list 300 300 0))
  (setq *map-step-dist* (float-vector 50 50 0))
  (setq *map-point-list* nil)
  (setq *max-list* nil)
  
  (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) (elt *map-step-dist* 0))))
    (dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) (elt *map-step-dist* 1))))
      (let ((max-point 0)
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
        
        (generate-valve-motion)
        (search-rotatable-range)
        (warning-message 2 "after :~a~%" (send (send *robot* :foot-midcoords) :copy-worldcoords))
        (max-list-index)
        (push map-point *map-point-list*)
        (push (car (cdr *max-index*)) *max-list*)
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *obj* *ground*))
        )))  
)
  
