(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://euslib/demo/otsubo/generate-valve-motion.l")


(defun generate-rotable-point
  (&key (valve-r 200) (valve-coords (make-coords :pos #f(500 0 1200) :rpy (list 0 pi/2 0))))

  (set-valve-enviroment)

  (send *obj* :rotate (deg2rad 20) :z :world)
  (setq *map-min-point* (list -600 -600 0))
  (setq *map-max-point* (list 600 600 0))
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
        (if (equal (car (cdr *max-index*)) 20)
            (push 9 *max-list*)
          (push (car (cdr *max-index*)) *max-list*))
        (warn "ix=~a~%" ix)
        (warn "iy=~a~%" iy)
        (objects (list *robot* *obj* *ground*))
        )))  
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
  (objects (append (list *obj* *robot* *ground*) *c-list*))
  )  

(defun max-list-index
  ()
  (let* ((valve-rotatable-range (search-rotatable-range))
         (i 0)
         (max 0)
         (index 0))
    (dotimes (i (/ (length *valve-ik-results*) 2))
      (when (< max (nth i valve-rotatable-range))
        (setq max (nth i valve-rotatable-range))
        (setq index i)
        )
      (incf i)
      )
    (setq *max-index* (list index max))      
    ))

(defun draw-ik-results
  ()
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* (draw-polygon
         draw-polygon-edges)
    (dotimes (i (/ (length *valve-ik-results*) 2))
      (when (car (cdr (nth i *valve-ik-results*)))
        (send (send *valve* :get :grasp-coords) :draw-on :flush t :color (float-vector 1 0 0) :width 5 :size 50)
        (push (send (send (send *valve* :get :grasp-coords) :copy-worldcoords) :worldpos) draw-polygon-edges)
        )
      (send *valve* :rotate (+ (deg2rad (/ 360.0 (- (/ (length *valve-ik-results*) 2) 1)))) :z :local)
      )
    
    ;; (push (send *valve* :worldpos) draw-polygon-edges)
    ;; (setq b draw-polygon-edges)
    ;; (setq a (quickhull draw-polygon-edges))
    ;; (send (make-prism draw-polygon-edges (send (send *valve* :worldcoords) :rotate-vector (float-vector 0 0 1))) :draw-on :flush t :color (float-vector 1 0 0) :width 5)
    ))

(defun draw-ik-results2
  ()
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* (draw-polygon
         draw-polygon-edges
         buf)
    (dotimes (i (/ (length *valve-ik-results*) 2))
      (when (car (cdr (nth i *valve-ik-results*)))
        ;;(send (send *valve* :get :grasp-coords) :draw-on :flush t :color (float-vector 1 0 0) :width 5 :size 50)
        (push (send (send (send *valve* :get :grasp-coords) :copy-worldcoords) :worldpos) draw-polygon-edges)
        )
      (send *valve* :rotate (+ (deg2rad (/ 360.0 (- (/ (length *valve-ik-results*) 2) 1)))) :z :local)
      )
    (setq
     buf
     (mapcar
      '(lambda (p1 p2)
         (make-prism (list (send *valve* :worldpos) p1 p2) (send (send *valve* :worldcoords) :rotate-vector (float-vector 0 0 1))))
      (append (last draw-polygon-edges) (butlast draw-polygon-edges))
      draw-polygon-edges))
    ;; (push (send *valve* :worldpos) draw-polygon-edges)
    ;; (send (make-prism draw-polygon-edges (send (send *valve* :worldcoords) :rotate-vector (float-vector 0 0 1))) :draw-on :flush t :color (float-vector 1 0 0) :width 5)
    (send-all buf :draw-on :flush t :color (float-vector 1 0 0) :width 5)
    (send-all buf :set-color (float-vector 0 1 0))
    buf
    ))
