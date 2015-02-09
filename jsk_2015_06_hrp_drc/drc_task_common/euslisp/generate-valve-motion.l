(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")

(defun set-valve-enviroment
  (&key (valve-r 200) (valve-coords (make-coords :pos #f(500 0 1200) :rpy (list 0 pi/2 0))))

  ;; generate robot
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (send *robot* :reset-manip-pose)

  (setq *ground* (make-cube 5000 5000 1))
  (objects (list *robot* *ground*))
  (send *irtviewer* :change-background #f(1 1 1))
  (send *irtviewer* :draw-objects)
  
  ;; place robot to origin
  (send *robot* :fix-leg-to-coords (make-coords))

  ;; generate valve
  (let* ((valve-sr (* 0.1 valve-r)))
    (setq *obj* (make-ring valve-r valve-sr :segments 16))
    )

  ;; atattch handle to valve
  (send *obj* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos (float-vector valve-r 0 0) :rpy (list 0 0 pi/2))
         :parent *obj*))

  ;; place valve to valve-coords
  (send *obj* :newcoords valve-coords)

  ;; draw
  (objects (list *robot* *obj* *ground*)) 

  )
(warning-message 2 "(set-valve-enviroment)~%")


(defun reset-valve-angle
  ()
  (let* ((handle-height-min 1e10)
         valve-rot-min)
    (dotimes (i 360)
      (send *obj* :rotate (deg2rad 1) :z :local)
      (when (> handle-height-min (elt (send (send *obj* :get :grasp-coords) :worldpos) 2))
        (setq handle-height-min (elt (send (send *obj* :get :grasp-coords) :worldpos) 2))
        (setq valve-rot-min (send (send (send *obj* :worldcoords) :copy-worldcoords) :worldrot))
        )
      )
    (send *obj* :newcoords (make-coords :pos (send (send *obj* :worldcoords) :worldpos) :rot valve-rot-min))
    )
  )


(defun generate-valve-motion
  ()
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* ((devide-num 10)
         ;;(devide-num 36)
         (reach-dist 70)
         (ik-results nil)
         (ik-list nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; rotate valve
    (dotimes (i (* devide-num 2))
      (print i)
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      (send *obj* :rotate (+ (deg2rad (/ 360.0 (- devide-num 1)))) :z :local)
      ;; solve ik
      (let* ((ik-ret nil)
             (tc nil)
             (ik-ret-dist nil)
             ))
      ;; check valve handle
      (setq tc
            (list
             (send *obj* :get :grasp-coords)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (with-move-target-link-list
       (mt ll *robot* '(:rarm :rleg :lleg))
       (setq ik-ret 
             (send *robot*
                   :fullbody-inverse-kinematics tc
                   ;;:look-at-target t    
                   :move-target mt :link-list ll
                   :additional-weight-list
                   (list (list (send *robot* :rleg :toe-p :child-link) 0)
                         (list (send *robot* :lleg :toe-p :child-link) 0))
                   :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5)
                   :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                   ;;:centroid-offset-func #'(lambda () (send *robot* :calc-static-balance-point (list #f(0 0 0) #f(0 0 0)))) 
                   ;;translation-axis can reduced translation constraint                                                              
                   :translation-axis (list t t t)
                   ;; rotation-axis can reduce rotation constraint                                                                    
                   :rotation-axis (list :z t t)
                   :stop 50
                   :debug-view nil
                   )))
      ;; check reaching point
      (setq tc
            (list
             (send (send (send *obj* :get :grasp-coords) :copy-worldcoords) :translate (float-vector reach-dist 0 0) :local)
             (send *robot* :rleg :end-coords :copy-worldcoords)
             (send *robot* :lleg :end-coords :copy-worldcoords)))
      (setq
       ik-ret-dist
       (and ik-ret
            (with-move-target-link-list
             (mt ll *robot* '(:rarm :rleg :lleg))
             (setq ik-ret 
                   (send *robot*
                         :fullbody-inverse-kinematics tc
                         ;;:look-at-target t    
                         :move-target mt :link-list ll
                         :additional-weight-list
                         (list (list (send *robot* :rleg :toe-p :child-link) 0)
                               (list (send *robot* :lleg :toe-p :child-link) 0))
                         :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5)
                         :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                         ;;:centroid-offset-func #'(lambda () (send *robot* :calc-static-balance-point (list #f(0 0 0) #f(0 0 0)))) 
                         ;;translation-axis can reduced translation constraint                                                              
                         :translation-axis (list t t t)
                         ;; rotation-axis can reduce rotation constraint                                                                    
                         :rotation-axis (list :z t t)
                         :stop 50
                         :debug-view nil
                         )))))
      ;; draw
      (send *irtviewer* :draw-objects :flush t)
      (push (list i (not (null ik-ret))) ik-results)
      (push (list ik-ret ik-ret-dist) ik-list)
      )
    (setq *ik-results* (reverse ik-results))
    (setq *ik-list* (reverse ik-list))
    (print *ik-results*)
    ))
(warning-message 2 "(generate-valve-motion)~%")


(defun search-rotatable-range
  ()
  (let* ((test-point 0))
    (setq *rotable-point* nil)
    (dotimes (i (/ (length *ik-results*) 2))
      (setq test-point 0)
      (print i)
      (setq j (+ i 1))
      (when (car (cdr (nth i *ik-results*)))
        (incf test-point)
        (print test-point)
        (while (car (cdr (nth j *ik-results*)))
          (incf test-point)
          (incf j))
        )
      (push test-point *rotable-point*)
      (print *rotable-point*)
      )
    (setq *rotable-point* (reverse *rotable-point*))      
    )
  )
(warning-message 2 "(search-rotatable-range)~%")

(defun max-list-index
  ()
  ;;(setq *rotable-point* '(0 1 0 0 0 0 3 2 1 0))
  (let* ((i 0)
         (max 0)
         (index 0))
    (dotimes (i (/ (length *ik-results*) 2))
      (when (< max (nth i *rotable-point*))
        (setq max (nth i *rotable-point*))
        (setq index i)
        )
      (incf i)
      )
    (setq *max-index* (list index max))      
    )
  )
(warning-message 2 "(max-list-index)~%")

(defun draw-ik-results
  ()
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* (draw-polygon
         draw-polygon-edges)
    (dotimes (i (/ (length *ik-results*) 2))
      (when (car (cdr (nth i *ik-results*)))
        (send (send *obj* :get :grasp-coords) :draw-on :flush t :color #f(1 0 0) :width 5 :size 50)
        (push (send (send (send *obj* :get :grasp-coords) :copy-worldcoords) :worldpos) draw-polygon-edges)
        )
      (send *obj* :rotate (+ (deg2rad (/ 360.0 (- (/ (length *ik-results*) 2) 1)))) :z :local)
      )
    
    ;; (push (send *obj* :worldpos) draw-polygon-edges)
    ;; (setq b draw-polygon-edges)
    ;; (setq a (quickhull draw-polygon-edges))
    ;; (send (make-prism draw-polygon-edges (send (send *obj* :worldcoords) :rotate-vector #f(0 0 1))) :draw-on :flush t :color #f(1 0 0) :width 5)
    )
  )
(warning-message 2 "(draw-ik-results)~%")

(defun draw-ik-results2
  ()
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* (draw-polygon
         draw-polygon-edges
         buf)
    (dotimes (i (/ (length *ik-results*) 2))
      (when (car (cdr (nth i *ik-results*)))
        ;;(send (send *obj* :get :grasp-coords) :draw-on :flush t :color #f(1 0 0) :width 5 :size 50)
        (push (send (send (send *obj* :get :grasp-coords) :copy-worldcoords) :worldpos) draw-polygon-edges)
        )
      (send *obj* :rotate (+ (deg2rad (/ 360.0 (- (/ (length *ik-results*) 2) 1)))) :z :local)
      )
    (setq
     buf
     (mapcar
      '(lambda (p1 p2)
         (make-prism (list (send *obj* :worldpos) p1 p2) (send (send *obj* :worldcoords) :rotate-vector #f(0 0 1))))
      (append (last draw-polygon-edges) (butlast draw-polygon-edges))
      draw-polygon-edges))
    ;; (push (send *obj* :worldpos) draw-polygon-edges)
    ;; (send (make-prism draw-polygon-edges (send (send *obj* :worldcoords) :rotate-vector #f(0 0 1))) :draw-on :flush t :color #f(1 0 0) :width 5)
    (send-all buf :draw-on :flush t :color #F(1 0 0) :width 5)
    (send-all buf :set-color #F(0 1 0))
    buf
    )
;;  (objects (append (draw-ik-results2) (list *robot* *obj*)))
  )
(warning-message 2 "(draw-ik-results)~%")


(defun get-angle-vector
  ()
  (
