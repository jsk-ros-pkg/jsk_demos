(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defun set-valve-environment
  (&key (valve-r 200)
        (valve-coords (make-coords :pos (float-vector 500 0 1200) :rpy (list 0 pi/2 0)))
        (generate-ri? t) (draw? t))
  ;; generate robot
  (unless (boundp '*robot*)
    (cond (generate-ri?
           (hrp2jsknts-init)
           (setq *robot* *hrp2jsknts*))
          (t
           (setq *robot* (instance hrp2jsknts-robot :init)))))
  ;; place robot to origin
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; generate valve
  (let* ((valve-sr (* 0.1 valve-r)))
    (setq *valve* (make-ring valve-r valve-sr :segments 16))
    )
  ;; atattch handle to valve
  (send *valve* :put :grasp-coords
        (make-cascoords
         :coords (make-coords :pos (float-vector valve-r 0 0) :rpy (list pi/2 -pi/2 0))
         :parent *valve*))
  ;; place valve to valve-coords
  (send *valve* :newcoords valve-coords)
  ;; draw
  (when draw?
    (setq *ground* (make-cube 5000 5000 1))
    (objects (list *robot* *valve* *ground*))
    (send *irtviewer* :draw-objects)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    )
  )
;;(warning-message 2 "(set-valve-environment)~%")

(defun set-valve-environment-full
  (valve-r
   valve-coords ;; body relative
   &key
   (draw? nil)
   (use-ri? t))
  (let* (valve-worldcoords)
    ;; place robot and valve to initial coords
    (set-valve-environment :valve-r valve-r :draw? draw? :generate-ri? use-ri?)
    (cond (use-ri?
           (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
          (t
           (send *robot* :angle-vector (get-potentio-vector-from-joint-states))))
    (send *robot* :fix-leg-to-coords (make-coords) '(:lleg :rleg))
    (send *valve* :newcoords (send (send (send *robot* :body_lk) :copy-worldcoords) :transform valve-coords :local))
    (when draw? (send *irtviewer* :draw-objects))
    ))

(defun reset-valve-angle
  ()
  (let* ((handle-height-min 1e10)
         valve-rot-min)
    (dotimes (i 360)
      (send *valve* :rotate (deg2rad 1) :z :local)
      (when (> handle-height-min (elt (send (send *valve* :get :grasp-coords) :worldpos) 2))
        (setq handle-height-min (elt (send (send *valve* :get :grasp-coords) :worldpos) 2))
        (setq valve-rot-min (send (send (send *valve* :worldcoords) :copy-worldcoords) :worldrot))
        )
      )
    (send *valve* :newcoords (make-coords :pos (send (send *valve* :worldcoords) :worldpos) :rot valve-rot-min))
    )
  )

(defun generate-valve-motion
  (&key (draw? t))
  ;; reset valve angle
  (reset-valve-angle)
  ;; loop
  (let* ((devide-num 10)
         (reach-dist 100)
         (ik-results nil)
         (original-foot-midcoords (send (send *robot* :foot-midcoords) :copy-worldcoords)))
    ;; rotate valve
    (dotimes (i devide-num)
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords original-foot-midcoords)
      ;; solve ik
      (let* (ik-ret-in
             ik-ret-out
             (tc nil))
        ;; solve ik for valve handle
        (when draw? (send (send *valve* :get :grasp-coords) :draw-on :flush t))
        (setq tc
              (list
               (send *valve* :get :grasp-coords)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (with-move-target-link-list
         (mt ll *robot* '(:rarm :rleg :lleg))
         (setq ik-ret-in
               (send *robot*
                     :fullbody-inverse-kinematics tc
                     ;;:look-at-target (send *valve* :worldcoords)
                     :move-target mt :link-list ll
                     :additional-weight-list
                     (list (list (send *robot* :rleg :toe-p :child-link) 0)
                           (list (send *robot* :lleg :toe-p :child-link) 0))
                     :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
                     :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                     ;;:centroid-offset-func #'(lambda () (send *robot* :calc-static-balance-point (list (float-vector 0 0 0) (float-vector 0 0 0)))) 
                     ;;translation-axis can reduced translation constraint                                                              
                     :translation-axis (list t t t)
                     ;; rotation-axis can reduce rotation constraint                                                                    
                     :rotation-axis (list t t t)
                     ;;:rotation-axis (list :z t t)
                     :max (float-vector  500  500  0  20  20  10) ;; for hrp2
                     :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
                     :stop 10
                     :debug-view nil
                     ))
         (when ik-ret-in
           (setq ik-ret-in
                 (send *robot* :look-at-target (send *valve* :worldcoords))))
         )
        ;; solve ik for valve reaching point
        (setq tc
              (list
               (send (send (send *valve* :get :grasp-coords) :copy-worldcoords) :translate (float-vector (- reach-dist) (- reach-dist) 0) :local)
               (send *robot* :rleg :end-coords :copy-worldcoords)
               (send *robot* :lleg :end-coords :copy-worldcoords)))
        (when ik-ret-in
          (with-move-target-link-list
           (mt ll *robot* '(:rarm :rleg :lleg))
           (setq ik-ret-out
                 (send *robot*
                       :fullbody-inverse-kinematics tc
                       ;;:look-at-target (send *valve* :worldcoords)
                       :move-target mt :link-list ll
                       :additional-weight-list
                       (list (list (send *robot* :rleg :toe-p :child-link) 0)
                             (list (send *robot* :lleg :toe-p :child-link) 0))
                       :root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.5 0.5)
                       :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                       ;;:centroid-offset-func #'(lambda () (send *robot* :calc-static-balance-point (list (float-vector 0 0 0) (float-vector 0 0 0)))) 
                       ;;translation-axis can reduced translation constraint                                                              
                       :translation-axis (list t t t)
                       ;; rotation-axis can reduce rotation constraint                                                                    
                       :rotation-axis (list t t t)
                       ;;:rotation-axis (list :z t t)
                       :max (float-vector  500  500  0  20  20  10) ;; for hrp2
                       :min (float-vector  -500  -500  -200  -20  -20  -10) ;; for hrp2
                       :stop 10
                       :debug-view nil
                       ))
           (when ik-ret-out
             (setq ik-ret-out
                   (send *robot* :look-at-target (send *valve* :worldcoords))))))
        ;; draw
        (when draw? (send *irtviewer* :draw-objects :flush t))
        (push (list i (not (null (and ik-ret-in ik-ret-out))) ik-ret-in ik-ret-out) ik-results)
        )
      (setq *valve-ik-results* (reverse ik-results))
      ;; rotate valve
      (send *valve* :rotate (+ (deg2rad (/ 360.0 devide-num))) :z :local)
      )
    ))
;;(warning-message 2 "(generate-valve-motion)~%")

(defun search-rotatable-range
  ()
  (let* ((ik-results-double (append *valve-ik-results* *valve-ik-results*))
         (test-point 0)
         (rotable-point nil))
    (dotimes (i (/ (length ik-results-double) 2))
      (setq test-point 0)
      ;;(print i)
      (setq j (+ i 1))
      (when (car (cdr (nth i ik-results-double)))
        (incf test-point)
        ;;(print test-point)
        (while (car (cdr (nth j ik-results-double)))
          (incf test-point)
          (incf j))
        )
      (push test-point rotable-point)
      )
    (setq *valve-rotable-success-num* (reverse rotable-point))
    )
  )
;;(warning-message 2 "(search-rotatable-range)~%")

(defun get-valve-motion
  ()
  (search-rotatable-range)
  (let* ((rotable-point-max (apply #'max *valve-rotable-success-num*))
         (rotable-point-idx (position (apply #'max *valve-rotable-success-num*) *valve-rotable-success-num*))
         angle-vector-ret angle-vector-ret-reach)
    (dotimes (i (length *valve-rotable-success-num*))
      (when (= (elt *valve-rotable-success-num* rotable-point-idx) 0)
        (return-from nil nil))
      (push (elt (elt *valve-ik-results* rotable-point-idx) 2) angle-vector-ret)
      (push (elt (elt *valve-ik-results* rotable-point-idx) 3) angle-vector-ret-reach)
      (setq rotable-point-idx (mod (+ rotable-point-idx 1) (length *valve-rotable-success-num*)))
      )
    (setq angle-vector-ret (append (reverse angle-vector-ret) angle-vector-ret-reach))
    (setq *valve-motion* angle-vector-ret)
    angle-vector-ret
    ))
;;(warning-message 2 "(get-valve-motion)~%")

(defun get-valve-motion-for-fc
  (valve-r valve-coords)
  (set-valve-environment-full valve-r valve-coords :draw? nil)
  (generate-valve-motion :draw? nil)
  (get-valve-motion)
  )

(defun get-valve-motion-for-ocs
  (valve-r valve-coords)
  (set-valve-environment-full valve-r valve-coords :draw? nil :use-ri? nil)
  (generate-valve-motion :draw? nil)
  ;; TODO : draw ik result
  (ros::ros-info "~a~%" *valve-ik-results*)
  (get-valve-motion)
  )

(defun send-valve-motion
  (radius coords &key (real? t))
  (let* ((rotate-num 3)
         (valve-motion-av-list (get-valve-motion-for-fc radius coords))
         (valve-motion-length (length valve-motion-av-list)))
    (when (not (= valve-motion-length 0))
      ;; first reaching
      (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
      (when real? (hand-model2real))
      (send *robot* :reset-manip-pose)
      (when real? (model2real :time 3000))
      (when real? (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 1000 :d-p 400)) ;; start imp
      (send *robot* :angle-vector (car (last valve-motion-av-list)))
      (when real? (model2real :time 5000))
      (dotimes (i rotate-num)
        ;; grasp
        (send *robot* :angle-vector (car valve-motion-av-list))
        (when real? (model2real :time 3000))
        (send *robot* :hand :arms :grasp-pose) ;; hand grasp-pose
        (when real? (hand-model2real))
        ;; rotate
        (when real?
          (send *ri* :angle-vector-sequence (subseq valve-motion-av-list 0 (/ valve-motion-length 2)) (make-list (/ valve-motion-length 2) :initial-element 2000))
          (send *ri* :wait-interpolation))
        ;; relese
        (send *robot* :hand :arms :hook-pose) ;; hand hook-pose
        (when real? (hand-model2real))
        (send *robot* :angle-vector (car (subseq valve-motion-av-list (/ valve-motion-length 2))))
        (when real? (model2real :time 3000))
        (cond ((= i (1- rotate-num))
               ;; last releasing
               (send *robot* :reset-manip-pose)
               (when real? (model2real :time 5000)))
              (t
               ;; reverse
               (when real?
                 (send *ri* :angle-vector-sequence (subseq valve-motion-av-list (/ valve-motion-length 2)) (make-list (/ valve-motion-length 2) :initial-element 2000))
                 (send *ri* :wait-interpolation))
               ))
        )))
  )

(defun publish-valve-motion
  (&key radius coords (step-time 400) (update? t))
  (when update?
    (get-valve-motion-for-ocs radius coords))
  (dolist (av *valve-motion*)
    (publish-model-joint-states :av av)
    (unix::usleep (* step-time 1000))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; source code above here is used in ocs/fc--executive.l ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun max-list-index
  ()
  ;;(setq *valve-rotable-success-num* '(0 1 0 0 0 0 3 2 1 0))
  (let* ((i 0)
         (max 0)
         (index 0))
    (dotimes (i (/ (length *valve-ik-results*) 2))
      (when (< max (nth i *valve-rotable-success-num*))
        (setq max (nth i *valve-rotable-success-num*))
        (setq index i)
        )
      (incf i)
      )
    (setq *max-index* (list index max))      
    )
  )
;;(warning-message 2 "(max-list-index)~%")

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
    )
  )
;;(warning-message 2 "(draw-ik-results)~%")

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
    )
;;  (objects (append (draw-ik-results2) (list *robot* *valve*)))
  )
;;(warning-message 2 "(draw-ik-results)~%")
