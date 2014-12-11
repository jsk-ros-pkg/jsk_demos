(defun gen-motion-large-valve-rotation
  (&key (shift (float-vector 0 0 0))
        (target-pos-offset (float-vector 0 0 0))
        (target-yaw-offset 0)
        (start 0) (end 90) (offset 0) (interval 10)
        (time-interval 1000)
        (object "drcbox-valve-large") (valve-object (send *drcbox* :object object))
        (real nil) (wait real))
  (labels ((get-target
            (valve deg)
            (send (send valve :joint :crank-joint) :joint-angle deg)
            (send (send valve :handle-valve-handle) :copy-worldcoords))
           (solve-ik
            (robot tc)
            (send robot :fix-leg-to-coords (make-coords))
            (with-move-target-link-list
             (mt ll robot '(:rarm :rleg :lleg))
             (send robot :fullbody-inverse-kinematics tc
                   :move-target mt :link-list ll
                   :target-centroid-pos (v+ (apply #'midpoint 0.5 (send robot :legs :end-coords :worldpos)) shift)
                   :debug-view nil;;:no-message
                   :additional-weight-list
                   (list (list (send robot :torso :waist-y :child-link) 0)
                         (list (send robot :torso :waist-p :child-link) 0)
                         (list (send robot :rleg :toe-p :child-link) 0)
                         (list (send robot :lleg :toe-p :child-link) 0))
                   :avoid-collision-distance 100
                   :avoid-collision-null-gain 5.0
                   :avoid-collision-joint-gain 3.0
                   :collision-avoidance-link-pair
                   (list (list (caddr (send robot :lleg :links))
                               (car (last (send robot :larm :links))))))))
           (make-deg-list
            (start end interval)
            (let (tmp)
              (if (< start end)
                  (do ((i start (+ i interval)))
                      ((> i end) 'done)
                    (push (+ i offset) tmp))
                (do ((i start (- i interval)))
                    ((< i end) 'done)
                  (push (+ i offset) tmp)))
              (reverse tmp))))
    (let ((deg-list
           (make-deg-list start end interval)
           ;;(append (make-deg-list start end interval) (reverse (make-deg-list start end interval)))
           )
          (tm 0)
          tc-list ret)
      (dolist (i deg-list)
        (solve-ik *robot*
                  (list (send
                         (send
                          (send (get-target valve-object i) :copy-worldcoords)
                          :translate target-pos-offset :local)
                         :rotate target-yaw-offset :z :local)
                        (send *robot* :rleg :end-coords :copy-worldcoords)
                        (send *robot* :lleg :end-coords :copy-worldcoords)
                        ))
        (send *irtviewer* :draw-objects)
        (push (list :angle-vector (send *robot* :angle-vector)
                    :root-coords (send (car (send *robot* :links)) :copy-worldcoords)
                    :contact-state '(:rarm :rleg :lleg)
                    :end-coords-list (mapcar #'(lambda (l) (send *robot* l :end-coords)) '(:rarm :rleg :lleg))
                    :time tm)
              ret)
        (setq tm (+ tm time-interval))
        (when real
          (send *ri* :angle-vector (send *robot* :angle-vector) time-interval)
          (send *ri* :wait-interpolation))
        (when wait (read-line))
        (x::window-main-one)
        )
      (reverse ret))))

