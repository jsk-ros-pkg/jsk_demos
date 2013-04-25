(defun sweep-under-table-yukizaki ()
  ;;モップを掴み直す
  (r2p)
  (setq z-axis-inversed-p
        (let ((r (send (send *pr2* :larm :end-coords) :rotate-vector #f(0 0 1))))
          (if (> (elt r 2) 0) t nil)))
  (setq z-axis (if z-axis-inversed-p 180
                 0))
  (send *pr2* :inverse-kinematics
        (make-coords :pos (send *pr2* :transform-vector #f(800 600 900)) :rpy (float-vector 0 0 (deg2rad z-axis)))
        :rotation-axis t
        :move-target (send *pr2* :larm :end-coords)
        :use-torso t
        :debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (p2r)
  (send *ri* :wait-interpolation)


  (send *ri* :larm :stop-grasp)
  (unix::sleep 2)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :move-end-pos #f(-500 0 0))
  (p2r)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  (send *ri* :wait-interpolation)
  ;;sweep under table
  (let ((i 0))
    (loop
     (if (= i 3) (return i))
     (setq i (+ i 1))
     ;;   (send *pr2* :angle-vector start-sweep)
     (send *pr2* :inverse-kinematics
           (make-coords :pos (send *pr2* :transform-vector #f(140 600 700)) :rpy (float-vector -0.2 -0.4 (deg2rad z-axis)))
           :rotation-axis t
           :move-target (send *pr2* :larm :end-coords)
           :link-list (send *pr2* :link-list
                            (send *pr2* :larm :end-coords :parent)
                            (send *pr2* :torso_lift_link))
           :debug-view nil)
     (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
     (p2r)
     (send *ri* :wait-interpolation)
     ;;   (send *pr2* :angle-vector end-sweep)
     (send *pr2* :inverse-kinematics
           (make-coords :pos (send *pr2* :transform-vector (float-vector 640 (- 500 (* 150 i)) 500)) :rpy (float-vector 0 -0.7 (deg2rad z-axis)))
           :rotation-axis t
           :move-target (send *pr2* :larm :end-coords)
           :link-list (send *pr2* :link-list
                            (send *pr2* :larm :end-coords :parent)
                            (send *pr2* :torso_lift_link))
           :debug-view nil)
     (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
     (p2r)
     (send *ri* :wait-interpolation)
     ))


  )