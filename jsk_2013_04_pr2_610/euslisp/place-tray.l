;; to test
;; (load "setup.l")
;; (setup)
;; (pick-tray)
;; (place-tray)
(defun place-tray ()
  (let ((table-height 790) p)
    ;;
    ;; place_tray function
    ;;
    ;; Before this function is called, the robot is assumed to have the tray gripped in both grippers
    ;; and the tray is posiitoned directly above the table.
    ;;
    ;;

    ;; get current gripper pose
    (setq p (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
    (send *pr2* :arms :move-end-pos (float-vector 0 0 (- table-height (elt p 2))) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; open grippers
    (send *ri* :stop-grasp :arms)
    (send *pr2* :larm :end-coords :dissoc *tray*) ;; ???

    ;; move arms apart
    (send *pr2* :arms :move-end-pos #f(0 0 50) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ))
