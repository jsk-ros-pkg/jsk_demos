;; to test
;; (load "setup.l")
;; (setup)
;; (pick-tray)
;; (place-tray)
(defun place-tray ()
  (let ((table-height 850)
        (yokei-up-height 150)
        (yokei-down-height 150)
        p)
    ;;
    ;; place_tray function
    ;;
    ;; Before this function is called, the robot is assumed to have the tray gripped in both grippers
    ;; and the tray is posiitoned directly above the table.
    ;;
    ;;

    ;; get current gripper pose
    (setq p (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
    (send *pr2* :arms :move-end-pos (float-vector 0 0 (- (+ table-height yokei-up-height) (elt p 2))) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; for navigation
    (speak-jp "まえにすすみます")
    ;;(speak-en "DANGER go 30 centimeter.")
    ;;(send *ri* :go-pos-unsafe 0.29 0)
    (send *ri* :go-pos-unsafe 0.34 0)

    ;; put forward a tray
    (send *pr2* :arms :move-end-pos (send *pr2* :rotate-vector #f(0 0 10)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; put down a tray
    (send *pr2* :arms :move-end-pos (float-vector 0 0 (- (- table-height yokei-down-height) (elt p 2))) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)


#|
    (dolist (offset (list (cons #f(0 30 0) #f(0 -30 0)) (cons #f(0 60 0) #f(0 -60 0))))
      (send *pr2* :inverse-kinemat
\cs
            (list
             (send (send tray-larm-handle :copy-worldcoords) :translate (car offset))  ;; use local coords, this cod works if tray is placed on different orientation
             (send (send tray-rarm-handle :copy-worldcoords) :translate (cdr offset)))
            :link-list (list     ;; https://sourceforge.net/p/jskeus/tickets/20/
                        (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                        (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
            :move-target (list (send *pr2* :larm :end-coords) (send *pr2* :rarm :end-coords))
            :use-torso t
            :revert-if-fail nil
            :look-at-target (midpoint 0.5 (send tray-larm-handle :worldpos) (send tray-rarm-handle :worldpos))
            )      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
|#
#|    (send *pr2* :inverse-kinematics
          (list (send *pr2* :rotate-vector #f(0 50 0))
                (send *pr2* :rotate-vector #f(0 -50 0)))
          :link-list (list
                      (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
          :move-target (list (send *pr2* :larm :end-coords) (send *pr2* :rarm :end-coords))
          :use-torso t
          :look-at-target nil)|#
    (send *pr2* :larm :move-end-pos (send *pr2* :rotate-vector #f(0 10 0)))
    (send *pr2* :rarm :move-end-pos (send *pr2* :rotate-vector #f(0 -10 0)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; open grippers
    (send *ri* :stop-grasp :arms)
    (send *pr2* :larm :end-coords :dissoc *tray*) ;; ???

    ;; move arms apart
    (send *pr2* :arms :move-end-pos #f(0 0 50) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    ;; for navigation
    (speak-jp "うしろにさがります")
;;    (speak-en "DANGER go back 50 centimeters")
    (send *ri* :go-pos-unsafe -1 0)
    (send *pr2* :translate (float-vector -1000 0 0))

    ))
