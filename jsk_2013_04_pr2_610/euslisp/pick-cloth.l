#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;; pick croth by Iwaishi
;;;;


(load "package://jsk_2013_04_pr2_610/euslisp/move-chair.l")


(defun pick-cloth ()
  (warn "pick-cloth ()~%")
  (let (croth-coords)
;    (send *pr2* :reset-pose)
;    (send *pr2* :move-to *chair-spot* :world)
;    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    ;;kamae
;    (send *pr2* :rarm :angle-vector #f(-40.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0))
;    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;    (send *ri* :wait-interpolation)
;    (send *ri* :stop-grasp :rarm :wait t)

    (send *ri* :go-pos-unsafe 0.2 0 0)
    (send *pr2* :translate #f(200 0 0))


;;----------------------------------------------------------------
;;    find out chair

    (ros::ros-info "start detect chair")
    (start-detect-chair)
    (ros::ros-info "end detect chair")
;;----------------------------------------------------------------

    (send *pr2* :head :look-at (send *chair* :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)


    ;;grasp
    (setq croth-coords
          (make-coords
           :pos
           (midpoint 0.5
                     (send (send *chair* :handle-above-cloth-larm) :worldpos)
                     (send (send *chair* :handle-above-cloth-rarm) :worldpos))
           :rot
           (rotate-matrix (send (send *chair* :handle-above-cloth-larm) :worldrot) -pi/2 :x)))
    (if (boundp '*irtviewer*) (send croth-coords :draw-on :flush t))
    (send *pr2* :rarm :inverse-kinematics
          croth-coords
          :revert-if-fail nil
          :look-at-target t)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm) ;; wait motion and grasp
    (send *ri* :start-grasp :rarm) ;; wait motion and grasp ;; make sure that we really grasp cloth

    ;;lift up
    (send *pr2* :rarm :angle-vector #f(-20.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0))
    (send *pr2* :look-at-hand :rarm)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :start-grasp :rarm)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :go-pos-unsafe -0.4 0 0)
    (send *pr2* :translate #f(-400 0 0))
    (send *ri* :wait-interpolation)

    ))
