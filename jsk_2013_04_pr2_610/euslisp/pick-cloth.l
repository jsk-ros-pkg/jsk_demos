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

    ;; copy from move-to-chair-larm
    (send *pr2* :angle-vector #f(50.1262 0.0 -10.0 40.0 -70.0 70.0 -90.0 90.0 -5.0 70.0 -105.0 -90.0 70.0 -6.0 20.0 0.007004 -1.914))
    (send *pr2* :torso :waist-z :joint-angle 50)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
    (send *ri* :stop-grasp :larm)


    (send *ri* :go-pos-unsafe 0.2 0 0)
    (send *pr2* :translate #f(200 0 0))


;;----------------------------------------------------------------
;;    find out chair

    (ros::ros-info "start detect chair")
    (start-detect-chair)
    (ros::ros-info "end detect chair")
;;----------------------------------------------------------------

;;    (send *pr2* :head :look-at (send *chair* :worldpos))
;;    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)


    ;;grasp
    (setq croth-coords
          (make-coords
           :pos
           (midpoint 0.5
                     (send (send *chair* :handle-above-cloth-larm) :worldpos)
                     (send (send *chair* :handle-above-cloth-rarm) :worldpos))
           :rot
           (rotate-matrix (send (send *chair* :handle-above-cloth-larm) :worldrot) -pi/2 :x)))

    ;look-at-cloth
    (send *pr2* :head :look-at (send croth-coords :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (if (boundp '*irtviewer*) (send croth-coords :draw-on :flush t))

    ;;move rarm
    (let (av-seq av-last)
      (send *pr2* :rarm :inverse-kinematics
            croth-coords
            :revert-if-fail nil
            :look-at-target t)
      (push (send *pr2* :angle-vector) av-seq)
      (setq av-last (send *pr2* :angle-vector))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      
      (send *pr2* :rarm :move-end-pos #f(-100 0 0))
      (push (send *pr2* :angle-vector) av-seq)
      (send *pr2* :angle-vector av-last)

      ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :angle-vector-sequence av-seq (list 2000 1000))
      (send *ri* :wait-interpolation)
      )
    (send *ri* :start-grasp :rarm :objects nil)
    (send *ri* :start-grasp :rarm :objects nil) ;; wait motion and grasp ;; make sure that we really grasp cloth

    ;;lift up
    (send *pr2* :rarm :angle-vector #f(-20.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0))
    (send *pr2* :look-at-hand :rarm)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :start-grasp :rarm)
    (send *ri* :move-gripper :rarm 0.0 :effort 50 :wait t)
    (unix:sleep 1)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (unix:sleep 1)
    (send *ri* :go-pos-unsafe -0.4 0 0)
    (send *pr2* :translate #f(-400 0 0))
    (send *ri* :wait-interpolation)
    ))

(defun pick-cloth-chair ()
  (warn "pick-cloth-chair ()~%")
  (let (croth-coords av-seq)
;    (send *pr2* :reset-pose)
;    (send *pr2* :move-to *chair-spot* :world)
;    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    ;;kamae
;    (send *pr2* :rarm :angle-vector #f(-40.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0))
;    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :larm :wait t)

    (send *ri* :go-pos-unsafe 0.2 -0.2 0)
    (send *pr2* :translate #f(200 -200 0))


;;----------------------------------------------------------------
;;    find out chair

    (ros::ros-info "start detect chair")
    (start-detect-chair)
    (ros::ros-info "end detect chair")
;;----------------------------------------------------------------

    ;;set cloth coord
    (send *pr2* :larm :angle-vector #f(40.0 -20.0 0.0 -70.0 70.0 -90.0 0.0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (setq av-seq nil)
    (setq croth-coords
          (make-coords
           :pos
           (midpoint 0.5
                     (send (send *chair* :handle-above-cloth-larm) :worldpos)
                     (send (send *chair* :handle-above-cloth-rarm) :worldpos))
           :rot
           (rotate-matrix (send (send *chair* :handle-above-cloth-larm) :worldrot) -pi/2 :x)))

    ;look-at-cloth
    (send *pr2* :head :look-at (send croth-coords :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (if (boundp '*irtviewer*) (send croth-coords :draw-on :flush t))

    (send *pr2* :larm :inverse-kinematics
          croth-coords
          :revert-if-fail nil
          :look-at-target t)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (push (send *pr2* :angle-vector) av-seq)

    (send *pr2* :larm :move-end-pos #f(-100 0 0))
    (push (send *pr2* :angle-vector) av-seq)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :angle-vector-sequence av-seq (list 3000 1000))
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :angle-vector))

    (let (grasp-width)
      (send *ri* :start-grasp :larm :objects nil) ;; wait motion and grasp
      (setq grasp-width (send *ri* :start-grasp :larm :objects nil)) ;; wait motion and grasp ;; make sure that we really grasp cloth
      ;; if pr2 fail to grasp the cloth, return nil.
      (when (and (send *ri* :simulation-modep) (< grasp-width 1.1))
	(send *ri* :go-pos-unsafe -0.2 0 0)
	(send *pr2* :translate #f(-200 0 0))
	(return-from pick-cloth-chair nil)

        )
      )
    ;;lift up
    (send *pr2* :larm :angle-vector #f(20.0 -20.0 0.0 -70.0 70.0 -90.0 0.0))
    (send *pr2* :look-at-hand :larm)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :start-grasp :larm :objects nil)
    (send *ri* :move-gripper :larm 0.0 :effort 50 :wait t)
    (unix:sleep 1)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (unix:sleep 1)
;    (send *ri* :go-pos-unsafe -0.4 0 0)
;    (send *pr2* :translate #f(-400 0 0))
    (send *ri* :go-pos-unsafe -0.2 0.2 0)
    (send *pr2* :translate #f(-200 200 0))
    (send *ri* :wait-interpolation)
    (send *pr2* :reset-pose)

    t ;; return t on success
    ))
