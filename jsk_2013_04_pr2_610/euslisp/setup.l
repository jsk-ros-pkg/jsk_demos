#!/usr/bin/env roseus
(load "furushchev-utils.l")

(setq banzai-potentio-vector #f(50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

(defun pick-tray ()
  (warn "pick-tray~%")
  )

(defun move-to-sink ()
  (warn "move-to-sink~%")
  )

(defun place-tray ()
  (warn "place-tray~%")
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  )

(defun pick-cloth ()
  (warn "pick-cloth~%")
  )

(defun move-to-laundary ()
  (warn "move-to-laundary~%")
  )

(defun open-laundary-door ()
  (warn "open-laundary-door~%")
  )

(defun put-cloth ()
  (warn "put-cloth~%")
  )

(defun close-laundary-door ()
  (warn "close-laundary-door~%")
  )

(defun pick-broom ()
  (warn "pick-broom~%")
  (speak-jp "もっぷ")
  (warn "reset-larm-rpy~%")
  (speak-jp "てをひろげる")
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (send *ri* :stop-grasp :larm) ; 手を広げる

  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(550 200 1000) :rpy (float-vector 0 0 (deg2rad -180)))
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq *pick-potentio-vector* #f(50.1352 17.7895 -20.1068 84.3393 -45.8187 -78.6757 -5.72958 146.766 -60.0998 73.8901 -69.9999 -119.882 -19.999 -29.9744 179.986 0.002763 -1.302))
  (send *pr2* :angle-vector *pick-potentio-vector*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "larm-start-grasp~%")
  (speak-en "larm-start-grasp")
  (send *ri* :start-grasp :larm) ;; TODO: 微妙ににぎる

  (send *ri* :go-pos-unsafe -0.5 0)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq z-axis-inversed-p
		(let ((r (send (send *pr2* :larm :end-coords) :rotate-vector #f(0 0 1))))
		  (if (> (elt r 2) 0) t nil)))

  (setq larm-rpy
		(if z-axis-inversed-p
			(list (deg2rad -70) 0 0)
		  (list (deg2rad -70) 0 (deg2rad 180))))

  (send *ri* :stop-grasp :rarm)
  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(630 0 1200) :rpy larm-rpy)
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(-100 0 0)
		:world
		:rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

 (send *ri* :move-gripper :larm (* 60 0.001) :wait nil)
 
  (warn "calib-pick-height~%")
  (speak-en "calib-pick-height")
  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(530 0 1000) :rpy larm-rpy)
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)

  (send *pr2* :larm :move-end-pos (float-vector 0 0 (if z-axis-inversed-p 200 -200)))
  (send *ri* :wait-interpolation)
  
  (speak-en "move-right-arm")
  (send *pr2* :rarm :inverse-kinematics
		(make-coords :pos #f(530 20 650) :rpy (list (deg2rad 70) 0 0))
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
;  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (send *ri* :start-grasp)

  (speak-en "move-3-centimeter-up")

  (send *pr2* :rarm :move-end-pos #f(0 0 200))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;for debug
  (speak-jp "もとにもどる")
  (send *ri* :stop-grasp)
  (send *ri* :go-pos-unsafe 0.5 0)
  (send *ri* :angle-vector (send *pr2* :reset-pose) 4000)
  )

(defun move-chair ()
  (warn "move-chair~%")
  )

(defun sweep-under-table ()
  (warn "sweep-under-table~%")
  )

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun sweep-floor ()
  (warn "sweep-floor~%")
  )


(defun demo ()
  (pick-tray)
  (move-to-sink)
  (place-tray)
  (move-to-chair)
  (pick-cloth)
  (move-to-laundary)
  (open-laundary-door)
  (put-cloth)
  (close-laundry-door)
  (pick-broom)
  (move-chair)
  (sweep-under-table)
  (sweep-floor)
  )

(warn "type (demo) to start~%")