#!/usr/bin/env roseus
(load "furushchev-utils.l")

(setq banzai-potentio-vector #f(50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

(setq *room610-origin* "/eng8/6f/room610")
(setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
(setq *chair-coords* (make-coords :pos #f(3800 0 0)))

(defun move-to-table ()
  (warn "move-to-table~%")
  (pr2-tuckarm-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :move-to *table-coords* :frame-id *room610-origin*)
  )

(defun pick-tray ()
  (warn "pick-tray (tanaka)~%")
  )

(defun move-to-sink ()
  (warn "move-to-sink~%")
  )

(defun place-tray ()
  (warn "place-tray (chan)~%")
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  (pr2-tuckarm-pose)
  (send *ri* :move-to *chair-coords* :frame-id *room610-origin*)
  )

(defun pick-cloth ()
  (warn "pick-cloth (iwaishi)~%")
  )

(defun move-to-laundary ()
  (warn "move-to-laundary~%")
  )

(defun open-laundary-door ()
  (warn "open-laundary-door~%")
  )

(defun put-cloth ()
  (warn "put-cloth (kuroiwa)~%")
  )

(defun close-laundry-door ()
  (warn "close-laundry-door (kuroiwa)~%")
  )

(defun pick-broom ()
  (warn "pick-broom (furushchev)~%")
  (warn "reset-larm-rpy~%")
  (speak-jp "てをひろげる")
  (r2p)

  (send *ri* :stop-grasp :larm) ; 手を広げる

  (speak-jp "ひだりて")
  (pr2-solve-ik :larm
				(make-coords :pos #f(550 200 1000)
							 :rpy (list 0 0 (deg2rad -180)))
				:rotation-axis t
				:time 2000)
 
  (speak-jp "もっぷ")
  (setq *pick-potentio-vector* #f(50.1352 17.7895 -20.1068 84.3393 -45.8187 -78.6757 -5.72958 146.766 -60.0998 73.8901 -69.9999 -119.882 -19.999 -29.9744 179.986 0.002763 -1.302))
  (send *pr2* :angle-vector *pick-potentio-vector*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "larm-start-grasp~%")
  (speak-jp "つかむ")
  (send *ri* :start-grasp :larm)

  (speak-jp "さがる")
  (unix:sleep 3)
  (send *ri* :go-pos-unsafe -0.5 0)
  (send *ri* :wait-interpolation)

  (r2p)
  (setq z-axis-inversed-p
		(let ((r (send (send *pr2* :larm :end-coords) :rotate-vector #f(0 0 1))))
		  (if (> (elt r 2) 0) t nil)))

  (setq larm-rpy
		(if z-axis-inversed-p
			(list (deg2rad -70) 0 0)
		  (list (deg2rad -70) 0 (deg2rad 180))))

  (send *ri* :stop-grasp :rarm)

  (speak-jp "もちあげる")
  (pr2-solve-ik :larm
				(make-coords :pos #f(530 0 1300)
							 :rpy larm-rpy)
				:rotation-axis t
				:time 1000)

#|  (send *pr2* :larm :move-end-pos #f(-100 0 150) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *ri* :wait-interpolation) |#

; (send *ri* :move-gripper :larm (* 60 0.001) :wait t)
  (grasp-with-angle :larm 60)

  (warn "calib-pick-height~%")
  (speak-en "calib-pick-height")

  (pr2-solve-ik :larm
				(make-coords :pos #f(530 0 1000)
							 :rpy larm-rpy)
				:rotation-axis t
				:time 1500)

  (send *ri* :start-grasp :larm)

  (send *pr2* :larm :move-end-pos (float-vector 0 0 (if z-axis-inversed-p 200 -200)))
  (send *ri* :wait-interpolation)
  
  (speak-en "move-right-arm")

  (pr2-solve-ik :rarm
				(make-coords :pos #f(530 20 650)
							 :rpy (list (deg2rad 70) 0 0))
				:rotation-axis t
				:time 1500)

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
  ;; show chair on objects
  ;; TODO update chair position using perception pipeline,
  ;;   load and instantiate chair model at setup function, 
  ;;   start/stop perception piplen in move-chair function
  (load "models/red-chair0-object.l") ;; need to use karimoku chair
  (setq *chair* (red-chair0))
  (send *chair* :move-to (make-coords :pos #f(800 0 0) :rpy (float-vector pi/2 0 0)) :world)
  (objects (list *pr2* *chair*))
  ;; use handle to manipulate
  (send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
  (send *pr2* :inverse-kinematics (send *chair* :handle)
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :larm :end-coords) (send *pr2* :rarm :end-coords))
        :look-at-target t
        )
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :start-grasp)
  ;; backword
  (send *ri* :go-pos-unsafe -0.5 0 30)
  (send *ri* :stop-grasp)
  )

(defun sweep-under-table ()
  (warn "sweep-under-table~%")
  )

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun sweep-floor ()
  (warn "sweep-floor (furuta)~%")
  )


(defun demo ()
  (move-to-table)
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