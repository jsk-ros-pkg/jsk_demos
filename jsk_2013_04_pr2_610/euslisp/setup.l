#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://roseus/euslisp/roseus-utils.l")

(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

(objects (list *room610* *pr2*))

(setq banzai-potentio-vector #f(50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

;; category of pr2-interface.l
;; クロージャ使いたい・・・
(defun grasp-with-angle (robot (&optional (arm :arms) &key ((:angle a) 0)))
  (send robot :move-gripper arm (* a 0.001) :wait t)
  ;; TODO: robotのviewerにアクセスして*pr2*と同期したい
  )


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


(setq pick-broom-pos (make-coords :pos #f(-26629.619 19523.138 1.559) :rpy (list 1.876 -0.005 -0.001)))
  (setq pick-potentio-vector (float-vector 50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

(defun pick-broom ()
  (warn "pick-broom~%")
;  (ros::advertise "robotsound_jp")
  (speak-en "pick-broom")
;  (send *ri* :angle-vector banzai-potentio-vector 2000)
; (unix:sleep 5)
 ; (setq prev-angle (send *pr2* :angle-vector)) ;; TODO: 前の動作からの姿勢引き継ぎ

  (warn "reset-larm-rpy~%")
  (speak-en "reset-larm-rpy")
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos (send (send *pr2* :larm :end-coords) :worldpos)
					 :rpy (list 0 0 0))
		:rotation-axis t
		:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (unix:sleep 3)

  (send *ri* :stop-grasp :larm) ; 手を広げる

  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(700 0 1100))
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (unix:sleep 2)
  (warn "larm-start-grasp~%")
  (speak-en "larm-start-grasp")
  (send *ri* :start-grasp :larm) ;; TODO: 微妙ににぎる

  (send *ri* :stop-grasp :rarm)
  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(630 0 1200))
		:rotation-axis nil
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (unix:sleep 2)
;  (grasp-with-angle *ri* :rarm :angle 6)
  (send *ri* :move-gripper :larm (* 60 0.001) :wait nil)
  (send *pr2* :larm :move-end-pos #f(-50 0 0)
		:world
		:rotation-axis nil)
  (send *ri* :start-grasp :larm)

  (warn "calib-pick-height~%")
  (speak-en "calib-pick-height")
  (send *pr2* :larm :inverse-kinematics
		(make-coords :pos #f(580 0 850))
		:rotation-axis nil
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (unix:sleep 3)

  (speak-en "move-right-arm")
  (send *pr2* :rarm :inverse-kinematics
		(make-coords :pos #f(580 0 650) :rpy (list 70 0 0))
		:rotation-axis t
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)

  (unix:sleep 3)
  (send *ri* :start-grasp)
  (unix:sleep 5)
  (speak-en "move-3-centimeter-up")
  (send *pr2* :arms :move-end-pos #f(0 0 200))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  ;for debug
  (speak-jp "もとにもどる")
  (unix:sleep 10)
  (send *ri* :stop-grasp)
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

;; utility
(defun sync-ri-to-pr2 ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
(defun sync-pr2-to-ri ()
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000))


(warn "type (demo) to start~%")
