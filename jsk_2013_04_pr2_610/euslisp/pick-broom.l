;;
(load "package://jsk_2013_04_pr2_610/euslisp/furushchev-utils.l")
;(load "models/room610-scene.l")
;(load "models/room610-laundary-machine-object.l")
;(load "models/room73b2-broom-object.l")

;  (setq *washer* (room610-laundary-machine))
;  (send *washer* :move-to (make-coords :pos #f(1220 280 0) :rpy (float-vector pi 0 0)) :world)
;  (objects (list *pr2* *broom* *washer* (room610)))
;(setq *broom* (room73b2-broom))
;(send *broom* :move-to (make-coords :pos #f(620 371 0) :rpy (float-vector pi 0 (deg2rad -5))) :world)
;(setq *room610* (room610))
;(objects (list *room610* *broom* *pr2*))
#|
(setq *broom-handle2* (make-cascoords :pos (float-vector 0.0 0.0 600.0)
									  :rot #2f((0.5 -0.866025 0.0) (0.866025 0.5 0.0) (0.0 0.0 1.0))
									  :name "handle2"))
						  (send self :assoc handle2))
						  handle2))
(send *broom* :assoc *broom-handle2*)
(defmethod room73b2-broom-object
  (:handle-handle2 () *broom-handle2*)
  (:handle () (list handle0 handle1 *broom-handle2*)))

(setq *pick-spot* (make-coords :pos #f(3400 1400 0) :rpy (list pi/2 0 0)))
(send *room610* :assoc *pick-spot*)
(defmethod room610-scene
  (:spots () (list *pick-spot*))
  (:spot-pick-spot () *pick-spot*))
|#

(defun pick-broom ()
  (warn "pick-broom (furushchev)~%")
  (setq debug t)

  (warn "reset-larm-rpy~%")
  (speak-jp "てをひろげる")
  (r2p)

  (send *ri* :stop-grasp) ; 手を広げる

  (speak-jp "ひだりて")
#|  (pr2-solve-ik :larm
				(make-coords :pos #f(550 200 1000)
;							 :rpy (list 0 0 (deg2rad -180)))
							 :rpy (list 0 0 0)
				:rotation-axis t
				:time 2000
				:debug-view debug)
|#

;  (pr2-reset-pose)

  (speak-jp "もっぷ")
;  (send *pr2* :inverse-kinematics (send (send *broom* :handle-handle0) :rotate -80 :z)
  (send *pr2* :inverse-kinematics (send (send *broom* :handle-larm) :rotate (deg2rad 170) :z)
		:link-list (list
					(send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
		:move-target (list (send *pr2* :larm :end-coords))
		:look-at-target t
		:debug-view debug)

  (if debug (break))
;  (setq *pick-potentio-vector* #f(50.1352 17.7895 -20.1068 84.3393 -45.8187 -78.6757 -5.72958 146.766 -60.0998 73.8901 -69.9999 -119.882 -19.999 -29.9744 179.986 0.002763 -1.302))
;  (send *pr2* :angle-vector *pick-potentio-vector*)
;  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (if debug (break))

  (warn "larm-start-grasp~%")
  (speak-jp "つかむ")
  (send *ri* :start-grasp :larm)

  (send (elt (send *pr2* :larm) 6) :assoc *broom*)

  (send *pr2* :torso :waist-z :joint-angle 200)
  (p2r)

  (if debug (break))

  (warn "go backward")
  (speak-jp "うしろへさがる")
  (send *pr2* :translate #f(-500 0 0))
  (send *ri* :go-pos-unsafe -0.5 0 0)

  (send *ri* :wait-interpolation)

  (if debug (break))

  (r2p)
  (setq z-axis-inversed-p
		(let ((r (send (send *pr2* :larm :end-coords) :rotate-vector #f(0 0 1))))
		  (if (> (elt r 2) 0) t nil)))

  (setq larm-rpy
		(if z-axis-inversed-p
			(list (deg2rad -70) 0 0)
		  (list (deg2rad -70) 0 (deg2rad 180))))

;  (send *ri* :stop-grasp :rarm)

  (setq pr2-pos (send (send *pr2* :coords) :pos))

  (warn "bring up")
  (speak-jp "もちあげる")
  (pr2-solve-ik :larm
				(make-cascoords :pos (v+ pr2-pos #f(530 0 1300))
								:rpy larm-rpy)
				:rotation-axis t
				:time 1000
				:debug-view debug)

#|  (send *pr2* :larm :move-end-pos #f(-100 0 150) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *ri* :wait-interpolation) |#

; (send *ri* :move-gripper :larm (* 60 0.001) :wait t)

  (if debug (break))

  (warn "right-hand~%")
  (speak-jp "みぎて")

  (pr2-solve-ik :rarm
				(make-coords :pos (v+ pr2-pos #f(530 20 650))
							 :rpy (list (deg2rad 70) 0 0))
				:rotation-axis t
				:time 1500
				:debug-view debug)
  
  (send *pr2* :torso :waist-z :joint-angle 0)
  (p2r)
  (send *ri* :wait-interpolation)
  (grasp-with-angle :arms 60)

  (send (elt (send *pr2* :larm) 6) :dissoc *broom*)

  (warn "calib-pick-height~%")
  (speak-jp "た'かさを,そろ'える")
  (if debug (break))
  (send *pr2* :inverse-kinematics (send *broom* :handle-larm) ; (send *broom* :handle-handle0)
		:link-list (list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
		:move-target (list (send *pr2* :larm :end-coords))
		:look-at-target t
		:debug-view debug)
  (send *pr2* :inverse-kinematics (send *broom* :handle-rarm) ;(send *broom* :handle-handle2)
		:link-list (list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
		:move-target (list (send *pr2* :rarm :end-coords))
		:look-at-target nil
		:debug-view debug)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)

#|  (pr2-solve-ik :larm
				(make-coords :pos (send (send *broom* :handle-handle1) :pos)
							 :rpy larm-rpy)
				:rotation-axis t
				:time 1500
				:debug-view debug)

  (pr2-solve-ik :rarm
				(make-coords :pos (send (send *broom* :handle-handle0) :pos)
							 :rpy (list (deg2rad 70) 0 0))
				:rotation-axis t
				:time 1500
				:debug-view debug)
|#
;  (send *ri* :start-grasp :larm)

#|
  (send *pr2* :larm :move-end-pos (float-vector 0 0 (if z-axis-inversed-p 200 -200)))
  (send *irtviewer* :draw-objects)
  (send *ri* :wait-interpolation)
  

  (send *ri* :start-grasp)

  (speak-jp)

  (send *pr2* :rarm :move-end-pos #f(0 0 200))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
|#
  ;for debug
  (speak-jp "もとにもどる")
  (send *ri* :stop-grasp)
  (send (elt (send *pr2* :larm) 6) :dissoc *broom*)
  (send *ri* :go-pos-unsafe 0.5 0)
  (send *ri* :angle-vector (send *pr2* :reset-pose) 4000)
  )
