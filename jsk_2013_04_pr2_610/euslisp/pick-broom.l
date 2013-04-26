(load "package://jsk_2013_04_pr2_610/euslisp/furushchev-utils.l")

(defun move-to-pick-broom-spot ()
  (warn "move-to-pick-broom-spot~%")
  (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)
;  (send *pr2* :translate #f(0 300 0) (send *room610* :worldcoords))
;  (send *ri* :move-to (send *room610* :spot-pick-broom-spot) :frame-id *room610-origin*)
  (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
  (setq mou-chotto-pos (v- (send (send *room610* :spot-pick-broom-spot) :pos) (send current-ri-coords :pos)))
  (setq mou-chotto-local-pos (transform (inverse-matrix (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
  (setq mou-chotto-roll
		(- 
		 (elt (elt (send (send *room610* :spot-pick-broom-spot) :rpy-angle) 0) 0)
		 (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
  (send *ri* :go-pos-unsafe (/ (elt mou-chotto-local-pos 0) 1000) (/ (elt mou-chotto-local-pos 1) 1000) (rad2deg mou-chotto-roll))
)

(defun pick-broom ()
  (warn "pick-broom (furushchev)~%")
  (setq debug nil)

  (warn "reset-larm-rpy~%")
  (speak-jp "てをひろげる")
  (r2p)

  (send *ri* :stop-grasp) ; 手を広げる
  (pr2-reset-pose)
  (speak-jp "ひだりて")

  (speak-jp "もっぷ")

  (send *pr2* :inverse-kinematics
		(make-coords :pos (send *pr2* :transform-vector #f(300 200 1000))
					 :rpy (list 0 0 0))
		:link-list (list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
		:move-target (list (send *pr2* :larm :end-coords))
		:look-at-target t
		:rotation-axis nil
		:debug-view debug)
  (p2r)
  (send *ri* :wait-interpolation)
  (send *pr2* :inverse-kinematics (send (send *broom* :handle-larm) :rotate (deg2rad 170) :z)
		:link-list (list
					(send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
		:move-target (list (send *pr2* :larm :end-coords))
		:look-at-target t
		:rotation-axis :z
		:debug-view debug)

  (if debug (break))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(100 0 0))
  (p2r)
  (send *ri* :wait-interpolation)

  (if debug (break))

  (warn "larm-start-grasp~%")
  (speak-jp "つかむ")
  (send *ri* :start-grasp :larm)

  (send (send *pr2* :l_wrist_roll_link) :assoc *broom*)

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

  (setq pr2-pos (send (send *pr2* :coords) :pos))

  (warn "bring up")
  (speak-jp "もちあげる")
  (pr2-solve-ik :larm
				(make-cascoords :pos (send *pr2* :transform-vector #f(530 0 1300))
								:rpy larm-rpy)
				:rotation-axis :z
				:time 1000
				:debug-view debug)

  (if debug (break))

  (warn "right-hand~%")
  (speak-jp "みぎて")

  (pr2-solve-ik :rarm
				(make-coords :pos (send *pr2* :transform-vector #f(530 100 650))
							 :rpy (list (deg2rad 70) 0 0))
				:rotation-axis :z
				:time 1500
				:debug-view debug)
  
  (send *pr2* :torso :waist-z :joint-angle 0)
  (p2r)
  (send *ri* :wait-interpolation)
;  (grasp-with-angle :arms 60)

  (send (send *pr2* :l_wrist_roll_link) :dissoc *broom*)

  (warn "calib-pick-height~%")
  (speak-jp "'たかさを,そ'ろえる")
  (if debug (break))
  (send *pr2* :inverse-kinematics (send *broom* :handle-larm)
		:link-list (list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
		:move-target (list (send *pr2* :larm :end-coords))
		:look-at-target t
		:rotation-axis :z
		:debug-view debug)
  (send *pr2* :inverse-kinematics (send *broom* :handle-rarm)
		:link-list (list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
		:move-target (list (send *pr2* :rarm :end-coords))
		:look-at-target nil
		:rotation-axis :z
		:debug-view debug)

  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :move-end-pos #f(80 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)

  (send *pr2* :torso :waist-z :joint-angle 100)

  ;for debug
  (when debug
  (speak-jp "もとにもどる")
  (send *ri* :stop-grasp)
  (send (elt (send *pr2* :larm) 6) :dissoc *broom*)
  (send *ri* :go-pos-unsafe 0.5 0)
  (send *ri* :angle-vector (send *pr2* :reset-pose) 4000))
  )
