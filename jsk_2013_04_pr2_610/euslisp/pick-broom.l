(defun pick-broom ()
  (warn "(pick-broom)~%")

  ;; move broom
  (send *broom* :move-to (send (make-coords :pos #f(650 -390 0)
                                            :rpy (list pi/2 (deg2rad 4) 0))
                               :transform *laundry* :world)
        :world)

  ;; move pr2 to pick-broom spot
  (pr2-reset-pose)  ;; debug
  (send *ri* :stop-grasp :larm :wait nil)
  (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)

  (grasp-broom)
  (goback-broom)
  (bringup-broom)
)

(defun grasp-broom ()
  (warn "(grasp-broom)~%")
  (let ((c (send (send *broom* :handle-larm) :copy-worldcoords))
        (laundry-coords (send *laundry* :copy-worldcoords)))
    (send c :translate #f(100 0 0) *laundry*)
    (send *pr2* :larm :inverse-kinematics
          (make-coords :pos (send c :worldpos)
                       :rpy (elt (send (send laundry-coords :rotate pi :z) :rpy-angle) 0))
          :look-at-target t
          :rotation-axis :z
          :debug-view t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send *pr2* :larm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

	(send *ri* :start-grasp :larm)
	(send *pr2* :larm :end-coords :assoc *broom*)
    )
  )


(defun goback-broom ()
  (warn "go backward~%")
  (send *pr2* :torso :waist-z :joint-angle 100)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)


  (speak-jp "うしろへさがる")
  (send *pr2* :translate #f(-500 0 0))
  (send *ri* :go-pos-unsafe -0.5 0 0)
  (send *ri* :wait-interpolation)
  )

(defun bringup-broom ()
  (warn "(bringup-broom)~%")
  (let (larm-rpy)
  (setq larm-rpy
        (if (gripper-z-axis-inversed-p :larm)
            (list (deg2rad -70) 0 0)
          (list (deg2rad -70) 0 (deg2rad 180))))

  (warn "bring up")
  (send *pr2* :larm :inverse-kinematics
        (make-cascoords :pos (send *pr2* :transform-vector #f(530 0 1300))
                        :rpy larm-rpy)
        :look-at-target t
        :rotation-axis :z
        :debug-view nil)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  (warn "right-hand~%")
  (send *pr2* :rarm :inverse-kinematics
        (make-coords :pos (send *pr2* :transform-vector #f(530 60 650))
                     :rpy (list (deg2rad 70) 0 0))
        :rotation-axis :z
        :debug-view nil)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)

  (send *pr2* :torso :waist-z :joint-angle 0)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (send *pr2* :larm :end-coords :dissoc *broom*)

  (warn "calib-pick-height~%")
  (let ((offset-l (send *pr2* :rotate-vector #f(0 -30 -500)))
        (offset-r #f(0 0 -500))
        (handle-larm (send (send *broom* :handle-larm) :copy-worldcoords))
        (handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)))
    (print offset-l)
    (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l :world)
          :look-at-target t
          :rotation-axis :z
          :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r :world)
          :look-at-target nil
          :rotation-axis :z
          :debug-view nil))
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp)
  (send *pr2* :larm :end-coords :assoc *broom*)

  (send *pr2* :arms :move-end-pos (send *pr2* :rotate-vector #f(0 0 400)))
  (send *pr2* :torso :waist-z :joint-angle 100)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  )
)

(defun pick-broom-old (&key debug)
  (warn "pick-broom (furushchev)~%")
  (let (larm-rpy)
    (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)

    (warn "reset-larm-rpy~%")
    (speak-jp "てをひろげる")
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (send *ri* :stop-grasp) ; 手を広げる
    (pr2-reset-pose)
    (speak-jp "ひだりて")

    (speak-jp "もっぷ")

    (send *pr2* :larm :inverse-kinematics
          (make-coords :pos (send *pr2* :transform-vector #f(300 200 1000))
                       :rpy (list 0 0 0))
          :look-at-target t
          :rotation-axis nil
          :debug-view debug)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics (send (send *broom* :handle-larm) :rotate (deg2rad 170) :z)
          :look-at-target t
          :rotation-axis :z
          :debug-view debug)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (if debug (break))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *pr2* :larm :move-end-pos #f(100 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)

    (if debug (break))

    (warn "larm-start-grasp~%")
    (speak-jp "つかむ")
    (send *ri* :start-grasp :larm)

    (send *pr2* :larm :end-coords :assoc *broom*)

    (send *pr2* :torso :waist-z :joint-angle 100)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)

    (if debug (break))

    (warn "go backward")
    (speak-jp "うしろへさがる")
    (send *pr2* :translate #f(-500 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :go-pos-unsafe -0.5 0 0)

    (send *ri* :wait-interpolation)

    (if debug (break))

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (setq larm-rpy
          (if (gripper-z-axis-inversed-p :larm)
              (list (deg2rad -70) 0 0)
            (list (deg2rad -70) 0 (deg2rad 180))))

    (warn "bring up")
    (speak-jp "もちあげる")
    (send *pr2* :larm :inverse-kinematics
          (make-cascoords :pos (send *pr2* :transform-vector #f(530 0 1300))
                          :rpy larm-rpy)
          :look-at-target t
          :rotation-axis :z
          :debug-view debug)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

    (if debug (break))

    (warn "right-hand~%")
    (speak-jp "みぎて")
    
    (send *pr2* :rarm :inverse-kinematics
          (make-coords :pos (send *pr2* :transform-vector #f(530 60 650))
                       :rpy (list (deg2rad 70) 0 0))
          :rotation-axis :z
          :debug-view debug)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    
    (send *pr2* :torso :waist-z :joint-angle 0)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm)
    (send *ri* :stop-grasp :larm)
    (send *pr2* :larm :end-coords :dissoc *broom*)

    (warn "calib-pick-height~%")
    (speak-jp "'たかさを,そ'ろえる")
    (if debug (break))
    (let ((offset-l (send *pr2* :rotate-vector #f(0 -30 -500)))
          (offset-r #f(0 0 -500))
          (handle-larm (send (send *broom* :handle-larm) :copy-worldcoords))
          (handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)))
      (print offset-l)
      (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l :world)
            :look-at-target t
            :rotation-axis :z
            :debug-view debug)
      (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r :world)
            :look-at-target nil
            :rotation-axis :z
            :debug-view debug))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
                                        ;    (send *pr2* :rarm :move-end-pos #f(20 0 0))
                                        ;    (send *ri* :angle-vector (send *pr2* :angle-vector))
                                        ;    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)
    (send *pr2* :larm :end-coords :assoc *broom*)

;    (send *pr2* :torso :waist-z :joint-angle 100)
#|    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos (send *pr2* :rotate-vector #f(0 0 -230)))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm)
|#
    (send *pr2* :arms :move-end-pos (send *pr2* :rotate-vector #f(0 0 400)))
    (send *pr2* :torso :waist-z :joint-angle 100)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

                                        ;for debug
    (when debug
      (speak-jp "もとにもどる")
      (send *ri* :stop-grasp)
      (send (elt (send *pr2* :larm) 6) :dissoc *broom*)
      (send *ri* :go-pos-unsafe 0.5 0)
      (send *ri* :angle-vector (send *pr2* :reset-pose) 4000))
    ))
