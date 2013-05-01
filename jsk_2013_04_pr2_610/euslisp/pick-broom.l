(defun pick-broom (&key debug)
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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics (send (send *broom* :handle-larm) :rotate (deg2rad 170) :z)
          :look-at-target t
          :rotation-axis :z
          :debug-view debug)

    (if debug (break))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (send *pr2* :larm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)

    (if debug (break))

    (warn "larm-start-grasp~%")
    (speak-jp "つかむ")
    (send *ri* :start-grasp :larm)

    (send *pr2* :larm :end-coords :assoc *broom*)

    (send *pr2* :torso :waist-z :joint-angle 200)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)

    (if debug (break))

    (warn "go backward")
    (speak-jp "うしろへさがる")
    (send *pr2* :translate #f(-500 0 0))
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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

    (if debug (break))

    (warn "right-hand~%")
    (speak-jp "みぎて")

    (send *pr2* :rarm :inverse-kinematics
          (make-coords :pos (send *pr2* :transform-vector #f(530 100 650))
                       :rpy (list (deg2rad 70) 0 0))
          :rotation-axis :z
          :debug-view debug)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    
    (send *pr2* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
                                        ;  (grasp-with-angle :arms 60)

    (send *pr2* :larm :end-coords :dissoc *broom*)

    (warn "calib-pick-height~%")
    (speak-jp "'たかさを,そ'ろえる")
    (if debug (break))
    (send *pr2* :larm :inverse-kinematics (send *broom* :handle-larm)
          :look-at-target t
          :rotation-axis :z
          :debug-view debug)
    (send *pr2* :larm :inverse-kinematics (send *broom* :handle-rarm)
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
    ))
