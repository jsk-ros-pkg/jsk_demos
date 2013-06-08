(defun pick-broom ()
  (let ((count 0))
    (warn "(pick-broom)~%")

    ;; debug
    (warn "*pr2*: ~A~%" *pr2*)
    (warn "*laundry*: ~A~%" *laundry*)
    (warn "*broom*: ~A~%" *broom*)

    ;; move pr2 to pick-broom spot
    (send *ri* :stop-grasp :larm :wait nil)
    (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)
    (pr2-reset-pose)  ;; debug

    (setq count 0)
    (while (null (grasp-broom))
      (incf count)
      (move-to-broom)
      (if (> count 3) (return-from pick-broom nil)))
    (goback-broom)
    (bringup-broom)
))

(defun grasp-broom ()
  (warn "(grasp-broom)~%")
  (let (av0 av1 target-coords laundry-coords ret)
    ;; move robot position
    (send *pr2* :move-to (send *ri* :state :worldcoords *room610-origin*) :world)

    ;; move broom
    (send *pr2* :larm :end-coords :dissoc *broom*)
    (send *broom* :move-to (send (make-coords :pos (float-vector 650 -490 0)
                                              :rpy (list pi/2 (deg2rad 4) 0))
                                 :transform *laundry* :world)
          :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    (setq target-coords 
          (make-coords :pos (send (send *broom* :handle-larm) :worldpos)
                       :rot 
                       (send (send (send *laundry* :copy-worldcoords) :rotate pi :z) :worldrot)))

    (send *pr2* :reset-pose)
    (setq ret
          (send *pr2* :larm :inverse-kinematics
                target-coords
                :look-at-target t
                ;;          :rotation-axis t
                :rotation-axis t
                :debug-view nil))
    (when (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects)
      (send target-coords :draw-on :flush t :size 500))

    (unless ret
      (ros::ros-error "could not solve ik to grasp broom")
      (speak-en "Could not reach to the broom")
      (return-from grasp-broom nil))
	
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :larm :move-end-pos #f(-100 0 0)) ;; via point
    (setq av0 (send *pr2* :angle-vector))

    (send *ri* :angle-vector-sequence (list av0 av1) (list 1000 1500))
    (send *ri* :wait-interpolation)

    (when (< (send *ri* :start-grasp :larm) 3.0)
      (send *ri* :stop-grasp :larm :wait t)
      (speak-jp "ほうきとれなかった")
      (send *ri* :angle-vector av0)
      (send *ri* :stop-grasp :larm :wait nil)
      (return-from grasp-broom nil)
      )
    t))


(defun goback-broom ()
  (warn "go backward~%")
  (send *pr2* :larm :end-coords :assoc *broom*)
  (send *pr2* :torso :waist-z :joint-angle 100)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)


  (speak-jp "うしろへさがった")
  (send *pr2* :translate #f(-500 0 0))
  (send *ri* :go-pos-unsafe -0.5 0 0)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :end-coords :dissoc *broom*)
  )

(defun bringup-broom ()
  (warn "(bringup-broom)~%")
  (let (larm-rpy av-list)
  (send *pr2* :larm :end-coords :assoc *broom*)
  (setq larm-rpy
        (if (not (gripper-z-axis-inversed-p :larm))
            (list (deg2rad -70) 0 0)
          (list (deg2rad -70) 0 (deg2rad 180))))

  (warn "bring up~%")
  (push (send *pr2* :larm :inverse-kinematics
        (make-cascoords :pos (send *pr2* :transform-vector #f(530 0 1300))
                        :rpy larm-rpy)
        :look-at-target t
        :rotation-axis :z
        :debug-view nil) av-list)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  (warn "right-hand~%")
  (push (send *pr2* :rarm :inverse-kinematics
        (make-coords :pos (send *pr2* :transform-vector #f(530 60 650))
                     :rpy (list (deg2rad 70) 0 0))
        :rotation-axis :z
;		:rotation-axis t
        :debug-view nil) av-list)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)

  (send *pr2* :torso :waist-z :joint-angle 0)
  (push (send *pr2* :angle-vector) av-list)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;  (send *ri* :angle-vector (send *pr2* :angle-vector) 1200)
;  (send *ri* :wait-interpolation)

  (send *ri* :angle-vector-sequence (reverse av-list) (list 1000 1500 1200))
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (send *pr2* :larm :end-coords :dissoc *broom*)
  (setq av-list nil)
  (warn "calib-pick-height~%")
  (let ((offset-l (send *pr2* :rotate-vector #f(0 -30 -500)))
        (offset-r #f(0 0 -500))
        (handle-larm (send (send *broom* :handle-larm) :copy-worldcoords))
        (handle-rarm (send (send *broom* :handle-rarm) :copy-worldcoords)))
    (print offset-l)
    (send *pr2* :larm :inverse-kinematics (send handle-larm :translate offset-l :world)
          :look-at-target t
          :rotation-axis :z
		  :rotation-axis t
          :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics (send handle-rarm :translate offset-r :world)
          :look-at-target nil
          :rotation-axis :z
		  :rotation-axis t
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
