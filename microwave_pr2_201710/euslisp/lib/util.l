(defun wait-enter
    ()
  (do-until-key
   (send *irtviewer* :draw-objects :flush nil)
   (x::window-main-one)
   (send (send *irtviewer* :viewer) :flush))
  )

(defun pr2-right-reset-pose
    ()
  (send *pr2* :rarm :angle-vector #f(-60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0))
  )

(defun pr2-left-reset-pose
    ()
  (send *pr2* :larm :angle-vector #f(60.0 74.0 70.0 -120.0 20.0 -30.0 180.0))
  )
