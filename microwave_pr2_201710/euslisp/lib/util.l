(defun wait-enter
    ()
  (do-until-key
      (send *irtviewer* :draw-objects :flush nil)
    (x::window-main-one)
    (send (send *irtviewer* :viewer) :flush))
  )
