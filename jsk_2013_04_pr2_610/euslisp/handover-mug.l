(defun handover-mug ()

  ;; move arm towards person
  (send *pr2* :rarm :move-end-pos #f(300 200 0))
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
          
  ;; release object
  (send *ri* :stop-grasp :rarm :wait t) 

  ;; retract arm
  (send *pr2* :rarm :move-end-pos #f(-300 -200 0))
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (pr2-reset-pose)
  
  ;; move back from person
  (send *ri* :go-pos-unsafe -0.3 0 -90)
  
  )
