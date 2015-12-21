(defun handover-remote ()

  (setq remote-handover-pose (send (send *pr2* :copy-worldcoords) :translate #f(550 200 800)))
  (send *pr2* :larm :inverse-kinematics
        remote-handover-pose
        )  
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
  (send *ri* :wait-interpolation)

  (setq remote-handover-pose (send (send (send *pr2* :copy-worldcoords) :translate #f(550 200 800)) :rotate (/ -pi 2) #f(0 1 0)))
  (send *pr2* :larm :inverse-kinematics
        remote-handover-pose
        )  
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
  (send *ri* :wait-interpolation)
            
  ;; release object
  (send *ri* :stop-grasp :larm :wait t) 

  (send *pr2* :larm :move-end-pos #f(-100 0 100) )
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; move back from person
  (send *ri* :go-pos-unsafe -0.3 0 0)
  
  )
