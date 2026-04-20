(defun handover-remote ()

  (setq remote-handover-pose (send (send *pr2* :copy-worldcoords) :translate #f(700 200 800))) ;; handover pose intermediate point (otherwise pr2's hand move in weird trajectory)
  (send *pr2* :larm :inverse-kinematics
        remote-handover-pose
        )  
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
  (send *ri* :wait-interpolation)

  (setq remote-handover-pose (send (send (send *pr2* :copy-worldcoords) :translate #f(700 200 800)) :rotate (/ -pi 2) #f(0 1 0))) ;; handover pose
  (send *pr2* :larm :inverse-kinematics
        remote-handover-pose
        )  
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
  (send *ri* :wait-interpolation)

  (unix::sleep 3)
  ;; release object
  (send *ri* :stop-grasp :larm) 

  ;; retract hand
  (send *pr2* :larm :move-end-pos #f(-100 0 100) )
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; move back from person
  (send *ri* :go-pos-unsafe -0.3 0 0)

  ;; put arm in read position for picking up laundry to avoid collision while navigating
  (send *pr2* :angle-vector #f(50.1262 0.0 -10.0 40.0 -70.0 70.0 -90.0 90.0 -5.0 70.0 -105.0 -90.0 70.0 -6.0 20.0 0.007004 -1.914))
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  )
