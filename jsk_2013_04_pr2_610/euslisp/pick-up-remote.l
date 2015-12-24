(defun pick-up-remote ()


  ;; detect remote
  (if (not (send *ri* :simulation-modep))
      (remote-detection *remote*)
    )
  (setq remote-pre-grasp-pose (send (send *remote* :copy-worldcoords) :translate #f(0 0 300) :world)) 
  (setq remote-pose (send *remote* :copy-worldcoords))

  ;; pick up remote
  
  ;; Open gripper
  (send *ri* :stop-grasp :larm) ;; stop-grasp wait until gripper motion stops   

  (dolist (hand-target (list remote-pre-grasp-pose remote-pose))
          (send *pr2* :larm :inverse-kinematics
                hand-target
                :look-at-target t
                )
          
          (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
          (send *ri* :wait-interpolation)
          )

  (send *ri* :start-grasp :larm) ;; grasp

  ;; pick it up
  (send *pr2* :larm :move-end-pos #f(0 0 100) :world)
  (send *pr2* :look-at-hand :larm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; move back from table
  (send *ri* :go-pos-unsafe -0.4 0 90)
  )

(defun remote-detection (*remote*)
  (print "remote-detection")
  (print "STUB!! NEED TO BE IMPLEMENTED")
  )