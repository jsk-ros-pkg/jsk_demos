(defun pick-up-kitchen-knife ()


  ;; detect remote
  (if (not (send *ri* :simulation-modep))
      (kitchen-knife-detection *kitchen-knife*)
    )
  (setq kitchen-knife-pre-grasp-pose (send (send *kitchen-knife* :copy-worldcoords) :translate #f(0 0 300) :world)) 
  (setq kitchen-knife-pose (send *kitchen-knife* :copy-worldcoords))

  ;; pick up kitchen knife
  
  ;; Open gripper
  (send *ri* :stop-grasp :rarm) ;; stop-grasp wait until gripper motion stops   

  (dolist (hand-target (list kitchen-knife-pre-grasp-pose kitchen-knife-pose))
          (send *pr2* :rarm :inverse-kinematics
                hand-target
                :use-torso t
                :look-at-target t
                )
          
          (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
          (send *ri* :wait-interpolation)
          )

  (send *ri* :start-grasp :rarm) ;; grasp

  ;; pick it up
  (send *pr2* :rarm :move-end-pos #f(0 0 200) :world)
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  )

(defun kitchen-knife-detection (*kitchen-knife*)
  (print "kitchen-knife-detection")
  (print "STUB!! NEED TO BE IMPLEMENTED")
  )
