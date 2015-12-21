(defun pick-up-mug ()


  (pr2-reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; detect mug
  (if (not (send *ri* :simulation-modep))
      (mug-detection *mug*)
    )
  (setq mug-pre-grasp-pose (send (send *mug* :copy-worldcoords) :translate #f(-300 0 0))) 
  (setq mug-pose (send *mug* :copy-worldcoords))

  ;; pick up mug

  ;; Open gripper
  (send *ri* :stop-grasp :rarm) ;; stop-grasp wait until gripper motion stops   

  (dolist (hand-target (list mug-pre-grasp-pose mug-pose))
          (send *pr2* :rarm :inverse-kinematics
                hand-target
                :look-at-target t
                )
          
          (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
          (send *ri* :wait-interpolation)
          )

  (send *ri* :start-grasp :rarm) ;; grasp mug

  ;; pick it up
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :move-end-pos #f(-300 0 0))
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :move-end-pos #f(0 0 -100) :world)
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; move back from table
  (send *ri* :go-pos-unsafe -0.3 0 90)
  (send *ri* :go-pos-unsafe 0.3 0 0)
  )

(defun mug-detection (*mug*)
  (print "mug-detection")
  (print "STUB!! NEED TO BE IMPLEMENTED")
  )
