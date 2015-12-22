(defun handover-kitchen-knife ()

  (setq handover-orientation (make-coords :rpy #f(1.011639 0.022894 0.075669))) ;; handover orientation learend from demonstration
  (setq object-to-hand-transform (send (make-coords :rpy (float-vector 0 (/ pi 2) 0)) :rotate (/ pi 4) #f(1 0 0))) ;; trasnformation from object coordinate to hand frame 
  ;; handover location and facing direction of receiver
  (setq kitchen-knife-handover-pose (send (send (send *pr2* :copy-worldcoords) :translate #f(600 -400 900)) :rotate (/ pi 2) #f(0 0 1)))
  ;; (setq handover-pose (send kitchen-knife-handover-pose :transform handover-orientation))
  (setq hand-pose (send (send kitchen-knife-handover-pose :transform handover-orientation) :transform object-to-hand-transform))
  (send *pr2* :rarm :inverse-kinematics
        hand-pose
        )
  (send *pr2* :head :look-at (send *person-at-kitchen* :copy-pos))

  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) 
  (send *ri* :wait-interpolation)

  ;; release object
  (send *ri* :stop-grasp :rarm :wait t) 

  (send *pr2* :rarm :move-end-pos #f(-100 100 -100) :world)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  ;; move back from person
  (send *ri* :go-pos-unsafe -0.3 0.3 0)
  (pr2-reset-pose)
  (pr2-tuckarm-pose :arms)
  )
