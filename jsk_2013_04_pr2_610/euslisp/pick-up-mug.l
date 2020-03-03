(defun pick-up-mug ()


  (pr2-reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  

  ;; get mug pos by detecting tray location
  (if (not (send *ri* :simulation-modep))
     (progn
       (send *pr2* :head :neck-p :joint-angle 40)
       (send *pr2* :head :neck-y :joint-angle -40)
       (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)                  
       (send *ri* :wait-interpolation)
       (tray-detection *tray*)
       (setq tray-larm-handle (send (send (send *tray* :handle-larm-handle) :copy-worldcoords) :rotate (deg2rad -20) :z) ;; do not move in world coords, use object(tray) relative coords
	     tray-rarm-handle (send (send (send *tray* :handle-rarm-handle) :copy-worldcoords) :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory
       (setq tray-pos (midpoint 0.5 (send tray-larm-handle :worldpos) (send tray-rarm-handle :worldpos)))
       (send *mug* :locate (v+ tray-pos #f(20 60 80)) :world) 

       )
    )

  ;; (send *mug* :locate #f(5050.0 100.0 800.0) :world)
  (setq mug-pre-grasp-pose (send (send *mug* :copy-worldcoords) :translate #f(-200 0 0))) 
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
          (send *ri* :angle-vector (send *pr2* :angle-vector) 2000) 
          (send *ri* :wait-interpolation)
          )

  (send *ri* :start-grasp :rarm) ;; grasp mug

  ;; pick it up
  (send *pr2* :rarm :move-end-pos #f(0 0 100) :world)
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :move-end-pos #f(-200 0 0))
  (send *pr2* :look-at-hand :rarm)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
   
  ;; move back from table
  (send *ri* :go-pos-unsafe -0.3 0 90)
  (send *ri* :go-pos-unsafe 0.3 0 0)
  )
