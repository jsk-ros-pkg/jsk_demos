(defun wesleyc-place-tray ()

	;;;
	;;; place_tray function
	;;; 
	;;; Before this function is called, the robot is assumed to have the tray gripped in both grippers
	;;; and the tray is posiitoned directly above the table.
	;;; 
	;;;

	(setq tableHeight 790)

	;; get current gripper pose
	(setq lgripperPose (send *pr2* :larm :end-coords))
	(setq rgripperPose (send *pr2* :rarm :end-coords))
	(setq lgripperPos (send lgripperPose :worldpos))
	(setq rgripperPos (send rgripperPose :worldpos))

	(setq lgripperPosCopy (copy-object lgripperPos)) ;; make copy from reference? -- will not work otherwise.
	(setq rgripperPosCopy (copy-object rgripperPos)) ;; make copy from reference? -- will not work otherwise.

	;; lower arms
	(setf (elt lgripperPosCopy 2) tableHeight)
	(setf (elt rgripperPosCopy 2) tableHeight)

	(moveLarmTo lgripperPosCopy)
	(moveRarmTo rgripperPosCopy)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	(unix:sleep 14)

	;; open grippers
	(send *ri* :stop-grasp :rarm)
	(send *ri* :stop-grasp :larm)
	(unix:sleep 3)

	;; move arms apart
	(setf (elt lgripperPosCopy 2) (+ (elt lgripperPosCopy 2) 50))
	(setf (elt rgripperPosCopy 2) (+ (elt rgripperPosCopy 2) 50))

	(moveLarmTo lgripperPosCopy)
	(moveRarmTo rgripperPosCopy)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 100)
	(unix:sleep 3))
