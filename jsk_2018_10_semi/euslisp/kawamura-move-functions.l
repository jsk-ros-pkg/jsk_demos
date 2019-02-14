(defun goto-tableside()
  (send *ri* :move-to (send (send (send *eng2-scene* :spot "/eng2/7f/room73B2-table-side1") :copy-worldcoords)
			       :transform (make-coords :pos #f(-300 -130 0) :rpy (float-vector -1.2 0 0))))
  (send *fetch* :move-to (send (send (send *eng2-scene* :spot "/eng2/7f/room73B2-table-side1") :copy-worldcoords)
			       :transform (make-coords :pos #f(-300 0 0) :rpy (float-vector -0.7 0 0))) :world)
  (send *irtviewer* :look-all *fetch*)
  )

;;(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :cop0y-worldcoords) :transform (make-coords :pos #f(600 0 500))) :debug-view t)

(defun goto-kitchen()
  (send *ri* :move-to (send *eng2-scene* :spot "/eng2/7f/room73B2-sink-front0"))
  (send *fetch* :move-to (send (send *eng2-scene* :spot "/eng2/7f/room73B2-sink-front0") :copy-worldcoords) :world)
  (send *irtviewer* :look-all *fetch*)
  )


(defun goto-73A3()
  (send *ri* :move-to (send *eng2-scene* :spot "/eng2/7f/room73A3-front") )
  (send *fetch* :move-to (send (send *eng2-scene* :spot "/eng2/7f/room73A3-front") :copy-worldcoords) :world)
  (send *irtviewer* :look-all *fetch*)
  )

(defun goto-73A2()
  (send *ri* :move-to (send *eng2-scene* :spot "/eng2/7f/room73A2-front") )
  (send *fetch* :move-to (send (send *eng2-scene* :spot "/eng2/7f/room73A2-front") :copy-worldcoords) :world)
  (send *irtviewer* :look-all *fetch*)
  )



;;default view to watch robot
(defun dv()
    (send *irtviewer* :look-all *fetch*)
)

