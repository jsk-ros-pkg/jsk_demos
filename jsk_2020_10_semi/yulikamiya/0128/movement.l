(send *fetch* :reset-pose)

(send *fetch* :stop-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :stop-grasp)

;;; まずは首を動かす　-80→90→-45
(send *fetch* :head :neck-y :joint-angle -80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *fetch* :head :neck-y :joint-angle 80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *fetch* :head :neck-y :joint-angle -45)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *fetch* :move-to (make-coords :rpy (float-vector -45 0 0)) :local)
(send *fetch* :head :neck-y :joint-angle 0)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation t)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :move-to (send *fetch* :copy-worldcoords))
;;riが回転するときにangle-vectorを待つ方法→(send *ri* :wait-interpolation t)

(send *fetch* :move-to (make-coords :pos (float-vector 3000 0 0)) :local)
(send *irtviewer* :draw-objects)
(send *ri* :wait-interpolation t)
(send *ri* :move-to (send *fetch* :copy-worldcoords))

(send *fetch* :rarm :angle-vector #f(52 30 -176 116 -27 -42 -13))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)

(send *fetch* :start-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :start-grasp)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :world)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))