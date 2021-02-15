(send *fetch* :stop-grasp)
(send *irtviewer* :draw-objects)

;;; まずは首を動かす　-80→90→-45
(send *fetch* :head :neck-y :joint-angle -80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :head :neck-y :joint-angle 80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :head :neck-y :joint-angle 45)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :move-to (make-coords :rpy (float-vector 45 0 0)) :local)
(send *fetch* :head :neck-y :joint-angle 0)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :move-to (send *fetch* :copy-worldcoords))
;;riが回転するときにangle-vectorを待つ方法→(send *ri* :wait-interpolation t)

(send *ri* :wait-interpolation t)
(send *fetch* :move-to (make-coords :pos (float-vector 2360 7074 51)) :world)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))

(send *ri* :wait-interpolation t)
(send *fetch* :torso :waist-z :joint-angle 80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :rarm :angle-vector #f(52 30 -176 116 -27 -42 -13))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :start-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :start-grasp)

(send *ri* :wait-interpolation t)
(send *fetch* :stop-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :stop-grasp)

(send *ri* :wait-interpolation t)
(send *fetch* :rarm :angle-vector #f(75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :reset-pose)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation t)
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-door-front") :world)
(send *fetch* :move-to (make-coords :pos (float-vector -600 -400 0)) :local)
(send *fetch* :move-to (make-coords :rpy (float-vector 90 0 0)) :local)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))