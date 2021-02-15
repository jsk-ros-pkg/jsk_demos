;;;movement with constants COMPLETE
;;; move in front of the door
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-door-front") :world)
(send *fetch* :move-to (make-coords :pos (float-vector -600 -400 0)) :local)
(send *fetch* :move-to (make-coords :rpy (float-vector 90 0 0)) :local)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))

(send *fetch* :stop-grasp)
(send *irtviewer* :draw-objects)

;;; まずは首を動かす until he finds someone　-80→90→-45
;; this should be another node from here
(send *fetch* :head :neck-y :joint-angle -80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :head :neck-y :joint-angle 80)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :head :neck-y :joint-angle 45)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :move-to (make-coords :rpy (float-vector 45 0 0)) :local)
(send *fetch* :head :neck-y :joint-angle 0)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :move-to (send *fetch* :copy-worldcoords))
;;riが回転するときにangle-vectorを待つ方法→(send *ri* :wait-interpolation)

(send *ri* :wait-interpolation)
(send *fetch* :move-to (make-coords :pos (float-vector 2360 7074 51)) :world)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))

(send *ri* :wait-interpolation)
(send *fetch* :head :neck-p :joint-angle -20)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :torso :waist-z :joint-angle 180)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
;;to here

;; facial recognition here
;; facial recognition complete, on to measuring temperature

;;third node
(send *ri* :wait-interpolation)
(send *fetch* :rarm :angle-vector #f(0.950514 78.0375 -175.645 128.973 -0.084058 55.6689 94.4493))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :head :neck-p :joint-angle 0)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :start-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :start-grasp :wait t)

(send *ri* :wait-interpolation)
(send *fetch* :stop-grasp)
(send *irtviewer* :draw-objects)
(send *ri* :stop-grasp :wait t)
;; to here, send done message to python node

;; send temperature to Spread Sheet(Python)

;; go back


(send *ri* :wait-interpolation)
(send *fetch* :rarm :angle-vector #f(75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0))
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :torso :waist-z :joint-angle 20)
(send *irtviewer* :draw-objects)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)

(send *ri* :wait-interpolation)
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-door-front") :world)
(send *fetch* :move-to (make-coords :pos (float-vector -600 -400 0)) :local)
(send *fetch* :move-to (make-coords :rpy (float-vector 90 0 0)) :local)
(send *irtviewer* :draw-objects)
(send *ri* :move-to (send *fetch* :copy-worldcoords))