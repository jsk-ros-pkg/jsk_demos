#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*

;; show in irtviewer
(objects (list *fetch* *scene*))
;; show in "fetch Kinematics Simulator"
(send *ri* :objects (send *scene* :objects))

;; use (mapcar #'(lambda (x) (send x :name)) (send *scene* :spots)) to get list of spots
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :world)

;; check 20201026-soft3-model, the code to focus on the robot model
(send *irtviewer* :look-all
      (send (geo::make-bounding-box
             (flatten (send-all (send *fetch* :bodies) :vertices)))
            :grow 2.0))


;; start to move
(setq *shelf-front* (send (send (send *scene* :spot "/eng2/7f/room73B2-table-side") :copy-worldcoords) :rotate pi/2 :z))
(send *fetch* :move-to *shelf-front* :world) ;; moved in irtviewer
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to *shelf-front*) ;; moved real robot

;; move arms
(send *fetch* :rarm :angle-vector #f(-55 80 -45 -110 0 30 15))
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)


;; back to original position
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)
(send *irtviewer* :draw-objects) ;; update viewer

(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :world)
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to  (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front"))
