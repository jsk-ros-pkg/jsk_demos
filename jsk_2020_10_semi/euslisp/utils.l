#!/usr/bin/env roseus

;; functions
(defun move-to-place (place)
(send *fetch* :move-to (send *scene* :spot place) :world)
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to (send *scene* :spot place))
)

(defun move-to-coord (coord)
(send *fetch* :move-to coord :world)
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to coord)
)

(defun base-position ()
;; back to original position
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)
(send *irtviewer* :draw-objects) ;; update viewer
(move-to-place "/eng2/7f/room73B2-fetch-dock-front")
)

