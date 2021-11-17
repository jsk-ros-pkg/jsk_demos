(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *pr2*))

;reportの代わりにボトル
(setq *bottle* (send *room73b2* :object "room73b2-bottle"))
(send (car (send *bottle* :handle)) :draw-on :flush t)

(defun wait-for-report ()
  (send *pr2* :move-to (make-coords :pos #f(3000 500 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *irtviewer* :draw-objects)
)

(defun catch-report ()
  (if (send *bottle* :parent) (send (send *bottle* :parent) :dissoc *bottle*))
  (send *pr2* :larm :inverse-kinematics
    (car (send *bottle* :handle))
    :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
    :move-target (send *pr2* :larm :end-coords)
    :rotation-axis :z
    :debug-view t)
  (send *pr2* :larm :end-coords :assoc *bottle*)
  (send *irtviewer* :draw-objects)
)

(defun carry-report ()
  (send *pr2* :larm :move-end-pos #f(0 -200 50) :world :debug-view t :look-at-target t)
  (send *pr2* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *irtviewer* :draw-objects)
  (send *pr2* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
  (send *pr2* :move-to (make-coords :pos #f(1000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *irtviewer* :draw-objects)
)

(defun all-task ()
  (wait-for-report)
  (catch-report)
  (carry-report)
)