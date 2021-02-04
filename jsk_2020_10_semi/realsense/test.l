#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")

(defun init ()
  (fetch-init)
  (setq *l* (instance ros::transform-listener :init))
  (objects (list *fetch*)))


(defun main ()

  (do-until-key
    (setq tm (send *l* :get-latest-common-time "/base_footprint" "/ray_target"))
    (when tm
      (ros::ros-info "get latest common time is ~A before" (send (ros::time- (ros::time-now) tm) :to-sec))
      (when (< (send (ros::time- (ros::time-now) tm) :to-sec) 1.0)
	(setq target-coords (send *l* :lookup-transform "/base_footprint" "/ray_target" tm))
	(setq target-pos  (send target-coords :worldpos))
	(ros::ros-info "lookup transform = ~A" target-pos)
	(setq coord-y (elt target-pos 1))
	(setq neck-y (* (if (> (elt target-pos 1) 0) 10 -10)))
	(send *fetch* :head :neck-y :joint-angle neck-y)
	;; (send *ri* :angle-vector (send *fetch* :angle-vector) 1000 :head-controller) ;; ????
	))
    (send *irtviewer* :draw-objects)
    ;;
    (unix:sleep 1)
    ))


