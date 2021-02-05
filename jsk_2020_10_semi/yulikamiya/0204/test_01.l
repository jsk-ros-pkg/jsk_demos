#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")

(defun init ()
  (fetch-init)
  (setq *l* (instance ros::transform-listener :init))
  (objects (list *fetch*)))


(defun main ()

  (do-until-key
   ;; (setq tm (send *l* :get-latest-common-time "/camera_link" "/ray_target"))
    (setq tm (send *l* :get-latest-common-time "/base_footprint" "/ray_target"))
    (setq neck-angle (send *fetch* :head :neck-y :joint-angle))
    (ros::ros-info "neck-angle is ~A" neck-angle)
    (when tm
      (ros::ros-info "get latest common time is ~A before" (send (ros::time- (ros::time-now) tm) :to-sec))
      (when (< (send (ros::time- (ros::time-now) tm) :to-sec) 1.0)
        ;;(setq target-coords (send *l* :lookup-transform "/camera_link" "/ray_target" tm))
	(setq target-coords (send *l* :lookup-transform "/base_footprint" "/ray_target" tm))
        (setq target-pos  (send target-coords :worldpos))
        (ros::ros-info "lookup transform = ~A" target-pos)
        (setq coord-x (elt target-pos 0))
        (setq coord-y (elt target-pos 1))
        ;;(setq neck-y (* (if (> (elt target-pos 1) 0) 10 -10)))
        (setq neck-y (+ neck-angle (* (* 5 (/ coord-y coord-x)))))
        (send *fetch* :head :neck-y :joint-angle neck-y)
        (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 200 :head-controller) ;; ????
      )
    )
    (send *irtviewer* :draw-objects)
    ;;
    (unix:usleep 200000)
    ))


