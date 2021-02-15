#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")

(defun init ()
  (fetch-init)
  (setq *l* (instance ros::transform-listener :init))
  (objects (list *fetch*)))


(defun main ()

  (do-until-key
    (setq tm (send *l* :get-latest-common-time "/camera_link" "/target_nose"))
    (setq neck-angle (send *fetch* :head :neck-y :joint-angle))
    (ros::ros-info "neck-angle number ~A" (numberp neck-angle))
    (when tm
      (ros::ros-info "get latest common time is ~A before" (send (ros::time- (ros::time-now) tm) :to-sec))
      (when (< (send (ros::time- (ros::time-now) tm) :to-sec) 1.0)
        (setq target-coords (send *l* :lookup-transform "/camera_link" "/target_nose" tm))
        (setq target-pos  (send target-coords :worldpos))
        (ros::ros-info "lookup transform = ~A" target-pos)
        (setq coord-x (elt target-pos 0))
        (setq coord-y (elt target-pos 1))
        (ros::ros-info "y / x is ~A" (/ coord-y coord-x))
        ;;(setq neck-y (* (if (> (elt target-pos 1) 0) 10 -10)))
        (if (or (> (/ coord-y coord-x) 0.18) (< (/ coord-y coord-x) -0.18))
          (progn 
            (setq neck-y (+ neck-angle (* (* 10 (/ coord-y coord-x)))))
            (send *fetch* :head :neck-y :joint-angle neck-y)
          )
          (progn
            (ros::ros-info "close enough ~A" neck-angle)
            (send *fetch* :move-to (make-coords :pos (float-vector (/ coord-x 100) (/ coord-y 100) 0)) :local)
            (send *fetch* :rarm :angle-vector #f(52 30 -176 116 -27 -42 -13))
          )
        )
        (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000 :head-controller) ;; ????
      )
    )
    (send *irtviewer* :draw-objects)
    ;;
    (unix:usleep 100000)
    ))


