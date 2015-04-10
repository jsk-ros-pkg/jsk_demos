#!/usr/bin/env roseus

(ros::roseus "rv_map_publiser")
(ros::load-ros-manifest "visualization_msgs")

(load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l")

;; *map-step-dist*   map上のxyz方向の点間隔を表すfloat-vector
;; *map-point-list*  map上の各点の座標のリスト
;; *map-color-list*  map上の各点の色のリスト

(defun init-rv-map
  ()
  (ros::advertise "reachable_visible_map" visualization_msgs::marker 1)
  (ros::rate 10)
  )

(defun generate-rv-map
  ()

  (setq *hrp2* (instance hrp2jsknt-robot :init))
  (send *hrp2* :reset-manip-pose)
  (send *hrp2* :fix-leg-to-coords (make-coords))
  (objects (list *hrp2*))

  (setq *map-min-point* (list 0 -500 500))
  (setq *map-max-point* (list 1000 500 1500))
  (setq *map-step-dist* (float-vector 50 50 50))
  (setq *map-angle-list* (list (- (/ pi 3)) (- (/ pi 6)) 0 (/ pi 6) (/ pi 3)))

  (setq *map-point-list* nil)
  (setq *map-color-list* nil)

  (dotimes (ix (round (/ (- (elt *map-max-point* 0) (elt *map-min-point* 0)) (elt *map-step-dist* 0))))
    (dotimes (iy (round (/ (- (elt *map-max-point* 1) (elt *map-min-point* 1)) (elt *map-step-dist* 1))))
      (dotimes (iz (round (/ (- (elt *map-max-point* 2) (elt *map-min-point* 2)) (elt *map-step-dist* 2))))
        (let ((solved-num 0)
              (map-point
               (float-vector
                (+ (elt *map-min-point* 0) (* ix (elt *map-step-dist* 0)))
                (+ (elt *map-min-point* 1) (* iy (elt *map-step-dist* 1)))
                (+ (elt *map-min-point* 2) (* iz (elt *map-step-dist* 2))))))
          (dolist (angle *map-angle-list*)
            (when (send *hrp2* :rarm :inverse-kinematics (make-coords :pos map-point :rpy (list 0 0 angle)))
                        (incf solved-num)))
          (push map-point *map-point-list*)
          (push (list (v+ (scale (/ (* solved-num 1.0) (length *map-angle-list*)) (float-vector 1 0 1))
                          (scale (- 1 (/ (* solved-num 1.0) (length *map-angle-list*))) (float-vector 0 1 1)))
                      (/ (* solved-num 1.0) (length *map-angle-list*)))
                *map-color-list*)
          )
        )))
  )

(defun save-rv-map
  ()
  (dump-loadable-structure "./log-rv-map.l" *map-step-dist* *map-point-list* *map-color-list*))

(defun load-rv-map
  ()
  (load "package://drc_task_common/euslisp/log-rv-map.l"))



(defun publish-rv-map
  ()

  (let* ((msg (instance visualization_msgs::marker :init))
         (pose-msg (instance geometry_msgs::pose :init))
         points-list-msg colors-list-msg
         ;; (scale-msg (instance geometry_msgs::vector3 :init :x 1 :y 1 :z 1))
         ;; (points-list-msg (list (instance geometry_msgs::point :init :x 0 :y 0 :z 0)
         ;;                        (instance geometry_msgs::point :init :x 1 :y 0 :z 0)
         ;;                        (instance geometry_msgs::point :init :x 2 :y 0 :z 0)))
         ;; (colors-list-msg (list (instance std_msgs::ColorRGBA :init :r 1 :g 0 :b 0 :a 1)
         ;;                       (instance std_msgs::ColorRGBA :init :r 0 :g 1 :b 0 :a 0.5)
         ;;                       (instance std_msgs::ColorRGBA :init :r 0 :g 0 :b 1 :a 0.1)))
         )
    (send msg :header (instance std_msgs::header :init
                                :stamp (ros::time-now) :frame_id "/robot_marker_root"))
    (send msg :frame_locked t)
    (send msg :type 6) ;; cube_list
    (send msg :action 0)
    (send msg :pose pose-msg)
    (send msg :scale (instance geometry_msgs::vector3 :init
                               :x (* 0.5 (* 0.001 (elt *map-step-dist* 0)))
                               :y (* 0.5 (* 0.001 (elt *map-step-dist* 1)))
                               :z (* 0.5 (* 0.001 (elt *map-step-dist* 2)))))
    (dotimes (i (length *map-point-list*))
      (push (instance geometry_msgs::point :init
                      :x (* 0.001 (elt (elt *map-point-list* i) 0))
                      :y (* 0.001 (elt (elt *map-point-list* i) 1))
                      :z (* 0.001 (elt (elt *map-point-list* i) 2)))
            points-list-msg)
      (push (vector->rgba (elt (elt *map-color-list* i) 0) (elt (elt *map-color-list* i) 1)) colors-list-msg))
    (send msg :points points-list-msg)
    (send msg :colors colors-list-msg)
    (do-until-key
     (send msg :header (instance std_msgs::header :init
                                 :stamp (ros::time-now) :frame_id "/robot_marker_root"))
     (ros::publish "reachable_visible_map" msg)
     (ros::sleep))
    )
  )

(defun main
  ()
  (init-rv-map)
  (load-rv-map)
  (publish-rv-map))

(main)

;;(generate-rv-map)
;;(save-rv-map)
