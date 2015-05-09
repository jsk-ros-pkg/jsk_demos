#!/usr/bin/env roseus
(ros::roseus  "check_hand")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/primitive-marker-util.l")
(load "package://drc_task_common/euslisp/robot-util.l")


;(setq *robot-name* (string-upcase (unix::getenv "ROBOT")))
;(generate-robot *robot-name* :generate-ri? nil)
(unless (boundp '*tf*)
  (setq *tf* (instance ros::transform-listener :init)))


(defun check-dist
  ()
  (setq a (get-primitive-marker-pose :frame-id "rarm_end_coords"))
  (format nil "~A" a)
  )

(defun insert-primitive-marker
  (&key (shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*) (name "default_name") (description "default_description") (resource "") (use-material t))
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type shape-type :action jsk_rviz_plugins::TransformableMarkerOperate::*INSERT* :frame_id "rarm_end_coords" :name name :description description :mesh_resource resource :mesh_use_embedded_materials use-material))
    (ros::service-call "/transformable_interactive_server/request_marker_operate" req)
    )) ;; over write!!
(defun insert-model ()
  (insert-primitive-marker :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*MESH_RESOURCE* :name "drill" :description "" :resource "package://drc_task_common/models/takenoko_drill.dae" :use-material t)
  (set-primitive-marker-pose (make-coords :pos (float-vector 50 2 140) :rpy (list (deg2rad 180) 0 pi)) "rarm_end_coords")
  )
(warn ";;
;; (insert-model)
;; (check-dist)
;;
")
