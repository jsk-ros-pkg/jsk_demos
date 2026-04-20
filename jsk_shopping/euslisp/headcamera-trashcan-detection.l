#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *topic-name* "/pcl_nodelet/boxes_trashcanedge")
(defvar *bounding-box-list* nil)

(defun bounding-box-array-cb (msg)
  (print 2)
  (setq *bounding-box-list* (send msg :boxes))
  (when *bounding-box-list*
    (let ((b (elt *bounding-box-list* 0))
          (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
          (cam-coords (send (send *fetch* :head_camera_depth_optical_frame_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send cam-coords :transform cam->obj-coords :worldpos))))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

(defun grasp-with-rec (obj-pos)
  (setq targe-cds (make-coords :pos (scale 1000 obj-pos)))
  (ros::ros-info "target-cds 1")
  (ros::ros-info (format nil "~A" target-cds))
  (send target-cds :translate #f(-200 0 50))
  (ros::ros-info "target-cds 2")
  (ros::ros-info (format nil "~A" target-cds))
  (objects (list *fetch* target-cds))

  (send *fetch* :inverse-kinematics target-cds)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  )
