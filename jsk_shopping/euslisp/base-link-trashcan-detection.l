;;棚から箱を取ってくる動作

(ros::roseus "trashcan-detection")
;;load packages
(load "package://fetcheus/fetch-interface.l")
;;(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
;;(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::load-ros-manifest "jsk_recognition_msgs")

;;defvarは一度やってしまったら上書きされない
(defvar *topic-name* "/pcl_nodelet/boxes_trashcanedge")
(defvar *gift-name* "/atohayoroshiku")
(defvar *bounding-box-list* nil)
(defvar *gift-number* nil)

;; (fetch-init)
;; (objects (list *fetch*))

;; `(setq time-stamp (ros::time 0))

;;callback
(defun bounding-box-array-cb (msg)
  ;;(ros::ros-info "cb called")
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  ;;(ros::ros-info "set box-list")
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (when (ros::time> time-stamp (send msg :header :stamp)) ;;rostime揃え
        (return-from bounding-box-array-cb nil))
      (ros::ros-info "tf")
      (setq obj-pos (scale 0.001
                           (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

(defun grasp-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (ros::ros-info "targe-cds 1")
  (ros::ros-info (format nil "~A" target-cds))
  (send target-cds :translate #f(-200 0 50)) ;;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
  (ros::ros-info "targe-cds 2")
  (ros::ros-info (format nil "~A" target-cds))
  (objects (list *fetch* target-cds))

  (send *fetch* :inverse-kinematics target-cds)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (send target-cds :translate #f(200 0 0)) :rotation-axis :y)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

(defun main()
  (ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)
  )
