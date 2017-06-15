#!/usr/bin/env runeus
;; -*- mode: EusLisp; coding: utf-8-unix; -*-

(ros::load-ros-manifest "detect_cans_in_fridge_201202")

(setq ros::*compile-message* t) ;; switch for compile

(ros::roseus "white_balance_main")

(ros::roseus-add-msgs "pcl_msgs")
(ros::roseus-add-srvs "jsk_perception")
(ros::roseus-add-srvs "jsk_recognition_msgs")
(ros::roseus-add-msgs "posedetection_msgs")

(defvar *topic-name* "/plane_extraction/output_nonplane_cloud") ;; point cloud from camera
(defvar *image-topic* "/kinect_head/rgb/image_rect_color")
(defvar *screen-topic* "/kinect_head/rgb/image_rect_color/screenpoint")
(defvar *result-topic* "/kinect_head/depth_registered/ObjectDetection")

(load "match-hist.l")

(defvar *model-names* (ros::get-param "/object_hs_histgram/names"))
(defvar *debug-view* (ros::get-param "~debug_view" t))

(setq *3dpos* (instance pointcloud :init))
(setq *p* nil
      *obj-lst* nil
      *obj-det* nil
      *ref-color* #f(1 1 1))

;; To get reference white color
(defun screenpoint-cb (msg)
  (let ((px (send msg :point :x)) (py (send msg :point :y)) img)
    (let ((var (gensym)) (i 0))
      (ros::subscribe *image-topic* sensor_msgs::Image
	#'(lambda (msg) (set var msg)))
      (while (and (ros::ok) (not (boundp var)) (< (incf i) 100))
	(ros::spin-once)
	(unix::usleep 1000000))
      (ros::unsubscribe *image-topic*)
      (when (boundp var) (setq img (eval var))))
    (when img
      (let ((idx (+ (* (send img :step) (round py)) (* (round px) 3)))
	    (data (send img :data)))
	(setq *ref-color* (scale (/ 1.0 256)
				 (float-vector (elt data (+ idx 2))
					       (elt data (+ idx 1))
					       (elt data (+ idx 0)))))
	(ros::ros-info "reference white is ~a" *ref-color*))
      )))
(ros::subscribe *screen-topic* geometry_msgs::PointStamped #'screenpoint-cb)

(defun pcloud-cb (msg)
  (pprint 'capture)
  (make-eus-pointcloud-from-ros-msg msg :pcloud *3dpos* :remove-nan t) ;; ???
  (let ((p (instance pointcloud :init)) q qlst obj)
    (setq *obj-det* nil)
    (send p :copy-from *3dpos*)
    (setq *p* p)
    (send p :filter :key #'(lambda (p) (> 1500 (norm p) 100)))
    (setq q (white-balance-points p *ref-color*))
    (setq qlst (euclidean-cluster-points q 0.01))
    (when *debug-view*
      (objects (list q))
      (x::window-main-one))
    (setq *obj-lst* (remove-if #'(lambda (x) (bbox-area (send x :box))) qlst))
    ;; *obj-lst* -> point cloud list
    ;; need match-hist.l
    (dolist (nm *model-names*)
      (setq obj nil)
      (when *obj-lst*
        ;; object match using histogram
        (setq obj (match-model *obj-lst* (format nil "/object_hs_histgram/~A" nm))))
      )
    (let (detected-objects)
      (dolist (nm *model-names*)
        (let ((mname (format nil "/object_hs_histgram/~A" nm))
              ret obj)
          (setq ret (remove-if #'(lambda (x) (not (assoc mname (get x :results) :test #'string=))) *obj-lst*))
          (when ret
            (sort ret #'(lambda (x y)
                          (>= (cadr (assoc mname (get x :results) :test #'string=))
                              (cadr (assoc mname (get y :results) :test #'string=)))))
            (pprint (list nm ret))
            (setq obj (car ret))
            (let ((bb (make-bounding-box (send obj :point-list)))
                  pos rospose
                  (o6dp (instance posedetection_msgs::Object6DPose :init)))
              (setq pos (scale 0.5 (v+ (send bb :maxpoint) (send bb :minpoint))))
              (setq rospose (ros::coords->tf-pose (make-coords :pos pos)))
              (send o6dp :pose rospose)
              (send o6dp :type nm)
              (push o6dp detected-objects)))
          ))
      (when detected-objects
        (let ((res (instance posedetection_msgs::ObjectDetection :init)))
          (send res :header (send msg :header))
          (send res :objects detected-objects)
          (format t "result publish~%")
          (ros::publish *result-topic* res)
          )))
    )
  )
(ros::advertise *result-topic* posedetection_msgs::ObjectDetection)

(setq *cloud-subscribed* nil)
(defun subscribe-cloud-topic ()
  (unless *cloud-subscribed*
    (ros::ros-info "subscribe cloud topics")
    (setq *cloud-subscribed* t)
    (ros::subscribe *topic-name* sensor_msgs::PointCloud2 #'pcloud-cb)))
(defun unsubscribe-cloud-topic ()
  (when *cloud-subscribed*
    (ros::ros-info "unsubscribe cloud topics")
    (setq *cloud-subscribed* nil)
    (ros::unsubscribe *topic-name*)))

(defun white-balance-points (3dp &optional (reference-color #f(1 1 1)))
  (let ((req (instance jsk_perception::WhiteBalancePointsRequest :init))
        ret)
    (send req :input (make-ros-msg-from-eus-pointcloud 3dp :with-color t))
    (send req :reference_color reference-color)

    (setq ret (ros::service-call "/white_balance/convert_points" req))
    (make-eus-pointcloud-from-ros-msg (send ret :output))
    ))

(defun euclidean-cluster-points (3dp &optional (tolerance 0.01))
  (when (or (not 3dp) (= (send 3dp :size) 0))
    (return-from euclidean-cluster-points nil))
  (let ((req (instance jsk_recognition_msgs::EuclideanSegmentRequest :init))
        ret)
    (send req :input (make-ros-msg-from-eus-pointcloud 3dp :with-color t))
    (send req :tolerance tolerance);;

    (setq ret (ros::service-call "/euclidean_clustering/euclidean_clustering" req))
    (mapcar #'(lambda (m) (make-eus-pointcloud-from-ros-msg m)) (send ret :output))
    ))

;; simple bounding box size check
(defun bbox-area (bx &optional (max-area 15000) (min-area 3000) (max-len 160))
  (let* ((bf
          (sort (send bx :diagonal) #'(lambda (x y) (>= x y)))))
    ;;(* (elt bf 0) (elt bf 1))
    (or (not (> max-area (* (elt bf 0) (elt bf 1)) min-area))
        (> (elt bf 0) max-len))
    ))

;; get reference white color from white cluster
(defun get-reference-white (objs)
  (dolist (obj objs)
    (let (bb size)
      (setq bb (make-bounding-box (send obj :point-list)))
      (setq size (v- (send bb :maxpoint) (send bb :minpoint)))
      ;;
      )))

(defun ros-spin ()
  (ros::rate 15)
  (do-until-key
   (cond
    ((= (ros::get-num-subscribers *result-topic*) 0)
     (unsubscribe-cloud-topic))
    (t (subscribe-cloud-topic)))
   (ros::spin-once)
   (ros::sleep)
   ;;(x::window-main-one)
   ))

(defun ros-shutdown ()
  (ros::roseus "shutdown")
  (exit))

(ros-spin)

;;;
;(setq p *lst*)
;(send p :filter :key #'(lambda (p) (> (norm p) 100)))

;; cluster
;(setq plst (euclidean-cluster-points p 0.01))
;(setq pobj-lst (remove-if #'(lambda (x) (bbox-area (send x :box))) plst))

;; ref-color <- #f(r g b) / observation of white
;;color convert
;(setq q (white-balance-points p *ref-color*))
;(setq qlst (euclidean-cluster-points p 0.01))
;(setq obj-lst (remove-if #'(lambda (x) (bbox-area (send x :box))) qlst))

;; match obj-lst to models
;(objects obj-lst)

(defun change-background (col)
 (let ((vw (send *viewer* :viewsurface)))
   (setq (vw . x::bg-color) col)
   (gl::glClearColorfv col)))
