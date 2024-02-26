#!/usr/bin/env roseus

(require "package://fetcheus/fetch-interface.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "opencv_apps")
(ros::roseus-add-msgs "image_view2")

(setq *max-marker-index* 0)
(defun publish-image-marker (f index)
  (let* ((cx (elt (cadr (send f :centroid)) 0))
         (cy (elt (cadr (send f :centroid)) 1))
         (wh (elt (v- (rightmost-point (send f :vertices) (float-vector cx 0 0) (float-vector cx 640 0) #f(0 0 1))
                      (leftmost-point  (send f :vertices) (float-vector cx 0 0) (float-vector cx 640 0) #f(0 0 1))) 0))
         mrk)
    ;; publish found face
    (setq mrk (instance image_view2::ImageMarker2 :init))
    (send mrk :type image_view2::ImageMarker2::*POLYGON*)
    (send mrk :id (incf index))
    (send mrk :points (list (instance geometry_msgs::Point :init :x (- cx wh) :y (- cy wh))
                            (instance geometry_msgs::Point :init :x (- cx wh) :y (+ cy wh))
                            (instance geometry_msgs::Point :init :x (+ cx wh) :y (+ cy wh))
                            (instance geometry_msgs::Point :init :x (+ cx wh) :y (- cy wh))))
    (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0)))
    (ros::publish "/edgetpu_human_pose_estimator/output/image_marker" mrk)
    ;; publish area text
    (setq mrk (instance image_view2::ImageMarker2 :init))
    (send mrk :type image_view2::ImageMarker2::*TEXT*)
    (send mrk :id (incf index))
    (send mrk :position (instance geometry_msgs::Point :init :x (+ cx 30) :y (- cy (* 4 wh))))
    (send mrk :scale 1)
    (send mrk :text (format nil "~4,1f" (send f :area)))
    (ros::publish "/edgetpu_human_pose_estimator/output/image_marker" mrk)
    index))

(defun remove-image-marker (index)
  (let (mrk)
    (do ((i (+ index 1)))
        ((> i *max-marker-index*))
        (setq mrk (instance image_view2::ImageMarker2 :init))
        (send mrk :action image_view2::ImageMarker2::*REMOVE*)
        (send mrk :id i)
        (ros::publish "/edgetpu_human_pose_estimator/output/image_marker" mrk)
        (incf i))
    ))

(defun publish-image-info (sec)
  (let (mrk)
    (setq mrk (instance image_view2::ImageMarker2 :init))
    (send mrk :type image_view2::ImageMarker2::*TEXT*)
    (send mrk :id 0)
    (send mrk :position (instance geometry_msgs::Point :init :x 240 :y 20))
    (send mrk :scale 1)
    (send mrk :width 20)
    (send mrk :text (format nil "last valid pose: ~4,1f sec ago" sec))
    (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.1 :g 1.0 :b 0.1 :a 1.0))
    (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.1 :g 1.0 :b 0.1 :a 1.0))
    (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0)))
    (ros::publish "/edgetpu_human_pose_estimator/output/image_marker" mrk)
    ))
;;

(defun find-faces (poses names)
  (let (index f max-area max-f mrk)
    (setq max-area 0
          index 0)
    (dolist (pose poses)
      (let ((indices (flatten (mapcar #'(lambda (name)
                                 (if (listp name)
                                     (car (remove nil (mapcar #'(lambda (n) (position n (send pose :limb_names) :test #'string=)) name)))
                                   (position name (send pose :limb_names) :test #'string=))) names))))
        ;; check if we can find face
        (when (= (length names) (length indices))
          (setq f
                (make-face-from-vertices
                 (mapcar #'list
                         (quickhull
                          (mapcar #'(lambda (i) (float-vector (send (elt (send pose :poses) i) :position :x)
                                                              (send (elt (send pose :poses) i) :position :y)
                                                              0))
                                  indices)))))
          (ros::ros-info "      matched poses for ~A, area ~A" names (send f :area))
          ;; publish image marker
          (setq index (publish-image-marker f index))
          (if (> (send f :area) max-area)
              (setq max-area (send f :area)
                    max-f f)))))
    ;; cleanup old images
    (remove-image-marker index)
    (setq *max-marker-index* index)
    max-f))

(setq *face-3d-point* nil)
(defun point-cb (msg) (setq *face-3d-point* msg))

(setq *last-received-msg* (instance jsk_recognition_msgs::PeoplePoseArray :init :header (instance std_msgs::header :init :stamp (ros::time-now))))
(defun pose-cb (msg)
  (let (target-ray target-3d
                   message-received target-face target-area target-size target-x target-y
                   last-feedback-msg-stamp since-last-pose roi-msg point-msg
                   (target-area-min 60))
    (setq target-ray (make-cube 100 100 100))
    (setq target-3d (make-cylinder 100 10))
    (setq message-received (send (ros::time- (ros::time-now) (send msg :header :stamp)) :to-sec))
    (when (> message-received 3.0)
      (ros::ros-info "Too old message received ~A" message-received)
      (return-from pose-cb nil))

    (dolist (pose (send msg :poses)) (ros::ros-info "found poses ~A" (send pose :limb_names)))
    (setq target-face (find-faces (send msg :poses) (list "nose" "left eye" "right eye" "left ear" "right ear")))
    (unless target-face
      ;; if you can find full faces, check partial faces
      (setq target-face (find-faces (send msg :poses) (list "nose" (list "left eye" "left ear") (list "right eye" "right ear")))))
    (when target-face
      (setq target-area (send target-face :area)
            target-x (elt (cadr (send target-face :centroid)) 0)
            target-y (elt (cadr (send target-face :centroid)) 1)))
    (ros::ros-info "look at ~A ~A (target-area ~A > ~A)" target-x target-y target-area target-area-min)
    ;; send targets
    (when (and target-x target-y (> target-area target-area-min))
      (setq target-size (elt (v- (rightmost-point (send target-face :vertices) (float-vector target-x 0 0) (float-vector target-x 640 0) #f(0 0 1))
                                 (leftmost-point (send target-face :vertices) (float-vector target-x 0 0) (float-vector target-x 640 0) #f(0 0 1))) 0))
      (send target-ray :locate (v+ (send *head-camera-rgb* :worldpos)
                                   (scale 1000 (send *head-camera-rgb* :ray target-x target-y))))

      ;; publish screen point for face position
      (setq point-msg (instance geometry_msgs::PointStamped :init))
      (send point-msg :header (send msg :header))
      (send point-msg :point (instance geometry_msgs::Point :init :x target-x :y target-y))
      (ros::publish "head_camera/rgb/image_rect_color/screenpoint" point-msg)

      ;; publish roi for face recognition
      (setq roi-msg (instance opencv_apps::FaceArrayStamped :init))
      (send roi-msg :header (send msg :header))
      (send roi-msg :faces
            (list (instance opencv_apps::Face :init :face (instance opencv_apps::Rect :init :x target-x :y target-y :width (* 2 target-size) :height (* 2 target-size)))))
      (ros::publish "head_camera/rgb/face_roi" roi-msg)
      ;;
      ;; look at target
      (send *fetch* :head :look-at (send target-ray :worldpos))
      (send *fetch* :head :angle-vector
            (v+ (scale 0.4 (v- (send *fetch* :head :angle-vector) (send *ri* :robot :head :angle-vector)))
                (send *ri* :robot :head :angle-vector)))

      ;; send motion
      (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 200 :head-controller)
      (unix::usleep (* 150 1000))
      (setq *last-received-msg* msg)

      ;;
      (when *face-3d-point*
        (let ((tm (send (ros::time- (ros::time-now) (send msg :header :stamp)) :to-sec)))
          (if (> tm 3)
              (ros::ros-warn "received too old message (~A sec ago)" tm)
            (progn
              (send target-3d :locate
                    (send *ri* :robot
                          (intern (string-upcase (format nil "~A_lk" (send *face-3d-point* :header :frame_id))) *keyword-package*)
                          :transform-vector (ros::tf-point->pos (send *face-3d-point* :point))))
              (ros::ros-info "found face position ~A" (send target-3d :worldpos))
              ))))
      )

    ;; if we do not send :head-controller for a while
    (setq since-last-pose (send (ros::time- (ros::time-now) (send *last-received-msg* :header :stamp)) :to-sec))
    ;; publish to image_view2
    (publish-image-info since-last-pose)
    ;;
#| ;; This node only deal with look-at stuff
    (when (> since-last-pose 10)
      (case (mod *not-found-count* 3)
            (0 (send *fetch* :head :angle-vector #f( 60 0)))
            (1 (send *fetch* :head :angle-vector #f(-60 0)))
            (2 (send *fetch* :head :angle-vector #f(  0 0))))
      (ros::ros-warn "we do not send targets for ~A sec, head :angle-vector ~A" since-last-pose (send *fetch* :head :angle-vector))
      (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 2000 :head-controller)
      (incf *not-found-count*)
      (setq *last-received-msg* msg)
      )
|#
    ;; visualize
    (objects (list *fetch* (make-cube 5000 5000 1) (send *ri* :robot) target-ray target-3d))
    (x::window-main-one)
    ))


(ros::roseus "search_people")
(fetch-init)
(setq *head-camera-rgb*
      (make-camera-from-ros-camera-info-aux
       640 480
       #f(527.3369104449899 0.0 317.4046362049818 0.0 0.0 530.5168862388377 234.1751028861198 0.0 0.0 0.0 1.0 0.0)
       (send *ri* :robot :head_camera_rgb_optical_frame_lk) :name :head_camera/rgb))
;;
(ros::advertise "head_camera/rgb/face_roi" opencv_apps::FaceArrayStamped 1)
(ros::advertise "head_camera/rgb/image_rect_color/screenpoint" geometry_msgs::PointStamped 1)
(ros::advertise "edgetpu_human_pose_estimator/output/image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "edgetpu_human_pose_estimator/output/poses" jsk_recognition_msgs::PeoplePoseArray #'pose-cb)
(ros::subscribe "pointcloud_screenpoint_nodelet/output_point" geometry_msgs::PointStamped #'point-cb)


;; (ros::set-logger-level "ros.roseus" ros::*roserror*)
(setq *not-found-count* 0)
(ros::rate 200)
(do-until-key
 ;; spin
 (ros::sleep)
 (ros::spin-once))

