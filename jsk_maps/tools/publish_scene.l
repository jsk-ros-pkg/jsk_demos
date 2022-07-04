#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_rviz_plugins")

(defvar *scene* nil)
(defvar *current-map-id* nil)
(defvar *pub-scene-marker-topic* "/scene_marker")
(defvar *tf-listener* nil)
(defvar *scene-name* nil)

;; TODO object->marker-msg should ignore cascadedcoords in :bodies...(defun object->marker-msg (obj header &key
;; See https://github.com/jsk-ros-pkg/jsk_roseus/pull/713
(defclass dummy-scene
  :super cascaded-coords
  :slots (list bodies))
(defmethod dummy-scene
  (:init (b) (send-super :init) (setq bodies b) self)
  (:bodies () bodies))
(defun load-scene ()
  (let (s)
    (load (format nil "package://jsk_maps/src/~A-scene.l" *scene-name*))
    (setq s (funcall (intern (string-upcase (format nil "make-~A-scene" *scene-name*)))))
    (setq *scene* (instance dummy-scene :init
                            (mapcan #'(lambda (b)
                                        (cond ((find-method b :faces) (list b))
                                              (t nil)))
                                    (send s :bodies))))))


(defun publish-scene (&key (alpha 1.0) (lifetime 0))
  (let* ((stamp (ros::time-now))
         (header (instance std_msgs::header :init :stamp stamp :frame_id "/world"))
         current-map-coords scene )

    (unless (ros::get-topic-publisher *pub-scene-marker-topic*)
      (unix:sleep 1))
    ;; publish scene
    (setq msg (object->marker-msg *scene* header :ns *scene-name* :id 1 :alpha alpha :lifetime lifetime))
    (ros::publish *pub-scene-marker-topic* msg)
    ))


(defun main ()
  (ros::roseus "publish_scene")

  ;; load params
  (setq *scene-name* (ros::get-param "~scene" "eng2"))

  (ros::ros-info "Advertise ~A" *pub-scene-marker-topic* )
  (ros::advertise *pub-scene-marker-topic* visualization_msgs::Marker 5 t);; latched true

  (load-scene)

  (publish-scene)
  (ros::spin))

(main)
