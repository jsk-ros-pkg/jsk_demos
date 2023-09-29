#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_rviz_plugins")

(defvar *scene* nil)
(defvar *current-map-id* nil)
(defvar *pub-scene-marker-topic* "/scene_marker_array")
(defvar *tf-listener* nil)
(defvar *scene-name* nil)

;; TODO object->marker-msg should ignore cascadedcoords in :bodies...(defun object->marker-msg (obj header &key
;; See https://github.com/jsk-ros-pkg/jsk_roseus/pull/713
(defun vector->rgba (cv &optional (alpha 1.0))
  (if (= 4 (length cv)) (setq alpha (elt cv 3)))
  (if (vectorp cv)
      (instance std_msgs::ColorRGBA :init
                :r (elt cv 0)
                :g (elt cv 1)
                :b (elt cv 2)
                :a alpha)
    (instance std_msgs::ColorRGBA :init
              :r 0 :g 0 :b 0 :a alpha)))
(defun object->marker-array-msg (obj header &rest args)
  (let* ((bodies (mapcan #'(lambda (b)
                             (cond ((find-method b :faces) (list b))
                                   (t nil)))
                         (send obj :bodies)))
         (b1 (mapcan #'(lambda (b) (let ((c (get b :face-color))) (if (not (and (vectorp c) (= (length c) 4) (< (elt c 3) 1.0))) (list b)))) bodies)) ;; flat color
         (b2 (mapcan #'(lambda (b) (let ((c (get b :face-color))) (if (and (vectorp c) (= (length c) 4) (< (elt c 3) 1.0)) (list b)))) bodies))) ;; transparent color
    (instance visualization_msgs::MarkerArray :init
              :markers
              (list (apply #'object->marker-msg (instance bodyset-link :init obj :bodies b1) header :id 1 args)
                    (apply #'object->marker-msg (instance bodyset-link :init obj :bodies b2) header :id 2 args)))))

(defun load-scene ()
  (let ()
    (load (format nil "package://jsk_maps/src/~A-scene.l" *scene-name*))
    (setq *scene* (funcall (intern (string-upcase (format nil "make-~A-scene" *scene-name*)))))))


(defun publish-scene (&key (alpha 1.0) (lifetime 0))
  (let* ((stamp (ros::time-now))
         (header (instance std_msgs::header :init :stamp stamp :frame_id "/world"))
         current-map-coords scene msg)

    (unless (ros::get-topic-publisher *pub-scene-marker-topic*)
      (unix:sleep 1))
    ;; publish scene
    (dolist (b (send *scene* :bodies)) (if (substringp "/floor-plane" (send b :name)) (send b :set-color #f(0 1 0 0.05))))  ;; force change transparent color for floor
    (setq msg (object->marker-array-msg *scene* header :ns *scene-name* :alpha alpha :lifetime lifetime))
    (ros::publish *pub-scene-marker-topic* msg)
    ))


(defun main ()
  (ros::roseus "publish_scene")

  ;; load params
  (setq *scene-name* (ros::get-param "~scene" "eng2"))

  (ros::ros-info "Advertise ~A" *pub-scene-marker-topic* )
  (ros::advertise *pub-scene-marker-topic* visualization_msgs::MarkerArray 5 t) ;; latched true

  (load-scene)

  (publish-scene)
  (ros::spin))

(main)
