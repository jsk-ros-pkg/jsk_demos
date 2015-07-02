#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "sensor_msgs")
(ros::roseus "robot_bbox")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")
;; publish bounding box of robot



(load (robot-file (unix::getenv "ROBOT")))

;; parameters
(setq *use-x* (ros::get-param "~use_x" nil))
(setq *robot* (make-robot-model-from-name (unix::getenv "ROBOT")))
(setq *use-links* (ros::get-param "~links" nil))
;; Analysis level: whoe_links or each links
(setq *analysis-level* (ros::get-param "~analysis_level" "whole_links"))
(setq *padding-x* (ros::get-param "~padding_x" 0.0))
(setq *padding-y* (ros::get-param "~padding_y" 0.0))
(setq *padding-z* (ros::get-param "~padding_z" 0.0))

(when *use-x*
  (objects (list *robot*)))

(defun all-links-bbox (bodies)
  (let ((vertices (flatten (send-all bodies :vertices))))
    (let ((bbox (make-bounding-box vertices)))
      (when *use-x*
        (let ((bbox-obj (send bbox :body)))
          (setf (get bbox-obj :face-color) :green)
          (gl::transparent bbox-obj 0.2)
          (send *irtviewer* :objects (list *robot* bbox-obj))
          (send *irtviewer* :draw-objects)
          ))
      (let ((dimensions (scale 0.001 (v- (send bbox :maxpoint) (send bbox :minpoint))))
            (center (scale (* 0.5 0.001) (v+ (send bbox :maxpoint) (send bbox :minpoint)))))
        (let ((bbox-msg (instance jsk_recognition_msgs::BoundingBox :init)))
          (send bbox-msg :pose :orientation :w 1.0)
          (send bbox-msg :pose :position :x (elt center 0))
          (send bbox-msg :pose :position :y (elt center 1))
          (send bbox-msg :pose :position :z (elt center 2))
          (send bbox-msg :dimensions :x (+ (* 2 *padding-x*) (elt dimensions 0)))
          (send bbox-msg :dimensions :y (+ (* 2 *padding-y*) (elt dimensions 1)))
          (send bbox-msg :dimensions :z (+ (* 2 *padding-z*) (elt dimensions 2)))
          (list bbox-msg))))))

(defun each-link-bbox (bodies)
  (when *use-x*
    (send *irtviewer* :objects (list *robot*)))
  (mapcar #'(lambda (b)
              (let ((vertices
                     (mapcar #'(lambda (v)
                                 (send (send b :copy-worldcoords)
                                       :inverse-transform-vector v))
                             (send b :vertices))))
                (let ((bbox (make-bounding-box vertices)))
                  (when *use-x*
                    (let ((bbox-obj (send bbox :body)))
                      (setf (get bbox-obj :face-color) :green)
                      (gl::transparent bbox-obj 0.2)
                      (send *irtviewer* :objects (cons bbox-obj (send *irtviewer* :objects)))
                      (send *irtviewer* :draw-objects)
                      ))
                  (let ((dimensions (scale 0.001 (v- (send bbox :maxpoint)
                                                     (send bbox :minpoint))))
                        (center (scale (* 0.5 0.001) (v+ (send bbox :maxpoint)
                                                         (send bbox :minpoint))))
                        (body-offset (send (send (send *robot* :body_lk) :copy-worldcoords)
                                           :transformation
                                           (send b :copy-worldcoords))))
                    (let ((bbox-msg (instance jsk_recognition_msgs::BoundingBox :init))
                          (pose (send body-offset :translate (scale 1000 center) :local)))
                      (send bbox-msg :pose (ros::coords->tf-pose pose))
                      ;; (send bbox-msg :pose :position :x (elt center 0))
                      ;; (send bbox-msg :pose :position :y (elt center 1))
                      ;; (send bbox-msg :pose :position :z (elt center 2))
                      (send bbox-msg :dimensions :x (+ (* 2 *padding-x*) (elt dimensions 0)))
                      (send bbox-msg :dimensions :y (+ (* 2 *padding-y*) (elt dimensions 1)))
                      (send bbox-msg :dimensions :z (+ (* 2 *padding-z*) (elt dimensions 2)))
                      bbox-msg)))))
          bodies))

(defun joint-states-callback (msg)
  (dotimes (i (length (send msg :name)))
    (let ((name (elt (send msg :name) i))
          (angle (elt (send msg :position) i)))
      ;; check name is included in joint-list
      (let ((j (find-if #'(lambda (x) (string= (send x :name) name))
                        (send *robot* :joint-list))))
        (if j (send j :joint-angle (rad2deg angle))))))
  ;; move robot's BODY to origin
  (let ((robot-trans (send (send *robot* :link "BODY") :copy-worldcoords)))
    (send *robot* :transform (send robot-trans :inverse-transformation)))
  (let* ((links (if *use-links*
                    (remove-if-not #'(lambda (b)
                                       (member (send b :name) *use-links* :test #'string=))
                                   (send *robot* :links))
                  (send *robot* :links)))
         (bodies (flatten (send-all links :bodies))))
    (send-all bodies :worldcoords)
    (let ((box-msgs (cond
                     ((string= *analysis-level* "whole_links")
                      (all-links-bbox bodies))
                     ((string= *analysis-level* "each_link")
                      (each-link-bbox bodies))))
          (bbox-msg-array (instance jsk_recognition_msgs::BoundingBoxArray :init)))
      ;; setup header for each box-msg
      (send-all box-msgs :header :stamp (send msg :header :stamp))
      (send-all box-msgs :header :frame_id "BODY")
      (send bbox-msg-array :boxes box-msgs)
      (send bbox-msg-array :header (send (car box-msgs) :header))
      (ros::publish "/robot_bbox_array" bbox-msg-array)))
  (when *use-x*
    (x::window-main-one))
  )

(ros::advertise "/robot_bbox" jsk_recognition_msgs::BoundingBox)
(ros::advertise "/robot_bbox_array" jsk_recognition_msgs::BoundingBoxArray) ;for visualization
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-states-callback)

(ros::rate 10)
(ros::spin)

