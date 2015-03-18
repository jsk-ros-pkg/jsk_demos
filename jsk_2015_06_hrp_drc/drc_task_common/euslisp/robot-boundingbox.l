#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "sensor_msgs")
(ros::roseus "robot_bbox")
(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")
;; publish bounding box of robot

(setq *use-x* nil)

(load (robot-file (unix::getenv "ROBOT")))
;;(init-robot-from-name (unix::getenv "ROBOT"))
(setq *robot* (make-robot-model-from-name (unix::getenv "ROBOT")))
(setq *use-links* (ros::get-param "~links"))

(when *use-x*
  (objects (list *robot*)))

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
          (let ((bbox-msg (instance jsk_recognition_msgs::BoundingBox :init))
                (bbox-msg-array (instance jsk_recognition_msgs::BoundingBoxArray :init)))
            (send bbox-msg :header :stamp (send bbox-msg :header :stamp))
            (send bbox-msg :header :frame_id "BODY")
            (send bbox-msg :pose :orientation :w 1.0)
            (send bbox-msg :pose :position :x (elt center 0))
            (send bbox-msg :pose :position :y (elt center 1))
            (send bbox-msg :pose :position :z (elt center 2))
            (send bbox-msg :dimensions :x (elt dimensions 0))
            (send bbox-msg :dimensions :y (elt dimensions 1))
            (send bbox-msg :dimensions :z (elt dimensions 2))
            (ros::publish "/robot_bbox" bbox-msg)
            (send bbox-msg-array :boxes (list bbox-msg))
            (send bbox-msg-array :header (send bbox-msg :header))
            (ros::publish "/robot_bbox_array" bbox-msg-array)))
        ))
    (when *use-x*
      (x::window-main-one))
    ))

(ros::advertise "/robot_bbox" jsk_recognition_msgs::BoundingBox)
(ros::advertise "/robot_bbox_array" jsk_recognition_msgs::BoundingBoxArray) ;for visualization
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-states-callback)

(ros::rate 10)
(ros::spin)

