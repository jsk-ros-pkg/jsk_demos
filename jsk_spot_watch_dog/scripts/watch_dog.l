#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "watch-dog")

(defvar *image-topic-name* (ros::get-param "~image_topic_name" "/spot/camera/frontleft/image"))

(defclass watch-dog-class
  :super propertied-object
  :slots ())
(defmethod watch-dog-class
  (:init ()
         (ros::subscribe *image-topic-name* sensor_msgs::Image #'send self :image-cb))
  (:image-cb (msg)
             (let ((img (instance grayscale-image :init
                                  (send msg :width)
                                  (send msg :height)
                                  (copy-object (send msg :data))))
                   (luminance 0))
               (dotimes (w (send img :width))
                 (dotimes (h (send img :height))
                   (setq luminance (+ luminance (send img :pixel w h)))))
               (setq luminance (/ luminance (* (send img :width) (send img :height))))
               (ros::ros-info "receive ~A, luminance is ~A" (send (send msg :header) :frame_id) luminance)
               t)
             )
  )

(setq m (instance watch-dog-class :init))

(ros::ros-info "start ~A" "node")

(do-until-key
 (ros::spin-once)
 )
;(ros::spin)
