#!/usr/bin/env roseus

(ros::roseus "parrot_node")

(load "package://pr2eus/speak.l")


(defclass parrot-subscriber
    :super propertied-object
  :slots (last-message))

(defmethod parrot-subscriber
    (:init () (ros::subscribe "/speech_recognition_candidates_to_string/output"
                              std_msgs::string #'send self :string-cb 1))
  (:string-cb (msg)
              (ros::ros-info (format nil "received: ~A" (send msg :data)))
              (setq last-message (send msg :data)))
  (:last-message ()
             last-message)
  (:clear ()
          (setq last-message nil)))
(setq ps (instance parrot-subscriber :init))
(send ps :clear)

(ros::rate 10)
(while (null (send ps :last-message))
    (ros::spin-once)
    (when (send ps :last-message)
      (speak-jp (format nil "~Aと言いましたか？" (send ps :last-message)))
      (send ps :clear)))
