#!/usr/bin/env roseus


(ros::roseus "listen_doa")

(ros::load-ros-manifest "jsk_hark_msgs")
(load "package://pr2eus/speak.l")


(defclass doa-subscriber
    :super propertied-object
  :slots (last-message))

(defmethod doa-subscriber
    (:init () (ros::subscribe "/doa"
                              jsk_hark_msgs::HarkPower #'send self :cb 1))
  (:cb (msg)
       (print (send msg :powers))
              (ros::ros-info (format nil "received doa resuls: theta: ~A, peakL: ~A, peakR: ~A"
                                     (send msg :directions)
                                     (elt (send msg :powers) 0)
                                     (elt (send msg :powers) 1)))
              (setq last-message msg))
  (:last-message ()
                 last-message)
  (:clear ()
          (setq last-message nil)))
(setq ds (instance doa-subscriber :init))
(send ds :clear)

(setq threshold 0.5)
(setq msg nil)
(setq theta nil)
(ros::rate 10)
(while (null (send ds :last-message))
  (ros::spin-once)
  (when (not (null (send ds :last-message)))
    (setq msg (send ds :last-message))
    (when (and (> (elt (send msg :powers) 0) threshold)
               (> (elt (send msg :powers) 1) threshold))
      (setq theta (send msg :directions))
      (cond ((< (abs theta) 20)
             (speak-jp "前方から音が聞こえました" :wait t))
            ((> theta 20)
             (speak-jp "右方向から音が聞こえました" :wait t))
            ((< theta -20)
             (speak-jp "左方向から音が聞こえました" :wait t)))
      )
    (send ds :clear)))
