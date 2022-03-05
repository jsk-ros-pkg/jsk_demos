(ros::roseus "kitchen_wrench_detector")

;; We cannot use (one-shot-subscribe) because we need wrench information at high frequency
;; wrench topic is published at about 20 hz, but one-shot-subscribe can be called at about 1~2 hz
(defclass kitchen-wrench-detector
  :super propertied-object
  :slots (right-wrench- left-wrench-))
(defmethod kitchen-wrench-detector
  (:init ()
    (ros::unsubscribe "/right_endeffector/wrench")
    (ros::unsubscribe "/left_endeffector/wrench")
    (ros::subscribe "/right_endeffector/wrench" geometry_msgs::WrenchStamped #'send self :rarm-cb)
    (ros::subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped #'send self :larm-cb))
  (:rarm-cb (msg)
    (setq right-wrench- msg))
  (:larm-cb (msg)
    (setq left-wrench- msg))
  (:check-touch (&key (arm :rarm) (direction :x) (threshold -15))
    "
Return t if touch is detected
"
    (let* ((wrench-msg (if (eq arm :rarm) right-wrench- left-wrench-))
           (force (send wrench-msg :wrench :force direction))
           (touch (if (> threshold 0) (> force threshold) (< force threshold))))
      (when touch
        (format t "Touch detected. Cancel moving.~%force ~A is ~A (threshold is ~A)~%"
                direction force threshold))
      touch)))

(defun cancel-motion-when-touch (&key (arm :rarm) (direction :x) (threshold -15))
  (let ((wrench-detector (instance kitchen-wrench-detector :init))
        touch)
    (ros::rate 20)
    (while (some #'identity (send *ri* :wait-interpolation nil 0.01))
      (print "loop!!")
      (ros::spin-once)
      (setq touch
            (send wrench-detector :check-touch
                  :arm arm :direction direction :threshold threshold))
      (when touch
        (send *ri* :cancel-angle-vector)
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
      )
    )
  (ros::unsubscribe "/right_endeffector/wrench")
  (ros::unsubscribe "/left_endeffector/wrench")
  )
