#!/usr/bin/env roseus

(ros::roseus "sound-direction")
(ros::roseus-add-msgs "std_msgs")

(setq *elapsed-sec* 0)
(defvar duration 300)

(defun respeaker-cb (msg)
  (let* ((deg-raw (send msg :data))
         (deg (if deg-raw (- 180.0 deg-raw))))
    (ros::ros-error (format nil "deg: ~A"  deg))
    (if deg
        (if (> *elapsed-sec* duration)
            (progn
              (setq *elapsed-sec* 0)
              (ros::ros-info "rotate to sound direction")
              (rotate-robot deg))))
    ))

(defun rotate-robot (deg)
  (send *ri* :go-pos 0 0 deg))


(ros::subscribe "/sound_direction" std_msgs::Int32 #'respeaker-cb)


(defun main ()
  (ros::rate 1)
  (while t
    (ros::spin-once)
    (setq *elapsed-sec* (+ *elapsed-sec* 1))
    (ros::ros-error (format nil "*elapsed-sec* :~A" *elapsed-sec*))
    (ros::sleep)
    (ros::rate 1)))
