#!/usr/bin/env roseus

(ros::load-ros-manifest "speech_recognition_msgs")

;; When a message is subscribed from topic [/go_to_point],
;; then go to the point specified in the message and speak some Japanese.
;; Currently, only "corridor" and "73B2" are supported.

(defun main ()
  ;; set places
  (ros::wait-for-service "speech_recognition")
  (let ((req (instance speech_recognition_msgs::SpeechRecognitionRequest :init)))
    (send (send req :vocabulary) :words (list "ろうか" "びーに"))
    (ros::service-call "speech_recognition" req nil))

  ;; start navigation
  (load "package://fetcheus/fetch-interface.l")
  (fetch-init)
  (ros::advertise "go_to_point_arrived" std_msgs::string 1)
  (ros::ros-info "[Get ready to subscribe: /go_to_point]")

  (send *ri* :speak-jp "どこにいきますか？" :wait t)
  (let ((points nil) (point nil))
    (print (setq points (one-shot-subscribe "speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates)))
    (setq point (car (send points :transcript)))
    (setq aaa point)
    (send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
    (send *ri* :wait-interpolation)
    (cond
     ((string= point "ろうか")
      (send *ri* :speak-jp "廊下に出ます。動きますから注意してください。" :wait t)
      (format t "廊下に出ます。動きますから注意してください。~%")
      (send *ri* :move-to (make-coords :pos #f(-1500 -3500 0)
                                       :rpy (float-vector (/ -pi 2) 0 0))
            :frame-id "/map" :no-wait nil)
      )
     ((string= point "びーに")
      (send *ri* :speak-jp "JSKへようこそ。ななさんびーに、に移動します。" :wait t)
      (format t "JSKへようこそ。ななさんびーに、に移動します。~%")
      (send *ri* :move-to (make-coords :pos #f(2300 7000 0)
                                       :rpy (float-vector pi 0 0))
            :frame-id "/map" :no-wait nil)
      ))

    (let ((msg_arrived (instance std_msgs::string :init)))
      (send msg_arrived :data (format nil "Arrived at ~A." point))
      (ros::ros-info "[~A]" (send msg_arrived :data))
      (ros::publish "go_to_point_arrived" msg_arrived)))
  )
