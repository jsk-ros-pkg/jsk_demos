#!/usr/bin/env roseus

(ros::roseus-add-msgs "speech_recognition_msgs")
;;(load "black_white.l")

(defun uber-cb (msg)
  (if msg
      (setq message (elt (send msg :transcript) 0))))

(defun speech_recog ()
  (ros::roseus "uber_speech_client")
  (ros::subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates #'uber-cb)
  (setq f_w 0)
  (setq message nil)
  (ros::rate 10)
  (while (ros::ok)
    ;;(setq message (one-shot-subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates))
    ;;(setq message (elt (send message :transcript) 0))
    (when message (ros::ros-info "~A" message))
  (ros::spin-once)
  (ros::sleep)
  ))
