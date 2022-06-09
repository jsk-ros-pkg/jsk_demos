(ros::load-ros-manifest "speech_recognition_msgs")
(ros::roseus-add-srvs "speech_recognition_msgs")


(defclass speech-recognition-client
    :slots (sub speech-msg))


(defmethod speech-recognition-client
  (:init (topic-name)
   (setq sub (ros::subscribe
              topic-name speech_recognition_msgs::SpeechRecognitionCandidates
              #'send self :callback)))
  (:callback (msg)
             (setq speech-msg msg))
  (:result ()
           (when (and (not (null speech-msg)) (> (length (send speech-msg :transcript)) 0))
             (car (send speech-msg :transcript)))))
