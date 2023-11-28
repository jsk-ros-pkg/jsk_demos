#!/usr/env/bin roseus
(setq *speech-recognition-service* t)
;; for subscribing
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "speech_recognition_msgs")

;; for (listen-with-google)
(ros::roseus "speech_to_text")

;; global variables
(defvar *speech-recognition-topic* "/speech_to_text_mux/selected")
(defvar *julius-msg* "/speech_to_text_julius")
(defvar *google-msg* "/speech_to_text_google")
(defvar *julius-service* "speech_recognition")

(defvar *julius-words* (list "すすんで" "うけとりました" "わたしました" "おちゃとぽっきぃ"))

(defun julius-words-list(*julius-words*)
    (ros::ros-info (format nil "Julius words are ~A" *julius-words*))
)

(defun listen-with-julius()
  (ros::wait-for-service *julius-service*)
  (let
      ((req (instance speech_recognition_msgs::SpeechRecognitionRequest :init))
       res)
    (send (send req :vocabulary) :words *julius-words*)
    (setq res (ros::service-call *julius-service* req t))
    (ros::ros-info (format nil "SpeechRecognition::Julius :transcript ~A :confidence ~A" (send (send res :result) :transcript) (send (send res :result) :confidence)))
    (car (send (send res :result) :transcript))
    )
)

(defun listen-with-google()
    (let
        ((res (one-shot-subscribe *google-msg* speech_recognition_msgs::SpeechRecognitionCandidates)))
        (ros::ros-info (format nil "SpeechRecognition::Google :transcript ~A" (car (send res :transcript))))
        (car (send res :transcript))
    )
)

(defun get-text-from-speech-recognition()
    (let
        (
        (topic-name (send (one-shot-subscribe *speech-recognition-topic* std_msgs::String) :data))
        )
        (ros::ros-info (format nil "The topic of speech recognition is ~A." topic-name))
        (cond 
            ((equal topic-name *google-msg*)
                (listen-with-google)
            )
            ((equal topic-name *julius-msg*)
                (listen-with-julius)
            )
            (t (ros::ros-error "No speech recognition services are available."))
        )
    )
)

(defun wait-for-speech-trigger(word)
    (while t 
        (if (equal word (get-text-from-speech-recognition))
            (return t)
            (unix:sleep 1))
    )
)

(defun wait-for-julius-trigger(word)
    (while t 
        (if (equal word (listen-with-julius))
            (return t)
            (unix:sleep 1))
    )
)

;; main
(julius-words-list *julius-words*)
