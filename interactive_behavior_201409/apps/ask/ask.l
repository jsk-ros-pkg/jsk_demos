#!/usr/bin/env roseus
;; ask.l

(ros::roseus "run_ask")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(ros::load-ros-manifest "speech_recognition_msgs")

(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun speech-callback (msg)
  (setq *answer* (car (send msg :transcript))))

(setq *answer* nil)

(defun main ()
  (let* ((person (cdr (assoc "person" *params* :test #'string=)))
         (place (person->place person))
         (content (cdr (assoc "content" *params* :test #'string=)))
         (scene (make-eng2-scene))
         start-pos
         spot)
    (pr2-init)
    (setq spot (get-spot scene *ri* place))

    (unless spot
      (ros::ros-error "Destination '~A' not found" place)
      (speak-jp "あー、ちょっとわからないですね。")
      (return-from main nil))
    (ros::ros-info "Going to ~A" (send spot :name))
    (speak-jp (format nil "~Aに行きます" (send spot :name)))
    (setq start-pos (send *ri* :state :worldcoords))
    (unless (send *ri* :move-to spot)
      (speak-jp "失敗しました。")
      (return-from main nil))
    ;;
    (speak-jp (format nil "~Aにつきました" place))
    ;;
    (speak-jp (format nil "~Aさん。聞きたいことがあります。。~A" person content))
    (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates
                    #'speech-callback)
    (ros::rate 2)
    (while (and (ros::ok) (null *answer*))
      (ros::ros-info "waiting for answer")
      (ros::spin-once)
      (ros::sleep))
    ;;
    (speak-jp "わかりました。")
    (speak-jp "元の場所に戻ります")
    (unless (send *ri* :move-to start-pos)
      (speak-jp "失敗しました。")
      (return-from main nil))
    (speak-jp (format nil "~Aさんに聞いてきました。~A、だそうです。" person *answer*))
  t)


(if (main) (exit 0) (exit 1))
