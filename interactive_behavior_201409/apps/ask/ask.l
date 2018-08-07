#!/usr/bin/env roseus
;; ask.l

(ros::roseus "run_ask")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :door-utils "package://interactive_behavior_201409/euslisp/door-utils.l")


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
  (let* ((person (get-arg :person))
         (place (person->place person))
         (content (get-arg :content))
         (scene (make-eng2-scene))
         door-handle handle-found-p
         start-pos
         spot)
    (pr2-init)
    (setq spot (get-spot place))

    (unless spot
      (ros::ros-error "Destination '~A' not found" place)
      (speak-jp "あー、ちょっとわからないですね。")
      (return-from main nil))
    (ros::ros-info "Going to ~A" (send spot :name))
    (speak-jp (format nil "~Aに行きます" (send spot :name)))
    (setq start-pos (send *ri* :state :worldcoords))
    (pr2-tuckarm-pose :rarm)
    (send *ri* :clear-costmap)
    (unless (send *ri* :move-to spot)
      (speak-jp "失敗しました。")
      (return-from main nil))
    ;;
    (speak-jp (format nil "~Aにつきました" place))
    ;;
    (dotimes (i 5)
      (setq door-handle (find-handle :arm))
      (if (null door-handle)
          (ros::ros-error "failed to find door handle")
          (when (go-to-handle door-handle)
            (setq handle-found-p t)
            (return))))
    (when handle-found-p
      (knock-door door-handle :rarm)
      (speak-jp "失礼します")
      (unless (grasp-handle :larm door-handle)
        (speak-jp "ハンドルを掴んでない気がします。")
        (setq handle-found-p nil)))
    (when handle-found-p
      (unlatch-door :larm :push 50)
      (send *ri* :go-pos-unsafe 0.1 0 0)
      (send *ri* :stop-grasp :larm))
    (unless handle-found-p
      (speak-jp "あれ、ドアノックしてないけど。まあいいか。"))
    ;;
    (speak-jp (format nil "~Aさん。聞きたいことがあります。次の質問に答えてください。。~A" person content))
    (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates
                    #'speech-callback)
    (ros::rate 2)
    (while (and (ros::ok) (null *answer*))
      (ros::ros-info "waiting for answer")
      (ros::spin-once)
      (ros::sleep))
    ;;
    (speak-jp "わかりました。" :wait t)
    (speak-jp "元の場所に戻ります")
    (speak-jp "少し後ろに下がります。" :wait t)
    (send *ri* :go-pos-unsafe -0.1 0 0)
    (when handle-found-p
      (send *ri* :go-pos-unsafe -0.1 0 0)
      (pr2-reset-pose)
      (pr2-tuckarm-pose))
    (send *ri* :clear-costmap)
    (unless (send *ri* :move-to start-pos)
      (speak-jp "失敗しました。")
      (return-from main nil))
    (speak-jp (format nil "~Aさんに聞いてきました。~A、だそうです。" person *answer*))
  t))

(defun short-test ()
  (if (not (boundp '*pr2*)) (pr2))
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))
  (if (not (boundp '*scene*)) (setq *scene* (make-eng2-scene)))
  (send *ri* :clear-costmap)
  (send *ri* :move-to (get-spot "73A3"))
  (dotimes (i 5)
    (setq door-handle (find-handle))
    (if (null door-handle)
        (ros::ros-error "failed to find door handle")
        (when (go-to-handle door-handle)
          (setq handle-found-p t)
          (return))))
  (unless handle-found-p
    (return-from short-test nil))
  (knock-door door-handle :rarm)
  ;; (grasp-handle :larm handle)
  ;; (unlatch-door :larm :push 50)
  )

(if (main) (exit 0) (exit 1))
