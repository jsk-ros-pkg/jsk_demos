#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(ros::roseus-add-msgs "spot_behavior_manager_msgs")

(unless (boundp '*ri*)
  (spot-init))

(defvar *spot_autowalk_data_path*
      "/home/spot/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data")

(defun autowalk-data-upload ()
  (setq *path*
        (ros::get-param
         "~path"
         (format nil "~A/autowalk/eng2_73b2kitchen.walk" *spot_autowalk_data_path*)))
  (setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
  (setq *upload* (ros::get-param "~upload" t))
  ;; Upload graphnav files to the robot.
  (if *upload*  (send *ri* :upload-path *path*))
  ;; Localize the robot in the map
  (ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
  ;; (send *ri* :initial-localization-waypoint *init-waypoint*)
  (send *ri* :initial-localization-fiducial))

(defun go-to-garbagecollection-area ()
  (let ((behavior_msg nil))
    (ros::advertise "/spot_behavior_manager_demo/execute_behaviors/goal"
                    spot_behavior_manager_msgs::LeadPersonActionGoal)
    (setq behavior_msg (instance spot_behavior_manager_msgs::LeadPersonActionGoal :init))
    (send (send behavior_msg :goal) :target_node_id "eng2_GarbageCollectionArea_east")
    (ros::ros-info "Move to ~A!" (send (send behavior_msg :goal) :target_node_id))
    (send *ri* :speak-jp "ゴミ捨て場に向かいます")
    (ros::publish "/spot_behavior_manager_demo/execute_behaviors/goal" behavior_msg)))

(defun main-old ()
  ;; garbage colletion from trash cans
  (autowalk-data-upload)
  (send *ri* :navigate-to 1)
  (send *ri* :go-pos -1.4 0 -90)
  (send *ri* :sit)
  (unix::sleep 1)
  (send *ri* :navigate-to 0)
  (go-to-garbagecollection-area)
  )

(defun confirm-mark ()
  (send *spotkinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.053131 -0.029053 0.064575 -90.015045 -90.041107 -90.033264))
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (setq *mark* nil)
  (unless (boundp '*tfl*) (setq *tfl* transform-listener :init))
  (while (not *mark*)
    (setq *mark* (send *tfl* :lookup-transform "/base_link" "/trash_mark" (ros::time 0))))
  (unix::sleep 30)
  (send *spotkinova* :kinova-rest-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation)
  t)

(defun main ()
  (confirm-mark)
  
  (go-to-garbagecollection-area)
  )

;; (defun speech-recognition (text &optional (duration 5))
;;   "args: text (string)"
;;   (let ((start nil) (tm nil))
;;     (setq start (send (ros::time-now) :sec))
;;     (while (or (< duration (- tm start))
;;                (not (string= text
;;                              (elt
;;                               (send
;;                                (one-shot-subscribe
;;                                 "/speech_to_text_jp"
;;                                 speech_recognition_msgs::SpeechRecognitionCandidates)
;;                                :transcript) 0))))
;;       (setq tm (send (ros::time-now) :now)))))

;; (send *ri* :power-off)

;; (send *spotkinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.040451 -0.030273 -45.02832 -0.042175 -90.013351 -45.023102))
;; (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
;; (send *ri* :wait-interpolation nil)

(defun speech-recognition (&optional (timeout 10000))
  (let* ((message
         (one-shot-subscribe
                "/speech_to_text_jp"
                speech_recognition_msgs::SpeechRecognitionCandidates
                :timeout timeout)))
    (when message
      (setq text (elt (send message :transcript) 0))
    (ros::ros-info "~A" text)
    text)))

(defun hoge ()

  ;; (confirm-mark)
  ;; (send *ri* :stand)
  ;; (send *ri* :go-pos 0 0.8 0)
  ;; (send *ri* :sit)

  (setq lift nil)
  (while (not lift)
    (when (substringp "運んでください" (speech-recognition 100000))
      (setq lift t)))
  (send *spotkinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.040451 -0.030273 -45.02832 -0.042175 -90.013351 -45.023102))
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation nil)
  (send *ri* :start-grasp) ;; grasping

  (send *spotkinova* :init-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation nil)

  (send *ri* :speak-jp "受け取りました" :wait t)
  (send *ri* :speak-jp "次の動作に進んでも良いですか" :wait t)
  (setq move nil)
  (while (not move)
    (when (substringp "お願いします" (speech-recognition 100000))
      (setq move t)))
  ;; start lifting
  ;;(send *ri* :speak-jp "ゴミ袋をのせます" :wait t)
  (send *spotkinova* :angle-vector #f(-31.6125 63.7154 -157.63 11.7098 56.1667 -154.122 -21.4785 65.2962 -155.618 19.7338 64.1409 -158.226 -0.033203 -0.003235 0.082428 90.002762 -0.015259 0.037933))
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation nil)

  (send *spotkinova* :angle-vector #f(-31.6125 63.7154 -157.63 11.7098 56.1667 -154.122 -21.4785 65.2962 -155.618 19.7338 64.1409 -158.226 -0.033203 -0.003235 0.082428 90.002762 -45.015259 0.037933))
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation nil)
  (send *ri* :speak-jp "ゴミ袋をのせました" :wait t)

  (send *spotkinova* :kinova-rest-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
  (send *ri* :wait-interpolation nil)

  (send *ri* :speak-jp "ゴミ袋を運搬します" :wait t)
  (send *ri* :speak-jp "何か問題があったら呼び止めてください" :wait t)
  (if (wait-person 10000)
      (send *ri* speak-jp "ゴミ袋を運搬します" :wait t))
  (send *ri* :speak-jp "出発します" :wait t)
  )

(defun wait-person (wait-time)
  (setq person nil)
  (setq no-wait nil)
  (setq wait nil)
  (setq timeout wait-time)
  (setq finishtm (ros::time-now))
  (setq finishtm (ros::time+ finishtm (ros::time (/ timeout 1000.0))))
  (ros::ros-info "wait person")
  (while (not person)
    (if (< (send (ros::time- finishtm (ros::time-now)) :to-Sec) 0)
        (progn
          (ros::ros-info "timeout")
          (setq person t)
          (setq no-wait t)))
    (ros::ros-info "~A" (send (ros::time- finishtm (ros::time-now)) :to-Sec))
    (when (substringp "待って" (speech-recognition 1000))
      (setq wait t)
      (setq person t)))
  (if wait
    (send *ri* :speak-jp "少し待ちます" :wait t))
  (if no-wait
    (return-from wait-person nil))
  (ros::ros-info "11111")
  (setq confirm nil)
  (while (not confirm)
    (when (substringp "いいよ" (speech-recognition 100000))
      (setq confirm t)))
  t)

(defun in-garbage-collection-area ()
  (send *ri* :speak-jp "ゴミを持ってきました．受け取ってください" :wait t)
  (send *spotkinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.040451 -0.030273 -45.02832 -0.042175 -90.013351 -45.023102))
  (send *ri* :angle-vector (send *spotkinvoa* :angle-vector) :fast)
  (send *ri* :wait-interpolation)

  (send *ri* :speak-jp "受け取ったら受け取りましたと言ってください" :wait t)
  (send *ri* :stop-grasp)
  (send *ri* :speak-jp "腕が動くので注意してください" :wait t)
  (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) :fast)
  )

(defun go-to-home ()
  (send *ri* :speak-jp "おうちに帰ります" :wait t)
  (send *ri* :go-to-spot "eng2_73B2")
  (send *ri* :dock 520))


(defun demo ()
  (hoge)
  (send *ri* :go-to-spot "eng2_GarbageCollectionArea_east")
  (in-garbage-collection-area)
  (go-to-home)
  )
