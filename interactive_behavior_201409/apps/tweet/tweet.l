#!/usr/bin/env roseus
;; tweet.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :tweet-client "package://jsk_robot_startup/lifelog/tweet_client.l")
(require :json-decode "package://roseus_mongo/euslisp/json/json-decode.l")

(ros::roseus "run_tweet")

(setq *param* (json::parse-from-string (ros::get-param "/action/parameters")))

(ros::ros-info "*param*: ~A" *param*)

;; (speak-jp "写真をとります")
;; (unix:sleep 2)
;; (speak-jp "さん")
;; (unix:sleep 1)
;; (speak-jp "に")
;; (unix:sleep 1)
;; (speak-jp "いち")
;; (unix:sleep 1)
;; (speak-jp "カシャ！")

;; (tweet-string "Hello!!"
;;               :warning-time nil
;;               :with-image "/kinect_head/rgb/image_rect_color")
(exit)
