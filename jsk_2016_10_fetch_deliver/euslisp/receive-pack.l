#!/usr/bin/env roseus

;(load "package://fetcheus/fetch-interface.l")
;(fetch-init t)

(ros::load-ros-manifest "roseus")

(ros::roseus "inkan")

(defun inkan nil
  ;受け取った荷物を置く
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(400 400 800))) 5000)
  (unix:sleep 7)
  (send *ri* :stop-grasp)

  ;印鑑を握る
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(450 -400 1000))) 5000)
  (unix:sleep 7)
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(450 -400 800))) 5000)
  (unix:sleep 7)
  (send *ri* :start-grasp)

  ;印鑑を押す
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(800 0 1000))) 5000)
  (unix:sleep 7)
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(800 0 950))) 5000)
  (unix:sleep 7)
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(800 0 1000))) 5000)
  (unix:sleep 7)

  ;印鑑を元に戻す
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(450 -400 800))) 5000)
  (unix:sleep 7)
  (send *ri* :stop-grasp)

  ;腕をしまう
  (send *ri* :angle-vector
	(send *fetch* :reset-pose))
  (unix:sleep 10)

  ;印鑑を押し終わったら、トピック"go_to_room"で行き先の部屋番号を伝える
  ;(部屋番号の決定方法はまだ考慮できていません(以下は73A2の例))
  (ros::advertise "go_to_room" std_msgs::string 1)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "73A2"))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "go_to_room" msg))

;トピック"recieved(要打ち合わせ)"でメッセージを受信した時のコールバック関数
;メッセージの内容が"ok"ならば、印鑑を押すフェーズへ
(defun task2-cb
  (msg)
  (let (recieved)
    (setq recieved (send msg :data))
    (print recieved)
    (if (equal recieved "ok")
	(inkan))))

(do-until-key
 (ros::subscribe "recieved" std_msgs::string #'task2-cb)
 (ros::spin())
)
