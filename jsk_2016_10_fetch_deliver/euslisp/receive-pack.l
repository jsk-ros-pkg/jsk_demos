#!/usr/bin/env roseus

;;(load "package://fetcheus/fetch-interface.l")
;;(fetch-init t)

(ros::load-ros-manifest "roseus")

(ros::roseus "inkan")
(ros::advertise "pick_up" std_msgs::string 1)

(defun inkan nil
  (send *ri* :angle-vector (send *fetch* :reset-pose))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :effort 30)

  ;;印鑑を握る
  (send *ri* :angle-vector #f(19.8442 -75.8265 1.95537 -100.456 81.2313 1.82956 112.141 76.7399 0.0 0.0))
  (send *ri* :wait-interpolation)

  (send *ri* :angle-vector #f(19.8442 -73.5364 2.84919 -100.313 95.7357 2.19163 100.054 76.3608 0.0 0.0))
  (send *ri* :wait-interpolation)

  ;;印鑑を持ち上げる
  (send *ri* :start-grasp :effort 50)
  (unix:sleep 1)
  (send *ri* :angle-vector-raw #f(19.8442 -75.3974 -6.85464 -108.317 91.8183 -8.54583 99.3381 68.8498 0.0 0.0) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-raw #f(19.8442 -77.8321 -6.77513 -107.537 77.3032 -8.10757 111.531 67.5958 0.0 0.0) 5000)
  (send *ri* :wait-interpolation)
  
  ;;印鑑を押す
  (send *ri* :angle-vector
  	(send *fetch* :inverse-kinematics
  	      (make-coords :pos #f(800 0 1000))) 5000)
  (send *ri* :wait-interpolation)  

  (send *ri* :speak "Please set the paper." :wait t)
  
  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(800 0 900))) 500)
  (send *ri* :wait-interpolation)

  (send *ri* :angle-vector
	(send *fetch* :inverse-kinematics
	      (make-coords :pos #f(800 0 1000))) 500)
  (send *ri* :wait-interpolation)
  (send *ri* :speak "Thank you." :wait t)
  

  ;;印鑑を元に戻す
  (send *ri* :angle-vector #f(19.8442 -75.3974 -6.85464 -108.317 91.8183 -8.54583 99.3381 68.8498 0.0 0.0))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-raw #f(19.8442 -74.0122 -0.183408 -102.661 95.0663 -1.2509 99.6699 74.272 0.0 0.0) 5000)
  (send *ri* :wait-interpolation)
  
  (send *ri* :stop-grasp :effort 30)
  (unix:sleep 1)

  (send *ri* :angle-vector-raw #f(19.8442 -77.819 -1.21048 -102.294 73.5982 -1.50427 117.202 74.1464 0.0 0.0))
  (send *ri* :wait-interpolation)

  ;;腕をしまう
  (send *ri* :angle-vector
	(send *fetch* :reset-pose) 3000)
  (send *ri* :wait-interpolation)
  
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "go"))
  (ros::ros-info "inkan finished")
  (ros::publish "pick_up" msg))

;;トピック"delivery"でメッセージを受信した時のコールバック関数
;;メッセージの内容が"ok"ならば、印鑑を押すフェーズへ
(defun task2-cb
  (msg)
  (let (delivery)
    (setq delivery (send msg :data))
    (print delivery)
    (if (string= delivery "ok")
	(inkan))))

(ros::subscribe "delivery" std_msgs::string #'task2-cb)
(ros::spin)
