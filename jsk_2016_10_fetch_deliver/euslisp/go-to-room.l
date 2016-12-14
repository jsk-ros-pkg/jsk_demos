#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "go_to_room")
(setq *fetch* (fetch))
(setq *ri* (instance fetch-interface :init))

;;コールバック関数
;;msgで受け取った部屋番号の部屋の前に移動し、発話する。
;;その後トピックgo_to_room_arrivedにメッセージを出版する。
(defun go-to-room-cb
  (msg)
  (let (room)
    (setq room (send msg :data))
    (cond
;;;;;
     ((string= room "practice")  ;;動作確認用
      (print 'practice)
      (send *fetch* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))
      (send *ri* :angle-vector (send *fetch* :angle-vector) 8000)
      (send *fetch* :reset-pose)
      (send *ri* :angle-vector (send *fetch* :angle-vector) 8000))
;;;;;

     ((string= room "73A1")  ;;稲葉...73A1
      (print '73A1)
      (send *ri* :move-to (make-coords :pos #f(-1250 -675 0)
				       :rpy (float-vector pi 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (send *ri* :speak "Hello, Mr. Inaba.
                         There are delivery items."))
     
     ((string= room "73A2")  ;;岡田...73A2
      (print '73A2)
      (send *ri* :move-to (make-coords :pos #f(-1250 2500 0)
				       :rpy (float-vector pi 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (send *ri* :speak "Hello, Mr. Okada.
                         There are delivery items."))

     ((string= room "73A3")  ;;山口...73A3
      (print '73A3)
      (send *ri* :move-to (make-coords :pos #f(-1250 6300 0)
				       :rpy (float-vector pi 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (send *ri* :speak "Hello, Ms. Yamaguchi.
                         There are delivery items."))

     ((string= room "73B1")  ;;浅野...73B1
      (print '73B1)
      (send *ri* :move-to (make-coords :pos #f( -850 -125 0)
				       :rpy (float-vector 0 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (send *ri* :speak "Hello, Mr. Asano.
                         There are delivery items."))

     ((string= room "73B2")  ;;学生...73B2
      (print '73B2)
      (send *ri* :move-to (make-coords :pos #f( -850 6675 0)
				       :rpy (float-vector 0 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (send *ri* :speak "Hello, students.
                         There are delivery items."))

     ((string= room "home-point")  ;;73B2室内
      (print 'home-point)
      (send *ri* :move-to (make-coords :pos #f( 2300 7000 0)
				       :rpy (float-vector 0 0 0))
	    :frame-id "/map"
	    :no-wait nil)
      (setq count 2)
      ;;(send *ri* :speak (format nil "room ~A" room))
      (send *ri* :speak "Hello, students.
                         There are delivery items.")))
    
    (format t "Arrived at ~A.~%" room)
    (ros::advertise "go_to_room_arrived" std_msgs::string 1)
    (let ((msg_arrived (instance std_msgs::string :init)))
      (send msg_arrived :data (format nil "Arrived at ~A." room))
      (ros::ros-info "msg [~A]" (send msg_arrived :data))
      (ros::publish "go_to_room_arrived" msg_arrived)))
  )


;;トピックgo_to_roomを購読してコールバック関数go-to-room-cbを呼び出す
(ros::subscribe "go_to_room" std_msgs::string #'go-to-room-cb)

(ros::spin)
