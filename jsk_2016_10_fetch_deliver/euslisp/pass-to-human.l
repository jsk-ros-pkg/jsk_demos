#!/usr/bin/env roseus

;;実機を動かす場合は、事前に以下の２つのファイルを実行しておく。
;;roslaunch pointcloud_screenpoint.launch　(~/semi_ws内にあるlaunchファイル)
;; /opt/ros/indigo/lib/opencv_apps/face_detection image:=/head_camera/rgb/image_raw __name:=face_detection  _debug_view:=true _use_camera_info:=true _image_transport:=theora (theoraという圧縮された画像を用いて顔認識をする。)

(load "package://fetcheus/fetch-interface.l")
(fetch-init)
(print ";; done face-init")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "opencv_apps")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "std_msgs")

(setq run-watasu t)
;;腕を上げ、ものを渡す関数。
(defun watasu ()
  (send *ri* :start-grasp)
  ;;いい感じに腕を上げる。
  (send *fetch* :angle-vector #f(0.0 39.1987 -30.6094 -80.5156 67.242 0.1699 -36.3253 50.333 0.0 0.0))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  ;;angle-vectorで送った軌道の補間が終わるまで待つ。
  (send *ri* :wait-interpolation)
  ;;理想のangle-vectorと実際のpotentio-vectorの差の大きさ(delta)が10以上になったら手を開く
  #|
  (let ((delta (v. (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)) (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))))
    (while (< delta 10)
      (setq delta (v. (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)) (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector))))
      )
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)
    )
  |#
  ;;deltaの値は以下の式で代用できるかもしれない。
  ;;(print (send *ri* :state :error-vector))

  ;;上の条件とどちらがいいか考える。
  (unix:sleep 3)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)

  ;;2秒待ち、腕を収納する。
  (unix:sleep 2)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  )


;(defun hantei (before after)
;  (and (> after (* before 0.8)) (< after (* before 1.2))))
(defmacro hantei (before after)
  `(and (> ,after (* ,before ,0.8)) (< ,after (* ,before ,1.2))))


;;今回はlook-at()のなかで呼ばれている。
;;コールバック関数。人がいると分かったらwatasu()を呼ぶ。
;;顔が写った瞬間にwatasuを実行すると誤作動する。何秒以上同じ場所に顔が写ったら人がいる、というように判断する。一回watasuが呼ばれたらもう必要ないのでフラグ変数を用意している。
(setq count 0) ;;何回連続で人が居ると判断されたか
(setq flag t) ;;複数回watasuを呼ぶのを防ぐ。一回watasuが呼ばれたらnilにする。
(setq xtemp 0) ;;直前の顔のx座標
(setq ytemp 0)
(setq wtemp 0)
(setq htemp 0)
(defun find-people (msg)
  ;;msgが存在すれば（目の前に人がいれば）whenの中身を実行する。
  ;;whenを入れないとmsgがなくなった瞬間にエラーが起きてプログラムが終了してしまう。
  (when (and flag (send msg :faces))
    ;;letだと直前に宣言した局所変数を使えない。
    (let* ((f (car (send msg :faces)))
	   (cx (send f :face :x)) (cy (send f :face :y))
	   (w (send f :face :width)) (h (send f :face :height))
	   (depth 0))
      ;;サービスを用いて顔の位置の深度情報を得る。
      (setq req (instance jsk_pcl_ros::TransformScreenpointRequest :init))
      (send req :x cx)
      (send req :y cy)
      (setq res (ros::service-call "pointcloud_screenpoint/screen_to_point" req))
      (setq depth (send res :point :z))
      ;;cx, cy, w, hの単位は全てピクセル。depthのみ単位はメートル。
      (format t "cx:~3d cy:~3d~%w:~3d h:~3d~%count:~3d~%depth:~A" cx cy w h count depth)
      ;;目の前に人が居るか判定、居たらcountに1を加える。
      (if (and (hantei xtemp cx) (hantei ytemp cy) (hantei wtemp w) (hantei htemp h) (and (< 0.8 depth) (< depth 2)))
	  (progn
	    (setq count (+ count 1))
	    (send *ri* :speak (format nil "~A" count)))
	(setq count 0))
      ;;3回以上連続で目の前に人がいればwatasu呼んでものを渡す。またフラグ変数をnilにする。
      (if (and (> count 2) run-watasu)
	  (progn (format t "watasu~%")
		 (watasu) (setq count 0) (setq flag nil) (setq flag1 nil))
	nil)
      (setq xtemp cx)
      (setq ytemp cy)
      (setq wtemp w)
      (setq htemp h)
      ))
  ;;msgがなければ（目の前に人がいなければ）unlessの中身を実行する。
  (unless (send msg :faces)
    (setq count 0))
  )
;(ros::subscribe "/face_detection/faces" opencv_apps::FaceArrayStamped #'find-people)


(setq flag1 t)
;;人の顔を見つけたらそっちを見る。
(defun look-at (msg)
  (when (and flag1 (send msg :faces))
    (let* ((f (car (send msg :faces)))
	   (cx (send f :face :x)) (cy (send f :face :y))
	   (w (send f :face :width)) (h (send f :face :height))
	   (theta-x 0) (theta-y 0) (neck-y 0) (neck-p 0))
      (setq req (instance jsk_pcl_ros::TransformScreenpointRequest :init))
      (send req :x cx)
      (send req :y cy)
      ;(print (list 'face cx cy))
      (setq res (ros::service-call "pointcloud_screenpoint/screen_to_point" req))
      (if (eq res nil)
	  ()
	(progn
	  ;(send *fetch* :angle-vector (send *ri* :state :potentio-vector))
	  ;;thetaはせいぜい-0.5~0.5[rad]くらい
					;(print (list (send res :point :x) (send res :point :y) (send res :point :z)))
	  (setq theta-x (atan (/ (send res :point :x) (send res :point :z))))
	  (setq theta-y (atan (/ (send res :point :y) (send res :point :z))))
	  (setq neck-y (send *fetch* :head :neck-y :joint-angle))
	  (setq neck-p (send *fetch* :head :neck-p :joint-angle))
	  (send *fetch* :head :neck-y :joint-angle (- neck-y (* 0.5 (rad2deg theta-x)))) ;;joint-angleの引数が正の時左を向く（引数はdeg）
	  ;;カメラのx軸正方向とneck-yの正方向が逆であることに注意。
	  (send *fetch* :head :neck-p :joint-angle (+ neck-p (* 0.5 (rad2deg theta-y))) :debug-view t) ;;joint-angleの引数が正の時下を向く（引数はdeg）
	  ;;カメラのy軸正方向とneck-pの正方向は同じ。
	  ;;(unix:sleep 5)
	  ;;
					;(format t "x:~A y:~A~%" cx cy)
					;(format t "x:~A y:~A z:~A~%" (send res :point :x) (send res :point :y) (send res :point :z))
					;(format t "neck-y:~A neck-p:~A~%" (send *fetch* :head :neck-y :joint-angle) (send *fetch* :head :neck-p :joint-angle))
					;(format t "theta-x[deg]:~A theta-y[deg]:~A~%" (rad2deg theta-x) (rad2deg theta-y))
	  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000 :head-controller)
	  (unix:usleep 500000) ;;0.5秒
	  ;;顔認識のプログラムを呼ぶ。
	  (find-people msg)
	  ;;(send *ri* :wait-interpolation)
	  )))
      )
  )

(setq main-called-flag nil)
(defun main-cb (msg)
  (unless main-called-flag
    (setq main-called-flag t)
    (format t ";; start subscribing faces~%")
    (ros::subscribe "/face_detection/faces" opencv_apps::FaceArrayStamped #'look-at)
    ))

(ros::roseus "pass_to_human")
(format t ";; wait for pointcloud_screenpoint/screen_to_point")
(ros::wait-for-service "pointcloud_screenpoint/screen_to_point")
(ros::subscribe "/go_to_room_arrived" std_msgs::string #'main-cb)
(format t ";; wait for /go_to_room_arrived")

(ros::spin)


