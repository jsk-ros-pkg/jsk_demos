#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)
(print "done face-init")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "opencv_apps")
(ros::roseus "give")


;;腕を上げ、ものを渡す関数。
(defun watasu ()
  (send *ri* :start-grasp) ;;他のプログラムと合わせるときはいらない。
  ;;いい感じに腕を上げる。
  (send *fetch* :angle-vector #f(0.0 39.1987 -30.6094 -80.5156 67.242 0.1699 -36.3253 50.333 0.0 0.0))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  
  ;;腕が動き始めてから7秒経ち、かつ、腕が動かされると手を離す。
  ;;(unix:sleep 7)
  (send *ri* :wait-interpolation) ;; :angle-vectorで送った軌道の補間が終わるまで待つ．k-okada
  (unix:sleep 2) ;; k-okada
  ;;理想のangle-vectorと実際のpotentio-vectorの差の大きさ(delta)が10以上になったら手を開く
  (print (send *ri* :state :error-vector)) ;; 実機で何が帰ってくるかわからないけど、こういうメソッドもあるはず．k-okada
  (let ((delta (v. (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)) (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))))
    (while (< delta 10)
      (setq delta (v. (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)) (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector))))
      )
    (send *ri* :stop-grasp)
    )

  ;;2秒待ち、腕を収納する。
  (unix:sleep 2)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  )



;;コールバック関数。人がいると分かったらwatasu()を呼ぶ。
;;顔が写った瞬間にwatasuを実行すると誤作動する。何秒以上同じ場所に顔が写ったら人がいる、というように判断する。一回watasuが呼ばれたらもう必要ないのでフラグ変数を用意しておく。
(defun find-people (msg) 
  (print (send msg :faces))
  ;;/face_detection/facesを用いて顔認識をし、人がいると判断できたらwatasuを呼ぶような処理を書く。
)

(ros::subscribe "/face_detection/faces" opencv_apps::FaceArrayStamped #'find-people)

;;キー入力があると終了する。
(do-until-key
 (ros::spin-once)
)
