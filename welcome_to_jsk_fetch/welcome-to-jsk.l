#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(ros::load-ros-manifest "opencv_apps")

;; When a message is subscribed from topic [/go_to_point],
;; then go to the point specified in the message and speak some Japanese.
;; The message type is std_msgs/String.
;; Currently, only "corridor" and "73B2" are supported.
(defun go-to-point (msg)
  (let (point)
    (setq point (send msg :data))

    (send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
    (send *ri* :wait-interpolation)

    (cond
     ((string= point "corridor")
      (send *ri* :speak-jp "廊下に出ます。動きますから注意してください。")
      (unix::sleep 5)
      (send *ri* :move-to (make-coords :pos #f(-1500 -3500 0)
                                       :rpy (float-vector (/ -pi 2) 0 0))
            :frame-id "/map" :no-wait nil)
      )

     ((string= point "73B2")
      (send *ri* :speak-jp "JSKへようこそ。説明会の部屋に移動します。")
      (send *ri* :move-to (make-coords :pos #f(2300 7000 0)
                                       :rpy (float-vector pi 0 0))
            :frame-id "/map" :no-wait nil)
      (send *ri* :speak-jp "説明会の部屋はこちらです。どうぞお入りください。")
      ))

    (let ((msg_arrived (instance std_msgs::string :init)))
      (send msg_arrived :data (format nil "Arrived at ~A." point))
      (ros::ros-info "[~A]" (send msg_arrived :data))
      (ros::publish "go_to_point_arrived" msg_arrived)))
  )

(defun guide-pose ()
  (send *ri* :start-grasp)
  (send *fetch* :reset-pose)
  (send *fetch* :torso :waist-z :joint-angle 350)
  (send *fetch* :inverse-kinematics (make-coords :pos #f(500 -500 900) :rpy (float-vector -pi/2 0 0)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  (send *ri* :wait-interpolation))

(defun face-cb (msg)
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (let ((faces (send msg :faces)) (nearest-face nil) (face-x nil) (face-y nil))
    (if (null faces)
      (format t "do not find faces~%")
      (progn
        (format t "find faces~%")
        (setq face-x (send (send (car faces) :face) :x))
        (setq face-y (send (send (car faces) :face) :y))

        (send *fetch* :head :neck-y :joint-angle (- (send *fetch* :head :neck-y :joint-angle) (/ (max -40 (min 40 (- face-x 320))) 5.0)))
        (send *fetch* :head :neck-p :joint-angle (+ (send *fetch* :head :neck-p :joint-angle) (/ (max -20 (min 20 (- face-y 240))) 5.0)))
        ))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 300)
    (send *ri* :wait-interpolation)))

(defun guide ()
  (ros::subscribe "face_detection/faces" opencv_apps::FaceArrayStamped #'face-cb)
  (let ((i 0))
    (do-until-key
     (ros::spin-once)
     (when (eq i 7)
       (send *ri* :speak-jp "JSK見学はこちらになります。")
       (setq i 0))
     (setq i (+ i 1))
     (unix:usleep (* 1000 10)))
    ))

(defun intro ()
  (send *ri* :speak-jp "稲葉・岡田研究室では、ロボットを作って、動かし、そこに知性を吹き込むソフトとハードのシステム技術を学び、３年または６年後にロボット技術者、研究者として世界に羽ばたくことを期待しています。"))

(defun give-bag ()
  (send *ri* :stop-grasp :wait t)
  (send *fetch* :reset-pose)
  (send *fetch* :torso :waist-z :joint-angle 350)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(400 -120 1090) :rpy (float-vector -2 -1.5 -0.5)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(0 -500 1000) :rpy (float-vector 0 pi pi)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (intro)

  (send *ri* :angle-vector-sequence
        (list
         (send *fetch* :inverse-kinematics (make-coords :pos #f(-35 -350 1050) :rpy (float-vector (* pi 0.7) 0 0)) :use-torso nil)
         (send *fetch* :inverse-kinematics (make-coords :pos #f(-35 -250 1050) :rpy (float-vector pi/2 0 0)) :use-torso nil))
        (list 3000 1500))
  (unix:sleep 7)
  (send *ri* :start-grasp :wait t)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(-40 -260 1200) :rpy (float-vector pi/2 0 0)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :wait t)
  (send *ri* :start-grasp :wait t)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(0 -350 1200) :rpy (float-vector (* pi 0.7) 0 0)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(0 -500 1200) :rpy (float-vector 0 pi pi)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (make-coords :pos #f(600 0 900)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)

  (send *ri* :speak-jp "本日の資料です。周りの皆さんに配ってください。")
)


(defun nav ()
  (ros::advertise "go_to_point_arrived" std_msgs::string 1)
  (ros::ros-info "[Get ready to subscribe: /go_to_point]")

  (ros::subscribe "go_to_point" std_msgs::string #'go-to-point)
  (ros::spin)
  )

(defun init()
  (load "package://fetcheus/fetch-interface.l")
  (fetch-init)
)
