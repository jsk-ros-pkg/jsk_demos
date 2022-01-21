;;#!/usr/bin/env roseus

;; mainly copied from jsk_demos/201910semi

(ros::roseus "trashcan-detection")
;;load packages
(load "package://fetcheus/fetch-interface.l")
;;(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
;;(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(load "move-trashcan-to-spot.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "speech_recognition_msgs")

;;defvarは一度やってしまったら上書きされない
(defvar *topic-name* "/pcl_nodelet/boxes_trashcanedge")
(defvar *gift-name* "/atohayoroshiku")
(defvar *bounding-box-list* nil)
(defvar *gift-number* nil)

;; (fetch-init)
;; (objects (list *fetch*))

(setq time-stamp (ros::time 0))

;;callback
(defun bounding-box-array-cb (msg)
  ;;(ros::ros-info "cb called")
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  ;;(ros::ros-info "set box-list")
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (when (ros::time> time-stamp (send msg :header :stamp)) ;;rostime揃え
        (return-from bounding-box-array-cb nil))
      (ros::ros-info "tf")
      (setq obj-pos (scale 0.001
                           (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

(defun grasp-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (ros::ros-info "targe-cds 1")
  (ros::ros-info (format nil "~A" target-cds))
  (send target-cds :translate #f(-200 0 50)) ;;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
  (ros::ros-info "targe-cds 2")
  (ros::ros-info (format nil "~A" target-cds))
  (objects (list *fetch* target-cds))

  (send *fetch* :inverse-kinematics target-cds)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (send target-cds :translate #f(200 0 0)) :rotation-axis :y)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation))


(defun look-at-coord (coord &optional (tm 3000) (wi nil))
    (send *fetch* :head :look-at (send coord :worldpos))
    (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
    (send *ri* :wait-interpolation wi))

(defun trashcan-grasp (obj-pos)
  (let ((cam-coords nil)
        (obj-coords (make-coords :pos (scale 1000 obj-pos) :rpy #f(0 0 0))))
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords
          (send *tfl* :lookup-transform "/base_link" "/head_camera_depth_optical_frame" (ros::time 0)))
    (send *fetch* :angle-vector #f(20.0 56.549 86.0026 -30.9913 101.496 -91.683 92.782 8.07181 0.0 0.0))
    ;; (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(400 -200 500) :rpy (float-vector 0 0 pi/2)))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send obj-coords :transform cam-coords :world)
    (send obj-coords :transform (make-coords :pos #f(0 0 100)) :world)
    (send *fetch* :head :neck-y :joint-angle -90)
    (send *fetch* :head :neck-p :joint-angle 60)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) :head-controller)
    (send *ri* :wait-interpolation)
    (send *fetch*
          :rarm :inverse-kinematics
          (make-coords :pos (send obj-coords :pos)
                       :rpy (float-vector 0 pi/2 pi/2)) :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000) ;; reaching
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *fetch* :start-grasp)
    (send *ri* :start-grasp)
    (unix::sleep 1)

    ;; TODO
    ;; trashcan-inside-detection
    (setq watching-coords
          (make-coords
           :pos (float-vector (- (elt (send obj-coords :pos) 0)
                                 (* (sin (/ pi/2 2)) (elt (send obj-coords :pos) 2)))
                              (elt (send obj-coords :pos) 1)
                              (* (cos (/ pi/2 2)) (elt (send obj-coords :pos) 2)))
           :rpy (float-vector 0 (/ pi/2 2) pi/2)))
    (send *fetch* :rarm :inverse-kinematics watching-coords :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ;; (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(400 -200 400) :rpy (float-vector 0 (/ pi/2 2) pi/2)))
    ;; #f(20.0 12.7397 58.0108 -63.9436 127.969 -161.961 123.759 -5.36722 0.0 0.0)

    (send *fetch* :move-end-pos #f(0 0 100))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *fetch* :stop-grasp)
    (send *ri* :stop-grasp)
    (unix::sleep 1)
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)))

(defun right-side(obj-pos)
  (let ((cam-coords nil)
        (obj-coords nil))
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords
          (send *tfl*
                :lookup-transform "/base_link" "/head_camera_depth_optical_frame"
                (ros::time 0)))
    (unix::sleep 2)
    ;; (send *fetch* :angle-vector #f(20.0 56.549 86.0026 -30.9913 101.496 -91.683 92.782 8.07181 0.0 0.0)) ;; (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(400 -200 500) :rpy (float-vector 0 0 pi/2)))
    ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    ;; (send *ri* :wait-interpolation)
    (setq obj-coords (make-coords :pos (scale 1000 obj-pos) :rpy #f(0 0 0)))
    (send obj-coords :transform cam-coords :world)
    (send obj-coords :transform (make-coords :pos #f(0 0 100)) :world)
    (ros::ros-info "~A" obj-coords)
    (send *fetch* :head :neck-y :joint-angle -90)
    (send *fetch* :head :neck-p :joint-angle 60)
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :inverse-kinematics
          (make-coords :pos (send obj-coords :pos)
                       :rpy (float-vector 0 pi/2 0)) :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000) ;; reaching
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :move-end-pos #f(100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
    (send *ri* :wait-interpolation)
    ;; (send *fetch* :start-grasp)
    ;; (send *ri* :start-grasp)
    (unix::sleep 1)
    (send *fetch* :rarm :move-end-rot -45 :z)
    (send *fetch* :rarm :move-end-pos #f(0 150 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000 :use-torso nil)
    (send *ri* :wait-interpolation)
    (send *ri* :speak-jp "ごみ箱の中身を確認します")
    (unix::sleep 2)

    (send *fetch* :rarm :move-end-pos #f(0 -150 0))
    (send *fetch* :rarm :move-end-rot 45 :z)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000 :use-torso nil)
    (send *ri* :wait-interpolation)

    (send *fetch* :rarm :move-end-pos #f(-100 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ))

(defun rope(obj-pos)
  (let ((cam-coords nil)
        (obj-coords nil))
    (ros::spin-once)
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords
          (send *tfl*
                :lookup-transform "/base_link" "/head_camera_depth_optical_frame"
                (ros::time 0)))
    (unix::sleep 2)
    ;; (send *fetch* :angle-vector #f(20.0 56.549 86.0026 -30.9913 101.496 -91.683 92.782 8.07181 0.0 0.0)) ;; (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(400 -200 500) :rpy (float-vector 0 0 pi/2)))
    ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    ;; (send *ri* :wait-interpolation)
    (setq obj-coords (make-coords :pos (scale 1000 obj-pos) :rpy #f(0 0 0)))
    (send obj-coords :transform cam-coords :world)
    (send obj-coords :transform (make-coords :pos #f(0 50 0)) :world)
    (ros::ros-info "~A" obj-coords)

    (send *fetch* :rarm :inverse-kinematics obj-coords :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000) ;; reaching
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :move-end-pos #f(70 0 0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)
    (send *ri* :speak-jp "ごみ袋を持ち上げます" :wait t)

    (send obj-coords :transform (make-coords :pos #f(-50 0 1300)) :world)
    (send *fetch* :rarm :inverse-kinematics obj-coords :rotation-axis nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso t)
    (send *ri* :wait-interpolation)
    (send *fetch* :torso :waist-z :joint-angle 360)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)

    (send *ri* :go-pos-unsafe 0 0 -90)
    (send *fetch* :torso :waist-z :joint-angle 0)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ))

(defun hole(obj-pos)
  (let ((cam-coords nil)
        (obj-coords nil))
    (ros::spin-once)
    (unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
    (setq cam-coords
          (send *tfl*
                :lookup-transform "/base_link" "/head_camera_depth_optical_frame"
                (ros::time 0)))
    (unix::sleep 2)
    (send *fetch* :angle-vector #f(20.0 56.549 86.0026 -30.9913 101.496 -91.683 92.782 8.07181 0.0 0.0)) ;; (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(400 -200 500) :rpy (float-vector 0 0 pi/2)))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :arm-controller)
    (send *ri* :wait-interpolation)
    
    (setq obj-coords (make-coords :pos (scale 1000 obj-pos)
                                  :rpy (float-vector 0 0 pi/2)))
    (send obj-coords :transform cam-coords :world)
    (send obj-coords :transform (make-coords :pos #f(-130 -25 30)) :world) ;; fetch1075 1020
    (setq obj-coords (make-coords :pos (send obj-coords :pos)
                                  :rpy (float-vector 0 0 pi/2)))
    (ros::ros-info "~A" obj-coords)

    (send *fetch* :rarm :inverse-kinematics obj-coords :rotation-axis t)
    (send *ri* :angle-vector (send *fetch* :angle-vector) :fast) ;; reaching
    (send *ri* :wait-interpolation)
    ;; (send *fetch* :rarm :move-end-pos #f(60 0 0)) ;; for plastic
    (send *fetch* :rarm :move-end-pos #f(45 0 0)) ;; for burnable
    (send *ri* :angle-vector (send *fetch* :angle-vector) :fast :use-torso nil)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)
    (send *ri* :speak-jp "ごみ袋を持ち上げます" :wait t)

    ;; lifting garbage bag
    (send obj-coords :transform (make-coords :pos #f(100 0 300)) :world)
    (ros::ros-info "~A" obj-coords)
    (send *fetch* :rarm :inverse-kinematics obj-coords :rotation-axis nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso t)
    (send *ri* :wait-interpolation)
    (send obj-coords :transform (make-coords :pos #f(-150 0 400)) :world)
    (ros::ros-info "~A" obj-coords)
    ;; (send obj-coords :transform (make-coords :pos #f(-50 0 700)) :world)
    (send *fetch* :rarm :inverse-kinematics obj-coords :rotation-axis nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso t)
    (send *ri* :wait-interpolation)
    (send *fetch* :torso :waist-z :joint-angle 360)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)

    ;;(send *ri* :go-pos-unsafe 0 0 -90)
    ;; (send *fetch* :torso :waist-z :joint-angle 0)
    ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    ;; (send *ri* :wait-interpolation)
    

    (send *fetch* :angle-vector #f(99.6253 -91.9143 -63.6014 -148.503 46.6524 -2.28871 25.3505 0.454471 0.492839 19.0466))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    ))

(defun hole-main()
  ;; (move-to-trashcan-front)
  (send *fetch* :reset-pose)
  (send *fetch* :head :neck-p :joint-angle 45)
  (send *ri* :angle-vector (send *fetch* :angle-vector) :fast :head-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "ゴミ箱が満杯です．回収します" :wait t)
  (unix::sleep 1)
  (ros::subscribe *topic-name*
                  jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-array-cb 1)
  (unix::sleep 1)
  (ros::spin-once)
  (hole obj-pos)
  (setq obj-pos nil)
  (send *ri* :go-pos-unsafe 0 0 -90)
  ;; (move-to-belka-front)
  
  ;; (send *fetch* :angle-vector #f(274.774 10.05741 -54.241 176.714 30.87212 -2.26673 70.3505 0.498438 -0.017 27.3303))
  ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  ;; (send *ri* :wait-interpolation)

  ;; (send *ri* :go-pos-unsafe 0.1 0 0)
  ;; (send *ri* :speak-jp "ゴミ袋を渡しました" :wait t)
  ;; (send *fetch* :angle-vector #f(204.973 -7.38554 -48.3304 -180.518 46.6744 -89.1906 -89.918 178.169 -0.130286 39.8107))
  ;; (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 10000)
  ;; (send *ri* :wait-interpolation)

  ;; (send *ri* :go-pos-unsafe -0.1 0 0)

  
  )

(defun right-side-main()
  (move-to-trashcan-front)
  (ros::subscribe *topic-name*
                  jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-array-cb 1)
  (ros::spin-once)
  (right-side obj-pos))

(defun speech-recognition (&optional (timeout 1000))
  (let* ((message
         (one-shot-subscribe
                "/speech_to_text_google"
                speech_recognition_msgs::SpeechRecognitionCandidates
                :timeout timeout)))

    (when message
      (setq text (elt (send message :transcript) 0))
    (ros::ros-info "~A" text)
    text)))

(defun giving-trashbag ()
  ;;(move-to-trash-pre-front)
  ;;(unix::sleep 3)
  (send *fetch* :angle-vector #f(304.797 50.0388 -54.1531 176.692 30.876 -2.33266 70.4823 0.520411 -0.345553 27.8796))
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "ゴミを運んでください" :wait t)
  (send *ri* :speak-jp "受け取ってください" :wait t)
  (setq f nil)
  (while (not f)
    (when (substringp "わかりました" (speech-recognition 1000))
      (setq f t)))
  (send *fetch* :angle-vector #f(306.809 29.5944 -22.4686 176.604 30.7881 -2.33266 31.4369 0.498438 9.0354 53.4338))
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "これを運んでください" :wait t)
  (setq take nil)
  (while (not take)
    (when (substringp "受け取りました" (speech-recognition 1000))
      (setq take t)))
  (send *ri* :stop-grasp)

  ;; (unless (string= "受け取りました" (speech-recognition 100000))
  ;;   (return-from giving-trashbag))
  (unix::sleep 1)
  (send *ri* :speak-jp "お願いします" :wait t)
  (send *ri* :stop-grasp)
  (setq bag nil)
  (while (not bag)
    (when (substringp "ゴミ袋" (speech-recognition 1000))
      (setq bag t)))
  (send *ri* :speak-jp "問題があったら呼び止めてください" :wait t)
  (wait-person 10000)
  ;; (send *fetch* :angle-vector #f(274.797 50.0388 -54.1531 176.692 30.876 -2.33266 70.4823 0.520411 -0.345553 27.8796))
  ;; (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 10000)
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 30000)
  (send *ri* :wait-interpolation)
  )

(defun wait-person (timeout)
  (setq person nil)
  (setq wait nil)
  (setq no-wait nil)
  (setq timeout timeout)
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
  (setq person nil)
  (if wait
    (send *ri* :speak-jp "少し待ちます" :wait t))
  (if no-wait
    (return-from wait-person nil))
  (ros::ros-info "11111")
  (setq confirm nil)
  (while (not confirm)
    (when (or
           (substringp "いいよ" (speech-recognition 100000))
           (substringp "オーケー" (speech-recognition 100000)))
      (setq confirm t)))
  (setq confirm nil))


;; #f(368.746 -11.8899 -54.5267 -180.474 46.6524 -2.35462 -54.4542 -86.0519 -0.345553 39.9206)
;; #f(368.731 -6.11112 -20.2713 -180.452 46.6524 -2.31067 -68.1212 -86.0079 -0.300235 39.8327) ;; give trashbag

;; #f(352.328 -5.8914 -3.70392 -180.012 46.8282 -2.55238 -106.661 -85.5685 -0.050987 39.7887) ;; release trashbag


;; #f(204.973 -7.38554 -48.3304 -180.518 46.6744 -89.1906 -89.918 178.169 -0.130286 39.8107) ;;pre release pose trashbag
;; #f(204.057 46.2497 -27.742 160.872 46.7403 -121.996 -111.012 -179.348 -0.062312 39.8766) ;;release
;; #f(274.774 -1.05741 -54.241 176.714 1.87212 -2.26673 25.3505 0.498438 -0.017 27.3303)
;; #f(258.661 -0.947549 71.7941 -136.484 125.249 -2.31067 42.2914 0.520411 -0.266248 41.8102)

;; send *ri* :angle-vector-raw #f(274.797 50.0388 -54.1531 176.692 30.876 -2.33266 70.4823 0.520411 -0.345553 27.8796)
;; #f(266.809 29.5944 -22.4686 176.604 30.7881 -2.33266 31.4369 0.498438 9.0354 53.4338)
