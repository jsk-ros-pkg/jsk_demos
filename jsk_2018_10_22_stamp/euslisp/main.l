#!/usr/bin/env roseus

(load "package://jsk_2018_10_22_stamp/euslisp/ros-utils.l")
(ros::load-ros-manifest "jsk_2018_10_22_stamp")
(ros::load-ros-manifest "speech_recognition_msgs")


(defun setup ()
  ;; (load "package://pr2eus/pr2-interface.l")
  ;; (pr2-init)
  ;; (setq *robot* *pr2*)
  (load "package://iory_robot/euslisp/robot.l")
  (irobot-init :robot "pr2")
  (when (not (boundp '*tfl*))
    (setq *tfl* (instance ros::transform-listener :init))))


(defun listen (&key (timeout 10000))
  (let (msg)
    (setq msg
          (one-shot-subscribe
           "/speech_to_text"
           speech_recognition_msgs::SpeechRecognitionCandidates
           :timeout 10000))
    (if msg
        (car (send msg :transcript))
      nil)))


(defun fullbody-inverse-kinematics
    (rarm-target-coords larm-target-coords)
  (let* ((target-coords
          (list
           rarm-target-coords
           larm-target-coords))
         (limbs
          (list (send *robot* :rarm :end-coords)
                (send *robot* :larm :end-coords)
                )))
    (send *robot*
          :inverse-kinematics target-coords
          :link-list (mapcar #'(lambda (limb) (send *robot* :link-list (send (send *robot* limb :end-coords) :parent))) (list :rarm :larm))
          :move-target (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) (list :rarm :larm)))))


(defun default-pose (&key (time 5000) (wait-interpolation t) (run t))
  (send *robot* :angle-vector
        #f(49.9598 44.8661 -20.138 64.3855 -66.6302 64.6722 -114.587 213.391 -44.8661 -20.138 -64.3855 -66.6302 -64.6722 -114.587 213.391 -0.005299 52.362))
  (when run
    (when (boundp '*ri*)
      (send *ri* :angle-vector (send *robot* :angle-vector) time))
    (when wait-interpolation
      (send *ri* :wait-interpolation))))


(defun lookup-transform (src-frame target-frame &key (stamp) (timeout 2.0))
  (let ((stamp-now (if stamp stamp (ros::time-now)))
        ret)
    (setq ret (send *tfl* :wait-for-transform src-frame target-frame stamp-now timeout))
    (when (null ret)
      (return-from lookup nil))
    (send *tfl* :lookup-transform src-frame target-frame stamp-now)))


(defun look-at-hanko-direction ()
  (let (i)
    (send *robot* :head :neck-p :joint-angle 20)
    (send *robot* :rarm
          :inverse-kinematics
          (send (send (send (send *robot* :narrow_stereo_r_stereo_camera_optical_frame_lk) :copy-worldcoords) :translate #f(65 0 150) :local) :rotate pi/2 :y)
          :revert-if-fail t)
    (ir2ri :wait-interpolation t)))


(defun pick-hanko ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        target-coords
        target-name
        av)
    (setq target-name "hanko_case")
    (setq av nil)
    (setq msg nil)
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (default-pose :wait-interpolation t :time 2000)
    (speak-jp "ハンコを探します。")
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                              (find target-name (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (speak-jp "ハンコが見つかりません。"))
    (speak-jp "ハンコが見つかりました。")

    (setq index (position target-name (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send (send world-to-base-transform :copy-worldcoords) :transform (ros::tf-pose->coords pose)))

    ;; pre-grasp pose
    (send *ri* :stop-grasp :rarm)

    (setq target-coords
          (send
           (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate 0 :y)
           :translate #f(-20 10 0) :local))
    (send target-coords :translate
          (float-vector 0 0 (- 800 (elt (send target-coords :worldpos) 2))) :world)
    (send *robot* :rarm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)
          :rotation-axis :z
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :rarm :inverse-kinematics (send (send target-coords :copy-worldcoords) :translate #f(0 0 -50) :world)
          :rotation-axis :z)
    (ir2ri :wait-interpolation t :time 2000)
    (send *ri* :start-grasp :rarm)

    (unix::usleep (* 1000 1000))
    (send *ri* :stop-grasp :rarm)
    (send *robot* :rarm :inverse-kinematics (send (send target-coords :copy-worldcoords) :translate #f(0 0 0) :world)
          :rotation-axis :z)
    (send *robot* :rarm :move-end-pos #f(10 0 0) :local)
    (ir2ri :wait-interpolation t :time 2000)

    ;; (send *robot* :rarm :inverse-kinematics (send (make-coords :pos #f(500 -300 740)) :rotate pi/2 :y))
    ;; (ir2ri :wait-interpolation t)
    (send *ri* :start-grasp :rarm)
    ;; force grasp
    ;; (send *ri* :move-gripper :rarm 0.0 :effort 100 :wait t)
    (setq angle (send *ri* :start-grasp :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "掴むのに失敗しました")
      (send *ri* :stop-grasp :rarm)
      (return-from pick-hanko nil))

    ;; pre-grasp pose
    (send *ri* :stop-grasp :larm)

    (send *robot* :rarm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (setq angle (send *ri* :start-grasp :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from pick-hanko nil))

    (send *robot* :rarm :inverse-kinematics
          (send (send (send (send *robot* :base_link_lk :copy-worldcoords) :translate #f(700 0 1000)) :rotate pi/2 :z) :rotate (/ -pi 4) :z))
    (send *robot* :look-at-hand :rarm)
    (send *robot* :larm :inverse-kinematics
          (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :rotate -pi/2 :z) :translate #f(0 0 -20) :world))
    (setq av (send *robot* :angle-vector))
    (send *robot* :larm :move-end-pos #f(-100 0 0) :local)
    (send *robot* :look-at-hand :rarm)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :angle-vector av)
    (send *robot* :larm :move-end-pos #f(30 0 0) :local)
    (ir2ri :wait-interpolation t :time 2000)

    ;;

    (send *robot* :larm :move-end-pos #f(0 20 0) :local)
    (send *robot* :look-at-hand :rarm)
    (send *robot* :larm :move-end-rot 30 :x :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :larm :move-end-rot 30 :x :local)
    (send *robot* :larm :move-end-pos #f(0 30 0) :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :larm :move-end-rot 30 :x :local)
    (send *robot* :larm :move-end-pos #f(0 30 40) :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :larm :move-end-pos #f(0 0 -30) :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :larm :move-end-pos #f(-200 0 0) :local
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (unix::usleep (* 1000 2000))
    (setq angle (send *ri* :start-grasp :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from pick-hanko nil))

    t))


(defun push-shuniku ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        target-coords
        (target-name "shuin"))
    (setq msg nil)
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (default-pose :wait-interpolation t)
    (speak-jp "朱肉を探します。")
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                              (find target-name (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (speak-jp "朱肉が見つかりません。"))
    (speak-jp "朱肉が見つかりました。")

    (setq index (position target-name (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send world-to-base-transform :transform (ros::tf-pose->coords pose)))
    (da)
    (draw2 world-to-target-transform)

    ;; force grasp
    (send *ri* :move-gripper :rarm 0.0 :effort 10 :wait t)

    (setq target-coords
          (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y))
    (send target-coords :translate #f(0 0 20) :world)
    (send *robot* :rarm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)
          :rotation-axis :x
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :rarm :inverse-kinematics target-coords)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :start-grasp :rarm)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from push-shuniku nil))

    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)

    (setq angle (send *ri* :start-grasp :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from push-shuniku nil))
    t))




(defun open-shuniku-case ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        target-coords
        arm
        pre-av
        av)
    (setq msg nil)
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (default-pose :wait-interpolation t :time 3000)
    (speak-jp "朱肉ケースを探します。")
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                              (find "shuin_case" (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (speak-jp "朱肉ケースが見つかりません。"))
    (speak-jp "朱肉ケースが見つかりました。")

    (setq index (position"shuin_case" (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send world-to-base-transform :transform (ros::tf-pose->coords pose)))

    ;; for rarm
    (setq target-coords
          (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 20 0) :world))
    (send target-coords :translate
          (float-vector 0 0 (- 740 (elt (send target-coords :worldpos) 2))) :world)

    (setq arm :rarm)
    (setq pre-av (send *robot* arm :inverse-kinematics
                       (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)))
    (setq av (send *robot* arm :inverse-kinematics target-coords
                   :rotation-axis :x))
    (when (or (null pre-av) (null av))
      (setq arm :larm)
      (default-pose :run nil)
      (setq target-coords
            (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 -20 0) :world))
      (send target-coords :translate
            (float-vector 0 0 (- 740 (elt (send target-coords :worldpos) 2))) :world)
      (setq pre-av (send *robot* arm :inverse-kinematics
                         (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)))
      (setq av (send *robot* arm :inverse-kinematics target-coords
                     :rotation-axis :x))
      (when (or (null pre-av) (null av))
        (speak-jp "ikが解けていないです")
        (return-from open-shuniku-case nil)))

    ;; pre-grasp pose
    (send *ri* :stop-grasp arm)

    (send *robot* :angle-vector pre-av)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :angle-vector av)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :start-grasp arm)
    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "掴むのに失敗しました")
      (send *ri* :stop-grasp arm)
      (return-from open-shuniku-case nil))

    (send *robot* arm :move-end-pos #f(0 0 100) :world
          :revert-if-fail nil)
    (ir2ri :time 2000 :wait-interpolation t)

    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "落とした気がします")
      (return-from open-shuniku-case nil))

    (send *robot* arm :move-end-pos #f(0 200 0) :world
          :revert-if-fail nil)
    (ir2ri :time 2000 :wait-interpolation t)
    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "落とした気がします")
      (return-from open-shuniku-case nil))
    (send *robot* arm :move-end-pos #f(0 0 -130) :world)
    (ir2ri :time 3000 :wait-interpolation t)
    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (send *ri* :stop-grasp)
    (send *robot* arm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)
    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    t))


(defun return-hanko-to-case ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        target-coords
        target-name)
    (default-pose :wait-interpolation t :time 2000)
    (setq msg nil)
    (setq target-name "hanko_case")
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (speak-jp "ハンコケースを探します。")
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                              (find target-name (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (when (null msg)
        (speak-jp "ハンコケースが見つかりません。")))
    (speak-jp "ハンコケースが見つかりました。")

    (setq index (position target-name (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send (send world-to-base-transform :copy-worldcoords) :transform (ros::tf-pose->coords pose)))
    (da)
    (draw2 world-to-target-transform)

    (setq target-coords
          (send
           (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y)
           :translate #f(-20 10 0) :local))
    (send target-coords :translate
          (float-vector 0 0 (- 790 (elt (send target-coords :worldpos) 2))) :world)
    (send *robot* :rarm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)
          :rotation-axis :x
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :rarm :inverse-kinematics target-coords
          :rotation-axis :x)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :stop-grasp :rarm)

    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)
    t))


(defun close-shuniku-case ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        pre-av
        av
        target-coords
        target-name
        arm)
    (setq target-name "shuin_case")
    (setq msg nil)
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (default-pose :wait-interpolation t :time 3000)
    (speak-jp "朱肉ケースを探します。")
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                              (find target-name (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (speak-jp "朱肉ケースが見つかりません。"))
    (speak-jp "朱肉ケースが見つかりました。")

    (setq index (position target-name (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send world-to-base-transform :transform (ros::tf-pose->coords pose)))

    (setq target-coords
          (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 20 0) :world))
    (send target-coords :translate
          (float-vector 0 0 (- 725 (elt (send target-coords :worldpos) 2))) :world)

    (setq arm :rarm)
    (setq pre-av (send *robot* arm :inverse-kinematics
                       (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)))
    (setq av (send *robot* arm :inverse-kinematics target-coords
                   :rotation-axis :x))
    (when (or (null pre-av) (null av))
      (setq arm :larm)
      (default-pose :run nil)
      (setq target-coords
            (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 -20 0) :world))
      (send target-coords :translate
            (float-vector 0 0 (- 725 (elt (send target-coords :worldpos) 2))) :world)
      (setq pre-av (send *robot* arm :inverse-kinematics
                         (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)))
      (setq av (send *robot* arm :inverse-kinematics target-coords
                     :rotation-axis :x))
      (when (or (null pre-av) (null av))
        (speak-jp "ikが解けていないです")
        (return-from close-shuniku-case nil)))

    ;; pre-grasp pose
    (send *ri* :stop-grasp arm)
    (send *robot* :angle-vector pre-av)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :angle-vector av)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :start-grasp arm)
    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "掴むのに失敗しました")
      (send *ri* :stop-grasp arm)
      (return-from close-shuniku-case nil))

    (send *robot* arm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 1000 :wait-interpolation t)

    (setq angle (gripper-angle arm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "落とした気がします")
      (return-from close-shuniku-case nil))

    (default-pose :wait-interpolation t :time 2000)

    (setq target-name "shuin")
    (speak-jp "朱肉を探します。")
    (setq msg nil)
    (setq world-to-base-transform (send *robot* :copy-worldcoords))
    (while (null msg)
      (setq msg (one-shot-subscribe-with-condition
                 "/label_with_geometry_filter/output"
                 jsk_2018_10_22_stamp::LabelWithGeometry
                 :timeout 5000
                 :condition #'(lambda (m)
                                (find target-name (send-all (send (send m :labelarray) :labels) :name) :test #'string=))))
      (speak-jp "朱肉が見つかりません。"))
    (speak-jp "朱肉が見つかりました。")

    (setq index (position target-name (send-all (send (send msg :labelarray) :labels) :name) :test #'string=))
    (setq pose (elt (send (send msg :posearray) :poses) index))

    (setq world-to-target-transform
          (send world-to-base-transform :transform (ros::tf-pose->coords pose)))

    (if (eq arm :rarm)
        (setq target-coords
              (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 20 0) :world))
      (setq target-coords
            (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 -20 0) :world)))
    (send target-coords :translate
          (float-vector 0 0 (- 725 (elt (send target-coords :worldpos) 2))) :world)
    (send *robot* arm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world)
          :rotation-axis :x
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* arm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 10) :world)
          :rotation-axis :x)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* arm :inverse-kinematics target-coords
          :rotation-axis :x)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* arm :move-end-pos #f(0 0 20) :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *robot* arm :move-end-pos #f(0 0 -40) :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *ri* :stop-grasp arm)

    (send *robot* arm :move-end-pos #f(0 0 200) :world)
    (ir2ri :wait-interpolation t :time 2000)
    (default-pose :wait-interpolation t :time 3000)
    t))


(defun people-pose-condition (msg)
  (let ((candidate-poses)
        limb-name
        pose
        base-to-camera-transform
        world-to-target-transform
        world-to-target-translation)
    (setq base-to-camera-transform
          (lookup-transform "base_footprint" (send msg :header :frame_id)))
    (when (null base-to-camera-transform)
      (return-from people-pose-condition nil))
    (dolist (person-pose (send msg :poses))
      (dotimes (i (length (send person-pose :limb_names)))
        (setq limb-name (elt (send person-pose :limb_names) i))
        (setq pose (elt (send person-pose :poses) i))
        (when (string= limb-name "RHand8")
          (setq world-to-target-transform
                (send (send base-to-camera-transform :copy-worldcoords)
                      :transform (ros::tf-pose->coords pose)))
          (setq world-to-target-translation (send world-to-target-transform :worldpos))
          (format t "~A~%" world-to-target-translation)
          (when (and (< 200 (elt world-to-target-translation 0))
                     (< (elt world-to-target-translation 0) 1000)
                     (< 500 (elt world-to-target-translation 2))
                     (< (elt world-to-target-translation 2) 1000))
            (setq candidate-poses (append candidate-poses (list (cons limb-name pose))))))))
    candidate-poses))

;; (do-until-key (push-hanko-with-interactive))
(defun push-hanko-with-interactive ()
  (let (text)
    (speak-jp "ここでいいですか？" :wait t)
    (setq text (listen))
    (cond ((string= text "いいよ")
           (speak-jp "では、押しますね。" :wait t)
           t)
          ((or (string= text "全然ダメ")
               (string= text "全然")
               (string= text "ダメ"))
           (speak-jp "分かりました。もう一度、押すところを指差てもらえますか")
           "retry")
          ((string= text "前")
           (speak-jp "前ですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(30 0 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "もう少し前")
           (speak-jp "もう少し前ですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(10 0 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "後ろ")
           (speak-jp "後ろですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(-30 0 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "もう少し後ろ")
           (speak-jp "もう少し後ろですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(-10 0 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "右")
           (speak-jp "右ですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(0 30 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "もう少し右")
           (speak-jp "もう少し右ですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(0 10 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "左")
           (speak-jp "左ですね。" :wait t)
           (send *robot* :rarm :move-end-pos #f(0 -30 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           nil)
          ((string= text "もう少し左")
           (send *robot* :rarm :move-end-pos #f(0 -10 0) :world
                 :rotation-axis :x
                 :revert-if-fail nil)
           (speak-jp "もう少し左ですね。" :wait t)
           nil)
          (t
           (speak-jp "聞き取れませんでした。" :wait t)
           nil))))


(defun push-hanko (&key (interactive nil))
  (let ((msg)
        av
        target-coords
        base-to-camera-transform
        world-to-base-transform
        world-to-target-transform)
    (default-pose :wait-interpolation nil :run nil)
    (send *robot* :head :neck-p :joint-angle 30)
    (ir2ri :wait-interpolation nil :time 1000)
    (speak-jp "押すところを指差てもらえますか" :wait t)

    (unix::usleep (* 1000 2000))

    (setq msg (one-shot-subscribe-with-condition
               "/people_pose_estimation_2d/pose"
               jsk_recognition_msgs::PeoplePoseArray
               :timeout 10000
               :condition #'people-pose-condition))
    (when (null msg)
      (speak-jp "見つかりませんね" :wait t)
      (return-from push-hanko nil))
    (setq camera-to-target-transform
          (ros::tf-pose->coords (cdr (car (people-pose-condition msg)))))
    (setq world-to-base-transform (send *robot* :worldcoords))
    (setq base-to-camera-transform (lookup-transform "base_footprint" (send msg :header :frame_id)))

    (setq world-to-target-transform
          (send (send world-to-base-transform :copy-worldcoords)
                :transform
                (send
                 (send base-to-camera-transform :copy-worldcoords)
                 :transform (send camera-to-target-transform :copy-worldcoords))))
    (da)
    (draw2 world-to-target-transform)

    (speak-jp "了解しました。")

    (setq target-coords
          (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y :local))

    (send target-coords :translate #f(0 0 10) :world)
    (send *robot* :rarm :inverse-kinematics
          target-coords
          :rotation-axis :x
          :revert-if-fail nil)
    (setq av (send *robot* :angle-vector))
    (send *robot* :rarm :move-end-pos #f(-100 0 0) :local
          :rotation-axis :x
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (when interactive
      (send *robot* :head :neck-p :joint-angle 90)
      (ir2ri :wait-interpolation t :time 1000)
      (let (interactive-result
            (flag nil))
        (while (null flag)
          (setq interactive-result
                (push-hanko-with-interactive))
          (cond ((string= interactive-result "retry")
                 (return-from push-hanko (push-hanko :interactive t)))
                (interactive-result
                 (setq flag t)
                 (send *robot* :rarm :move-end-pos #f(100 0 0) :local
                       :rotation-axis :x
                       :revert-if-fail nil)
                 (setq av (send *robot* :angle-vector)))
                ((null interactive-result)
                 (ir2ri :wait-interpolation t :time 1000))))))

    (send *robot* :angle-vector av)
    (send *robot* :look-at-hand :rarm)
    (ir2ri :wait-interpolation t :time 2000)
    (send *robot* :angle-vector av)
    (send *robot* :look-at-hand :rarm)
    (send *robot* :rarm :move-end-rot 2 :y :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *robot* :angle-vector av)
    (send *robot* :look-at-hand :rarm)
    (send *robot* :rarm :move-end-rot -2 :y :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *robot* :angle-vector av)
    (send *robot* :look-at-hand :rarm)
    (send *robot* :rarm :move-end-rot 2 :y :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *robot* :angle-vector av)
    (send *robot* :look-at-hand :rarm)
    (send *robot* :rarm :move-end-rot -2 :y :local)
    (ir2ri :wait-interpolation t :time 1000)

    (send *robot* :rarm :move-end-pos #f(-200 0 0) :local
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    t))



(defun gripper-angle (&optional (arm :arms))
  (case arm
        (:rarm (* 1000.0 (send (send (*ri* . r-gripper-action) :get-result) :position)))
        (:larm (* 1000.0 (send (send (*ri* . l-gripper-action) :get-result) :position)))
        (:arms (list
                (* 1000.0 (send (send (*ri* . r-gripper-action) :get-result) :position))
                (* 1000.0 (send (send (*ri* . l-gripper-action) :get-result) :position))))))


(defun pick-hanko-pose ()
  (let ((angle)
        (flag t))
    (while t
      (send *ri* :move-gripper :rarm 0.02 :effort 100 :wait t)
      (if flag
          (speak-jp "ハンコを落としてしまったので、拾って渡してください。" :wait t)
        (speak-jp "もう一度はんこを渡してください。" :wait t))
      (send *robot* :rarm :angle-vector #f(-22.2219 -13.4444 -71.2395 -73.7222 -108.101 -23.7166 204.513))
      (ir2ri :time 2000)
      (send *ri* :wait-interpolation)

      (unix::usleep (* 1000 1000))
      (speak-jp "腕をふってください。" :wait t)
      (wait-for-hand-impact :rarm)
      (speak-jp "掴みますよ。" :wait t)
      (unix::usleep (* 1000 1000))
      (setq angle (send *ri* :start-grasp :rarm))

      (unix::usleep (* 1000 1000))

      (when (> angle 5)
        (speak-jp "ありがとうございます。")
        (return-from pick-hanko-pose t))
      (setq flag nil))))


(defun run ()
  (let (result)
    (setq result
          (open-shuniku-case))
    (when (null result)
      (speak-jp "もう一度挑戦します。")
      (open-shuniku-case)
      (return-from run nil))
    (setq result (pick-hanko))
    (when (null result)
      (pick-hanko-pose))
    (setq result (push-shuniku))
    (when (null result)
      (pick-hanko-pose))

    (setq result (push-hanko :interactive t))
    (when (null result)
      (return-from run nil))

    (setq result (return-hanko-to-case))
    (setq result (close-shuniku-case))

    (when (null result)
      (speak-jp "もう一度挑戦します。")
      (close-shuniku-case))))

(setup)
(run)
(exit)
