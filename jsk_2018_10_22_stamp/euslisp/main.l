#!/usr/bin/env roseus

(load "package://jsk_2018_10_22_stamp/euslisp/ros-utils.l")
(ros::load-ros-manifest "jsk_2018_10_22_stamp")


(defun setup ()
  ;; (load "package://pr2eus/pr2-interface.l")
  ;; (pr2-init)
  ;; (setq *robot* *pr2*)
  (load "package://iory_robot/euslisp/robot.l")
  (irobot-init :robot "pr2")
  (when (not (boundp '*tfl*))
    (setq *tfl* (instance ros::transform-listener :init))))


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


(defun default-pose (&key (time 5000) (wait-interpolation t))
  ;; (send *robot* :reset-pose)
  ;; (send *robot* :head :neck-p :joint-angle 50)
  (send *robot* :angle-vector
        #f(49.9598 59.9994 73.9966 69.9995 -119.999 20.0009 -29.9994 179.999 -44.8661 -20.138 -64.3855 -66.6302 -64.6722 -114.587 213.391 -0.005299 52.362))
  (when (boundp '*ri*)
    (send *ri* :angle-vector (send *robot* :angle-vector) time)))


(defun lookup-transform (src-frame target-frame &key (stamp) (timeout 2.0))
  (let ((stamp-now (if stamp stamp (ros::time-now)))
        ret)
    (setq ret (send *tfl* :wait-for-transform src-frame target-frame stamp-now timeout))
    (when (null ret)
      (return-from lookup nil))
    (send *tfl* :lookup-transform src-frame target-frame stamp-now)))



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
    (default-pose :wait-interpolation t :time 3000)
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
    (da)
    (draw2 world-to-target-transform)

    ;; pre-grasp pose
    (send *robot* :rarm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)
    (send *ri* :stop-grasp)

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
    (ir2ri :wait-interpolation t :time 2000)

    ;; (send *robot* :rarm :inverse-kinematics (send (make-coords :pos #f(500 -300 740)) :rotate pi/2 :y))
    ;; (ir2ri :wait-interpolation t)
    (send *ri* :start-grasp :rarm)
    ;; force grasp
    (send *ri* :move-gripper :rarm 0.0 :effort 100 :wait t)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "掴むのに失敗しました")
      (send *ri* :stop-grasp :rarm)
      (return-from pick-hanko nil))

    ;; pre-grasp pose
    (send *robot* :larm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)
    (send *ri* :stop-grasp :larm)

    (send *robot* :larm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 -50) :world)
          :rotation-axis :zy
          :revert-if-fail nil)
    (setq av (send *robot* :angle-vector))
    (send *robot* :larm :move-end-pos #f(-100 0 0) :local
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)
    (send *robot* :angle-vector av)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :start-grasp :larm)

    (send *robot* :rarm :move-end-pos #f(-30 0 0) :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :rarm :move-end-pos #f(-30 0 -20) :local)
    (ir2ri :wait-interpolation t :time 2000)

    (send *robot* :rarm :move-end-pos #f(0 0 200) :world
          :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    (send *ri* :stop-grasp :larm)

    ;; (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    ;; (ir2ri :time 2000 :wait-interpolation t)

    ;; (setq angle (gripper-angle :rarm))
    ;; (format t "gipper-angle ~A~%" angle)
    ;; (when (< angle 5)
    ;;   (speak-jp "落とした気がします")
    ;;   (return-from pick-hanko nil))

    ;; (send *robot* :larm :inverse-kinematics
    ;;       (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0 50 -30) :world)
    ;;       :rotation-axis :z)
    ;; (ir2ri :wait-interpolation t :time 2000)

    ;; (send *robot* :larm :inverse-kinematics
    ;;       (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0 0 -30) :world)
    ;;       :rotation-axis :z)
    ;; (ir2ri :wait-interpolation t :time 2000)

    ;; (send *robot* :larm :move-end-pos #f(0 0 20) :world)
    ;; (ir2ri :wait-interpolation t :time 2000)
    ;; (send *ri* :start-grasp :larm)
    ;; (unix::usleep (* 1000 1000))

    ;; (send *ri* :stop-grasp :rarm)
    ;; (send *robot* :rarm :move-end-pos #f(0 -50 50) :world)
    ;; (ir2ri :wait-interpolation t :time 2000)

    ;; (ri2ir)

    ;; (send *robot* :rarm :inverse-kinematics
    ;;       (send (send (send *robot* :larm :end-coords :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 150) :world)
    ;;       :rotation-axis :x
    ;;       :revert-if-fail nil)
    ;; (ir2ri :wait-interpolation t :time 2000)

    ;; (send *robot* :rarm :inverse-kinematics
    ;;       (send (send (send *robot* :larm :end-coords :copy-worldcoords) :rotate pi/2 :y :local) :translate #f(0 0 50) :world)
    ;;       :rotation-axis :x
    ;;       :revert-if-fail t)
    ;; (ir2ri :wait-interpolation t :time 2000)

    ;; (send *ri* :start-grasp :rarm)
    ;; (unix::usleep (* 1000 1000))
    ;; (send *ri* :stop-grasp :larm)
    ))


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

    ;; pre-grasp pose
    (send *robot* :rarm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

    ;; force grasp
    (send *ri* :move-gripper :rarm 0.0 :effort 100 :wait t)

    (setq target-coords
          (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y))
    (send *robot* :rarm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world))
    (ir2ri :wait-interpolation t)

    (send *robot* :rarm :inverse-kinematics target-coords)
    (ir2ri :wait-interpolation t)

    (send *ri* :start-grasp)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from push-shuniku nil))

    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)

    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 5)
      (speak-jp "落とした気がします")
      (return-from push-shuniku nil))
    ))




(defun open-shuniku-case ()
  (let (angle
        msg
        index
        pose
        (world-to-base-transform (send *robot* :copy-worldcoords))
        world-to-target-transform
        target-coords)
    (default-pose :wait-interpolation t)
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

    ;; pre-grasp pose
    (send *robot* :rarm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)
    (send *ri* :stop-grasp)

    (setq target-coords
          (send (send (make-coords :pos (send world-to-target-transform :worldpos)) :rotate pi/2 :y) :translate #f(0 20 0) :world))
    (send *robot* :rarm :inverse-kinematics
          (send (send target-coords :copy-worldcoords) :translate #f(0 0 200) :world))
    (ir2ri :wait-interpolation t)

    (send *robot* :rarm :inverse-kinematics target-coords)
    (ir2ri :wait-interpolation t)

    (send *ri* :start-grasp)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "掴むのに失敗しました")
      (send *ri* :stop-grasp :rarm)
      (return-from open-shuniku-case nil))

    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)

    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "落とした気がします")
      (return-from open-shuniku-case nil))

    (send *robot* :rarm :move-end-pos #f(0 200 0) :world)
    (ir2ri :time 2000 :wait-interpolation t)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (when (< angle 65)
      (speak-jp "落とした気がします")
      (return-from open-shuniku-case nil))
    (send *robot* :rarm :move-end-pos #f(0 0 -130) :world)
    (ir2ri :time 3000 :wait-interpolation t)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)
    (send *ri* :stop-grasp)
    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)
    (setq angle (gripper-angle :rarm))
    (format t "gipper-angle ~A~%" angle)))


(defun return-hanko-to-case ()
  (let (angle
        msg
        index
        pose
        world-to-base-transform
        world-to-target-transform
        target-coords
        target-name)
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

    ;; pre-grasp pose
    (send *robot* :rarm :move-end-pos #f(0 0 200) :world :revert-if-fail nil)
    (ir2ri :wait-interpolation t :time 2000)

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
    (ir2ri :wait-interpolation t)

    (send *robot* :rarm :inverse-kinematics target-coords
          :rotation-axis :x)
    (ir2ri :wait-interpolation t)

    (send *ri* :stop-grasp :rarm)

    (send *robot* :rarm :move-end-pos #f(0 0 100) :world)
    (ir2ri :time 2000 :wait-interpolation t)
    ))


(defun gripper-angle (&optional (arm :arms))
  (case arm
        (:rarm (* 1000.0 (send (send (*ri* . r-gripper-action) :get-result) :position)))
        (:larm (* 1000.0 (send (send (*ri* . l-gripper-action) :get-result) :position)))
        (:arms (list
                (* 1000.0 (send (send (*ri* . r-gripper-action) :get-result) :position))
                (* 1000.0 (send (send (*ri* . l-gripper-action) :get-result) :position))))))



(setup)
