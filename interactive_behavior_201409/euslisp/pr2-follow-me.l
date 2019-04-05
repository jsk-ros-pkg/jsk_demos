#!/usr/bin/env roseus
;; pr2-follow-me.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "pr2_follow_me")

(ros::load-ros-manifest "jsk_recognition_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defparameter *cmd-vel-topic* "/base_controller/command_unchecked")
(defparameter *people-pose-topic* "/interaction/people_pose_estimation/pose")
(defparameter *base-frame-id* "base_footprint")
(defvar *tfl* (instance ros::transform-listener :init))
(defvar *people-pose* nil)

(defun pr2-follow-pose (&optional (arm :rarm))
  (let* ((av
         (case arm
           (:rarm (float-vector -61.8813 50.6757 -24.162 -106.014 22.1616 -39.9751 196.862))
           (:larm (float-vector  61.8813 50.6757  24.162 -106.014 22.1616 -39.9751 196.862))))
         (tuck-arm (if (eq arm :rarm) :larm :rarm))
         (tuck-av
          (case tuck-arm
            (:rarm (float-vector  -10.0 70.0  -100.0 -90.0 70.0 -10.0 20.0))
            (:larm (float-vector  10.0  70.0   100.0 -90.0 70.0 -10.0 20.0)))))

    (pr2-reset-pose)
    (send *ri* :start-grasp :arms)
    (send *ri* :wait-interpolation)
    ;;
    (send *pr2* arm :angle-vector av)
    (send *pr2* tuck-arm :angle-vector tuck-av)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)))

(defun send-cmd-vel (x y &optional r)
  (let ((msg (instance geometry_msgs::Twist :init)))
    (send msg :linear :x (* 0.003 x))
    (send msg :linear :y (* 0.003 y))
    (ros::publish *cmd-vel-topic* msg)))

(defun people-pose-cb (msg)
  (let ((person (car (send msg :poses)))
        (limb-names (list "Neck" "Nose" "RShoulder" "LShoulder"))
        result ps)
    (dotimes (i (length (send person :limb_names)))
      (when (> (elt (send person :scores) i) 0.5)
        (push (list (elt (send person :limb_names) i)
                    (elt (send person :poses) i)
                    (elt (send person :scores) i)) result)))
    (sort result #'>= #'caddr)
    (dolist (r result)
      (when (find (car r) limb-names :test #'string=)
        (setq ps (instance geometry_msgs::PoseStamped :init))
        (send ps :header (send msg :header))
        (send ps :pose (cadr r))
        (setq *people-pose* (send *tfl* :transform-pose *base-frame-id* ps))
        (return-from people-pose-cb t)))))

(defun main (&optional (arm :rarm))
  (let ((current-state :init)
        zero-cds cur-cds diff-pos)
    (ros::rate 10)
    (while (ros::ok)
      (case current-state
        (:init
         (ros::advertise *cmd-vel-topic* geometry_msgs::Twist 1)
         (ros::subscribe *people-pose-topic* jsk_recognition_msgs::PeoplePoseArray #'people-pose-cb)
         (pr2-init)
         (pr2-follow-pose arm)
         (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
         (setq zero-cds (send *pr2* arm :end-coords :copy-worldcoords))
         (send zero-cds :rpy 0 0 0)
         (setq current-state :follow))
        (:follow
         (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
         (setq cur-cds (send *pr2* arm :end-coords :copy-worldcoords))
         (send cur-cds :rpy 0 0 0)
         (setq diff-pos (send zero-cds :difference-position cur-cds))
         (when *people-pose*
           (send *pr2* :head :look-at (send *people-pose* :worldpos))
           (send *ri* :head-angle-vector (send *pr2* :angle-vector) 500))
         ;; (ros::ros-info "~A" diff-pos)
         (when (> (norm (subseq diff-pos 0 2)) 10)
           (send-cmd-vel (elt diff-pos 0) (elt diff-pos 1)))
         )
        (t (error "invalid state")))
      (setq *people-pose* nil)
      (ros::spin-once)
      (ros::sleep))
))

(main)
(exit)
