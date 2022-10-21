#!/usr/bin/env roseus

(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :base64 "lib/llib/base64.l")

(require :main "package://detect_cans_in_fridge_201202/euslisp/main.l")


(defmacro def-state-func (name args &rest body)
  `(defun ,name ,args
     (set-alist 'description "" ,(car args))
     (set-alist 'image "" ,(car args))
     ,@body))

(defun image-cb (msg) (setq *image* msg))


(def-state-func init-func (userdata)
  (let ((can-type (cdr (assoc 'can-type userdata))))
    (setq *image* nil)
    (ros::subscribe "/edgetpu_object_detector/output/image/compressed"
                    sensor_msgs::CompressedImage #'image-cb)
    (initialize-demo can-type)
    ;; wait for subscribing the image topic
    (ros::spin-once)
    (set-alist 'description (format nil "冷蔵庫に~Aを取りにいくよ" can-type) userdata)
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    t))


(def-state-func move-to-fridge-func (userdata)
  (let ((res (goto-front-of-fridge)))
    (ros::spin-once)
    (if res
      (set-alist 'description "冷蔵庫の前に移動したよ" userdata)
      (set-alist 'description "冷蔵庫へ行けなかったよ" userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func open-fridge-func (userdata)
  (let* ((use-arm (cdr (assoc 'use-arm userdata)))
         (use-base (cdr (assoc 'use-base userdata)))
         (res (open-fridge-door :use-arm use-arm :use-base use-base)))
    (ros::spin-once)
    (if res
      (set-alist 'description "冷蔵庫の扉を開けたよ" userdata)
      (set-alist 'description "冷蔵庫の扉を開けられなかったよ" userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func grasp-can-func (userdata)
  (let* ((can-type (cdr (assoc 'can-type userdata)))
         (use-arm (cdr (assoc 'use-arm userdata)))
         (res (grasp-can :use-arm use-arm)))
    (ros::spin-once)
    (if res
      (set-alist 'description (format nil "~Aを取り出せたよ" can-type) userdata)
      (set-alist 'description (format nil "~Aを取り出せなかったよ" can-type) userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func close-fridge-func (userdata)
  (let* ((use-arm (cdr (assoc 'use-arm userdata)))
         (res (close-fridge :use-arm use-arm)))
    (ros::spin-once)
    (if res
      (set-alist 'description "冷蔵庫の扉を閉めたよ" userdata)
      (set-alist 'description "冷蔵庫の扉を閉められなかったよ" userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func move-to-initial-position-func (userdata)
  (let* ((can-type (cdr (assoc 'can-type userdata)))
         (use-arm (cdr (assoc 'use-arm userdata)))
         (res (goto-initial-position :use-arm use-arm)))
    (ros::spin-once)
    (if res
      (set-alist 'description (format nil "~Aを持っていったよ" can-type) userdata)
      (set-alist 'description (format nil "~Aを持っていけなかったよ" can-type) userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func hand-over-can-func (userdata)
  (let* ((can-type (cdr (assoc 'can-type userdata)))
         (use-arm (cdr (assoc 'use-arm userdata)))
         (res (pass-to-human :use-arm use-arm)))
    ;; we dont need this here for better photo 
    ;; (ros::spin-once)
    (if res
      (set-alist 'description (format nil "~Aを渡したよ" can-type) userdata)
      (set-alist 'description (format nil "~Aを渡せなかったよ" can-type) userdata))
    (if *image* (set-alist 'image (remove #\newline (base64encode (send *image* :serialize))) userdata))
    res))


(def-state-func finish-func (userdata) t)


(def-state-func finish-failure-func (userdata) t)


(defun fridge-demo-state-machine ()
  (setq *sm*
        (make-state-machine
          '((:init -> :move-to-fridge)
            (:move-to-fridge -> :open-fridge)
            (:move-to-fridge !-> :finish-failure)
            (:open-fridge -> :grasp-can)
            (:open-fridge !-> :finish-failure)
            (:grasp-can -> :close-fridge)
            (:grasp-can !-> :finish-failure)
            (:close-fridge -> :move-to-initial-position)
            (:close-fridge !-> :finish-failure)
            (:move-to-initial-position -> :hand-over-can)
            (:move-to-initial-position !-> :finish-failure)
            (:hand-over-can -> :finish)
            (:hand-over-can !-> :finish-failure)
            (:finish -> t)
            (:finish-failure -> nil))
          '((:init 'init-func)
            (:move-to-fridge 'move-to-fridge-func)
            (:open-fridge 'open-fridge-func)
            (:grasp-can 'grasp-can-func)
            (:close-fridge 'close-fridge-func)
            (:move-to-initial-position 'move-to-initial-position-func)
            (:hand-over-can 'hand-over-can-func)
            (:finish 'finish-func)
            (:finish-failure 'finish-failure-func))
          '(:init)
          '(t nil))))


(defun fridge-demo-smach-main (&key (can-type "georgia")
                                    (use-arm :larm)
                                    (use-base nil))
  (when (not (boundp '*sm*))
    (fridge-demo-state-machine))
  (let ((result-state
          (exec-state-machine
            *sm* `((description . ,(format nil "冷蔵庫に~Aを取りに行ったよ" can-type))
                   (image . "")
                   (can-type . ,can-type)
                   (use-arm . ,use-arm)
                   (use-base . ,use-base))
            :hz 2.0)))
    (send result-state :name)))


(provide :fridge-demo-smach-main)
