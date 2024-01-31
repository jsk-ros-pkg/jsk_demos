#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :base64 "lib/llib/base64.l")

(ros::roseus-add-msgs "speech_recognition_msgs")
(ros::roseus-add-msgs "smach_msgs")
(setq *skip-flag* nil)

(defun skip-cb (msg)
  (if (string= (elt (send msg :transcript) 0) "スキップ")
      (setq *skip-flag* t))
  )

(defun start-func (args)
  (setq *skip-flag* t)
  (ros::ros-error"start-func")
  ;; (unix::sleep 3)
  :started)

(defun end-func (args)
  (ros::ros-error "end-func")
  ;; (unix::sleep 3)
  :finished)

(defun sample1-func (args)
  (ros::ros-error "sample1-func")
  ;; (unix::sleep 3)
  :finished)

(defun sample2-func (args)
  (ros::ros-error "sample2-func")
  ;; (unix::sleep 3)
  :finished)

(defun sample3-func (args)
  (ros::ros-error "sample3-func")
  ;; (unix::sleep 3)
  :finished)

(defun sample-skip-sm ()
  (setq sm
        (make-state-machine
         '((:start :started :sample1)

           (:sample1 :finished :sample2)
           (:sample1 :skip :sample3)
           (:sample1 -> :sample1)
           (:sample1 !-> :start)

           (:sample2 :finished :sample3)
           (:sample2 -> :sample2)
           (:sample2 !-> :start)
           ;;(:sample2 :skip :end)

           (:sample3 :finished :end)
           (:sample3 :skip :end)

           (:end :finished :goal))
         '((:start 'start-func)
           (:sample1 'sample1-func)
           (:sample2 'sample2-func)
           (:sample3 'sample3-func)
           (:end 'end-func))
         '(:start)
         '(:goal)
         ))
  (send sm :arg-keys 'description)
  sm)


(defun main (&key (tweet t))
  (when (not (boundp 'sm))
    (sample-skip-sm))
  (let ((result-state
         (exec-state-machine sm '(nil)
                             :hz 2.0
                             :before-hook-func
                             '(lambda (userdata)
                               (if *skip-flag*
                                   (progn
                                     (setq *skip-flag* nil)
                                     (send sm :force-next-active-state userdata)
                                     (setq msg (instance smach_msgs::SmachContainerStatus :init))
                                     (one-shot-publish "/server_name/smach/container_status"))))
                             )))
    (send result-state :name)))

(ros::roseus "sample-skip-state")
(ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'skip-cb)
