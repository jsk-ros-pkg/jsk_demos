#!/usr/bin/env roseus
;; pr2-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :pr2-actionlib)

(ros::load-ros-manifest "speech_recognition_msgs")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-preemptive-interface "package://interactive_behavior_201409/euslisp/pr2-preemptive-interface.l")
;;
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :coe-actionlib "package://jsk_2017_12_pr2_coe/euslisp/coe-actionlib.l")
(require :door-utils "package://interactive_behavior_201409/euslisp/door-utils.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Action implementations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *speech-msg* nil)

(defun speech-callback (msg)
  (setq *speech-msg* (car (send msg :transcript))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Action definitions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defaction move-to (from-place to-place &optional userdata)
  (ros::ros-info "(move-to ~A ~A)" from-place to-place)
  (save-start-pose)
  (case to-place
    ('kitchen (go-to-sink userdata)) ;; coe
    ('dishwasher (go-to-dishwasher userdata)) ;; coe
    (t
     (let (target-spot)
       (if (cdr (assoc :target-place userdata))
           (setq target-spot (get-spot (cdr (assoc :target-place userdata))))
         (setq target-spot (get-spot to-place)))
       (use-tilt-laser-obstacle-cloud nil)
       (prog1 (send *ri* :move-to target-spot)
         (use-tilt-laser-obstacle-cloud t))))))


(defaction pick (obj arm spot &optional userdata)
  (ros::ros-info "(pick ~A ~A ~A)" obj arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
    (t
     (unless (find-tabletop-object userdata)
       (return-from pick nil))
     (pick-tabletop-object userdata))))

(defaction place (obj arm spot &optional userdata)
  (ros::ros-info "(place ~A ~A ~A)" obj arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (put-into-dishwasher userdata))
   (t ;; place to support plane
    (place-object userdata))))

(defaction open-door (arm spot &optional userdata)
  (ros::ros-info "(open-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (open-dishwasher userdata))
   (t
    (open-push-door userdata))))

(defaction close-door (arm spot &optional userdata)
  (ros::ros-info "(close-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher)
    (close-dishwasher userdata))
   (t t)))

(defaction identify-object (obj arm spot &optional userdata)
  ;; TODO: ask the in-hand object to person
  t)

(defaction deliver (obj arm person spot &optional userdata)
  (setq arm (intern (string-upcase arm) *keyword-package*))
  (speak-jp (format nil "~Aを どうぞ" (string-upcase obj)))
  (hand-over arm :wait-shock t)
  t)


(defaction ask (person spot &optional userdata)
  (let ((content (cdr (assoc :content userdata))))
    (speak-jp
     (format nil "~Aさん。聞きたいことがあります。次の質問に答えてください。。~A" person content))
    (ros::subscribe "/speech_to_text"
                    speech_recognition_msgs::SpeechRecognitionCandidates
                    #'speech-callback)
    (ros::rate 2)
    (setq *speech-msg* nil)
    (while (and (ros::ok) (null *speech-msg*))
      (ros::ros-info "waiting for answer")
      (ros::spin-once)
      (ros::sleep))
    (speak-jp "わかりました。" :wait t)
    ;;
  t))


#|
(defaction pick (target arm spot &optional userdata)
  (let (target-cds pregrasp grasp)
    ;; FIXME
    (setq target-cds (make-coords :pos (float-vector 800 0 600)))
    ;;
    (setq arm (intern (symbol-string arm) *keyword-package*))
    (setq grasp
     (send *robot* arm :inverse-kinematics target-cds
                       :rotation-axis :z
                       :look-at-target t))
    (setq pregrasp
     (send *robot* arm :move-end-pos
           (float-vector -100 0 0)))
    (unless (and grasp pregrasp)
      (ros::ros-error "ik fail")
      (return-from pick nil))
    (send *ri* :stop-grasp arm :wait t)
    ;;
    (send *ri* :angle-vector-sequence
          (list pregrasp grasp)
          (list 10000 3000))
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :start-grasp arm)
    ;;
    (send *ri* :angle-vector pregrasp 3000)
    (send *ri* :wait-interpolation)))

(defaction place (target arm spot &optional userdata)
  (let (target-cds preungrasp ungrasp)
    ;; FIXME
    (setq target-cds (make-coords :pos (float-vector 800 0 600)))
    ;;
    (setq arm (intern (symbol-string arm) *keyword-package*))
    (setq ungrasp
          (send *robot* arm :inverse-kinematics target-cds
                            :rotation-axis :z
                            :look-at-target t))
    (setq preungrasp
          (send *robot* arm :move-end-pos
                (float-vector 0 0 100)))
    (unless (and preungrasp ungrasp)
      (ros::ros-error "ik fail")
      (return-from place nil))
    ;;
    (send *ri* :angle-vector-sequence
          (list preungrasp ungrasp)
          (list 10000 3000))
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :stop-grasp arm :wait t)
    ;;
    (send *ri* :angle-vector preungrasp 3000)
    (send *ri* :wait-interpolation)))

(defaction move-to (from-spot to-spot &optional userdata)
  (let (cds)
    ;; FIXME
    (let ((start-time (ros::time-now)))
      (while (and (ros::ok)
                  (< (send (ros::time- (ros::time-now) start-time) :to-sec) 10.0))
             (send *ri* :ros-wait 1.0 :spin t :spin-self t)))
    (return-from move-to t)
    (setq cds
          (case to-spot
            ('START (make-coords :pos (float-vector 0 0 0)))
            ('TABLE (make-coords :pos (float-vector 0 5000 0)))
            ('KITCHEN (make-coords :pos (float-vector 10000 0 0)))
            ('COUNTER (make-coords :pos (float-vector 5000 0 0)))
            (t (error "unknown spot" to-spot))))
    (send *ri* :move-to cds)))
|#
