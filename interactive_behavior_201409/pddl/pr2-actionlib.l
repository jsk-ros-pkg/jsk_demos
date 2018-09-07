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
  ;;
  (when (or (not (has-start-pose)) (eq 'from-place 'start))
    (save-start-pose))
  ;;
  (send *ri* :change-inflation-range 0.25)
  ;; check if need to tuck arm
  (let ((larm (cdr (assoc :larm userdata)))
        (rarm (cdr (assoc :rarm userdata)))
        (arms (cdr (assoc :arms userdata))))
    (ros::ros-info "onhand: larm=~A rarm=~A arms=~A" larm rarm arms)
    (cond
      (arms t)
      ;; ((and larm rarm) t)
      ((and larm (eq 'bowl larm))
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose-for-bowl :larm :outside))
      ((and rarm (eq 'bowl rarm))
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose-for-bowl :rarm :outside))
      (rarm
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose :rarm :outside))
      (larm
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose :larm :outside))
      (t
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose))))

  (send *ri* :clear-costmap)
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

(defaction find-object (obj spot &optional userdata)
  (ros::ros-info "(find-object ~A ~A)" obj spot)
  ;; lookup label
  (let ((*tabletop-bbox-topic* "/tabletop/euclidean_clustering_decomposer/boxes"))
    (case obj
      ('bowl
       (setq *tabletop-bbox-topic* "/ssd/bounding_box_pose/output/boxes")
       (set-alist :target-object-label 45 userdata))
      ('cup
       (setq *tabletop-bbox-topic* "/ssd/bounding_box_pose/output/boxes")
       (set-alist :target-object-label 41 userdata))
      ('bottle
       (setq *tabletop-bbox-topic* "/ssd/bounding_box_pose/output/boxes")
       (set-alist :target-object-label 39 userdata))
      )
    (find-tabletop-object userdata)))

(defaction pick (obj arm spot &optional userdata)
  (ros::ros-info "(pick ~A ~A ~A)" obj arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (let ((arm-controller (read-from-string (format nil "~A-controller" (cdr (assoc :arm userdata)))))
        res)
    (send *pr2* (cdr (assoc :arm userdata)) :angle-vector (get-side-pose (cdr (assoc :arm userdata))))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 arm-controller)
    (send *ri* :angle-vector (pr2-pick-tray-pose) 2000 :torso-controller)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (pr2-pick-tray-pose) 2000 arm-controller)
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq res (case obj
                ('bowl
                 (pick-bowl userdata))
                (t
                 (pick-tabletop-object userdata))))
    (case (cdr (assoc :arm userdata))
      (:larm (set-alist :larm obj userdata))
      (:rarm (set-alist :rarm obj userdata))
      (:arms (set-alist :arms obj userdata)))
  ;; tuckarm is moved to move-to action
    res))

(defaction find-placement (obj arm spot &optional userdata)
  (ros::ros-info "(find-placement ~A ~A ~A)" obj arm spot)
  t)

(defaction place (obj arm spot &optional userdata)
  (ros::ros-info "(place ~A ~A ~A)" obj arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (put-into-dishwasher userdata))
   ((eq obj 'bowl)
    (place-bowl userdata))
   ((eq spot 'fridge)
    (place-in-fridge userdata))
   (t ;; place to support plane
    (place-object userdata)))
  (case (cdr (assoc :arm userdata))
    (:larm (set-alist :larm nil userdata))
    (:rarm (set-alist :rarm nil userdata))
    (:arms (set-alist :arms nil userdata)))
)

(defaction open-door (arm spot &optional userdata)
  (ros::ros-info "(open-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (open-dishwasher userdata))
   ((eq spot 'fridge)
    ;; look at door
    (base-go-to-fridge-init-pose :torso-lift 130 :head-pitch 0 :wait t)
    ;; open door
    (open-fridge-door :use-arm (if (eq :rarm (cdr (assoc :arm userdata))) :larm :rarm))
    ;; reach to fridge
    (send *ri* :go-pos-unsafe 0 0 -15)
    (send *ri* :go-pos-unsafe 0.30 -0.05 0)
    t)
   (t
    (open-push-door userdata))))

(defaction close-door (arm spot &optional userdata)
  (ros::ros-info "(close-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (cond
   ((eq spot 'dishwasher)
    (close-dishwasher userdata))
   ((eq spot 'fridge)
    ;; (send *ri* :go-pos-unsafe -0.22 -0.2 17)
    (send *ri* :go-pos-unsafe -0.18 -0.22 17)
    (send *ri* :go-pos-unsafe 0.05 -0.05 0)
    (close-fridge :use-arm (cdr (assoc :arm userdata)))
    t)
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
