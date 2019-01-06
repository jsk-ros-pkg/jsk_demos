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

(defun untuck-bowl-larm ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :angle-vector
        (float-vector 26.2517 61.506 101.673 -121.542 -91.2933 -93.1499 -72.758))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :larm :angle-vector
        (float-vector 45.6178 20.9171 122.559 -118.358 -122.343 -105.088 -109.796))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;
  )

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
  ;;
  ;; check if need to tuck arm
  (let ((big-objs '(bowl serial))
        larm rarm arms)
    (dolist (st (remove-if-not #'(lambda (x) (eq (car x) 'onhand))
                               (cdr (assoc :current-state userdata))))
      (case (caddr st)
        ('larm (setq larm (cadr st)))
        ('rarm (setq rarm (cadr st)))
        ('arms (setq rarm (cadr st)))))
    (when (and (null larm) (cdr (assoc :larm userdata)))
      (setq larm (cdr (assoc :larm userdata))))
    (when (and (null rarm) (cdr (assoc :rarm userdata)))
      (setq rarm (cdr (assoc :rarm userdata))))
    (when (and (null arms) (cdr (assoc :arms userdata)))
      (setq arms (cdr (assoc :arms userdata))))
    ;;
    (ros::ros-info "onhand: larm=~A rarm=~A arms=~A" larm rarm arms)
    (cond
      (arms t)
      ;; ((and larm rarm) t)
      ((and larm (memq larm big-objs))
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (untuck-bowl-larm)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (send *pr2* :larm :angle-vector (get-side-pose :larm))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
       (send *ri* :wait-interpolation)
       (pr2-tuckarm-pose-for-bowl :larm :outside))
      ((and rarm (memq rarm big-objs))
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (send *pr2* :rarm :angle-vector (get-side-pose :rarm))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
       (send *ri* :wait-interpolation)
       (pr2-tuckarm-pose-for-bowl :rarm :outside))
      (rarm
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose :rarm :outside))
      (larm
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose :larm :outside))
      (t
       (send *ri* :go-pos-unsafe -0.1 0 0)
       (pr2-tuckarm-pose)))
    ;;
    ;; (send *ri* :clear-costmap)
    (case to-place
      ('kitchen (go-to-sink userdata)) ;; coe
      ('dishwasher
       (when (eq from-place 'fridge)
         ;; rotate to avoid collision to table
         (send *ri* :go-pos-unsafe 0.1 -0.1 -90))
       (go-to-dishwasher userdata)) ;; coe
      (t
       (let (target-spot)
         (if (cdr (assoc :target-place userdata))
             (setq target-spot (get-spot (cdr (assoc :target-place userdata))))
             (setq target-spot (get-spot to-place)))
         (with-monitoring (:sound)
           (unwind-protect
                (progn
                  (when (or larm rarm arms)
                    (use-tilt-laser-obstacle-cloud nil))
                  (send *ri* :move-to target-spot))
             (when (or larm rarm arms)
               (use-tilt-laser-obstacle-cloud t)))))))))

(defaction find-object (obj spot &optional userdata)
  (ros::ros-info "(find-object ~A ~A)" obj spot)
  ;; just look at dishwasher
  (cond
    ((eq spot 'dishwasher)
     (let ((dishwasher-knob (cdr (assoc :dishwasher-knob userdata))))
       (cond
         (dishwasher-knob
          (send *pr2* :head :look-at
                (send (send (send *pr2* :copy-worldcoords)
                            :transformation (send dishwasher-knob :copy-worldcoords))
                      :worldpos)))
         (t
          (send *pr2* :head :neck-p :joint-angle 60)
          (send *pr2* :head :neck-y :joint-angle -60)))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
       (send *ri* :wait-interpolation)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))))
    (t
     ;; lookup label
     (let ((*tabletop-bbox-topic* "/tabletop/euclidean_clustering_decomposer/boxes")
           (*preferable-object-pose* (make-coords :pos (float-vector 700 0 750)))
           (object-label (get-object-label obj)))
       (when object-label
         (setq *tabletop-bbox-topic* "/ssd/bounding_box_pose/output/boxes")
         (set-alist :target-object-label object-label userdata))
       (unless object-label
         ;; soft grasp for unlabeled objects
         (set-alist :target-effort 0.007 userdata))
       (case spot
         ('fridge
          (grasp-can-init)
          (setq *tabletop-bbox-topic* "/detect_cans/bounding_box_pose/output/boxes")
          (setq *preferable-object-pose* (make-coords :pos (float-vector 700 150 950))))
         (t t))
       (find-tabletop-object userdata)))))

(defaction pick (obj arm spot &optional userdata)
  (ros::ros-info "(pick ~A ~A ~A)" obj arm spot)
  (with-monitoring (:sound)
    (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
    (let ((arm-controller (read-from-string
                           (format nil "~A-controller" (cdr (assoc :arm userdata)))))
          res)
      ;; pre pick pose for avoiding collision
      (unless (or (eq spot 'fridge)
                  (eq spot 'dishwasher))
        (send *pr2* (cdr (assoc :arm userdata)) :angle-vector
              (get-side-pose (cdr (assoc :arm userdata))))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 arm-controller)
        (send *ri* :angle-vector (pr2-pick-tray-pose) 2000 :torso-controller)
        (send *ri* :wait-interpolation)
        (send *ri* :angle-vector (pr2-pick-tray-pose) 2000 arm-controller)
        (send *ri* :wait-interpolation))
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      ;;
      (cond
        ((eq spot 'dishwasher)
         (setq res (pick-from-dishwasher userdata)))
        (t
         (with-monitoring (:joint)
           (cond
             ((eq spot 'fridge)
              (setq res (pick-in-fridge userdata)))
             ((eq obj 'bowl)
              (setq res (pick-bowl userdata)))
             (t
              (setq res (pick-tabletop-object userdata)))))))
      ;;
      (case (cdr (assoc :arm userdata))
        (:larm (set-alist :larm obj userdata))
        (:rarm (set-alist :rarm obj userdata))
        (:arms (set-alist :arms obj userdata)))
      res)))

(defaction find-placement (obj arm spot &optional userdata)
  (ros::ros-info "(find-placement ~A ~A ~A)" obj arm spot)
  ;; compute offset
  (let (offset)
    (case spot
      ('table
       (case obj
         ((bottle cup)
          (case arm
            ('larm (setq offset (float-vector -100 200 0)))
            ('rarm (setq offset (float-vector -100 -200 0)))))
         (t t)))
      (t t))
    (when offset
      (ros::ros-info "set place offset: ~A" offset)
      (set-alist :place-offset offset userdata)))
  t)

(defaction place (obj arm spot &optional userdata)
  (ros::ros-info "(place ~A ~A ~A)" obj arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (with-monitoring (:joint :sound)
    (cond
      ((eq spot 'dishwasher) ;; coe
       (put-into-dishwasher userdata))
      ((eq obj 'bowl)
       (place-bowl userdata))
      ((eq spot 'fridge)
       (place-in-fridge userdata))
      (t ;; place to support plane
       (place-object userdata))))
  (case (cdr (assoc :arm userdata))
    (:larm (set-alist :larm nil userdata))
    (:rarm (set-alist :rarm nil userdata))
    (:arms (set-alist :arms nil userdata)))
)

(defaction open-door (arm spot &optional userdata)
  (ros::ros-info "(open-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (with-monitoring (:sound)
    (cond
      ((eq spot 'dishwasher) ;; coe
       (open-dishwasher userdata))
      ((eq spot 'fridge)
       (send *pr2* :reset-coords)
       ;; look at door
       (base-go-to-fridge-init-pose :torso-lift 130 :head-pitch 0 :wait t)
       ;; open door
       (open-fridge-door :use-arm (if (eq :rarm (cdr (assoc :arm userdata))) :larm :rarm))
       ;; reach to fridge
       (send *ri* :go-pos-unsafe 0 0 -15)
       (send *ri* :go-pos-unsafe 0.30 -0.05 0)
       t)
      (t
       (open-push-door userdata)))))

(defaction close-door (arm spot &optional userdata)
  (ros::ros-info "(close-door ~A ~A)" arm spot)
  (set-alist :arm (intern (string-upcase arm) *keyword-package*) userdata)
  (with-monitoring (:sound)
    (cond
      ((eq spot 'dishwasher)
       (close-dishwasher userdata))
      ((eq spot 'fridge)
       ;; (send *ri* :go-pos-unsafe -0.22 -0.2 17)
       (send *ri* :go-pos-unsafe -0.18 -0.22 17)
       (send *ri* :go-pos-unsafe 0.05 -0.05 0)
       (send *pr2* :larm :angle-vector (get-side-pose :larm))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 :larm-controller)
       (send *ri* :wait-interpolation)
       (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
       (close-fridge :use-arm (cdr (assoc :arm userdata))
                     :tuck-arm nil
                     :move-only-use-arm t)
       t)
      (t t))))

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
