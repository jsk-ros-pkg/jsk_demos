(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(load "package://fetcheus/fetch-interface.l")
(load "package://welcome_to_jsk_fetch/euslisp/roseus-utils.l")
(load "package://welcome_to_jsk_fetch/euslisp/fetch-interface-utils.l")
(ros::load-ros-manifest "actionlib_msgs")
(ros::load-ros-manifest "speech_recognition_msgs")
(ros::load-ros-manifest "opencv_apps")


(defvar spot-names
  ;; list of (spot name, pin name, relative rot [rad])
  (list (list "じぇーえすけー" "/eng2/7f/room73A3-front" pi)
        (list "ろうか" "/eng2/7f/room73A3-front" pi)
        (list "えれべーた" "/eng2/7f/elevator-outside" 0)
        ))

(defun name-to-pin (spot-name)
  (dolist (sn spot-names)
    (when (equal spot-name (car sn))
      (return-from name-to-pin (cadr sn)))))

(defun name-to-relative-rot (spot-name)
  (dolist (sn spot-names)
    (when (equal spot-name (car sn))
      (return-from name-to-relative-rot (caddr sn)))))

(defun register-spots-to-julius ()
  (ros::wait-for-service "speech_recognition")
  (let ((req (instance speech_recognition_msgs::SpeechRecognitionRequest :init)))
    (send (send req :vocabulary) :words (mapcar #'car spot-names))
    (ros::service-call "speech_recognition" req nil)))


(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (undock-rotate nil) (clear-costmap t))
  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (if clear-costmap (send *ri* :clear-costmap))
  (let ((undock-success nil))
    (block go-to-spot-undock
      (dotimes (i 3)
        (let* ((battery-charging-state (get-battery-charging-state)))
          ;; You may fail to subscribe /battery_state
          ;; because of message md5 difference between melodic and indigo.
          (unless battery-charging-state
            (ros::ros-warn "Failed to subscribe /battery_state")
            (ros::ros-warn "Skip undocking, so please make sure that Fetch is already undocked.")
            (return-from go-to-spot-undock t))
          (if (equal battery-charging-state :charging)
            (progn
              (setq undock-success (auto-undock :n-trial 3 :rotate-in-place undock-rotate)))
            (return-from go-to-spot-undock t))
          (if (not undock-success)
            (progn
              (ros::ros-error "Failed to undock")
            (send *ri* :speak "I failed to undock.")))))
      (if (not undock-success) (return-from go-to-spot nil))))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *ri* :move-to goal-pose :frame-id frame-id :no-wait t)))


(defun go-to-spot-from-speech (&key (move-base-retry 100000))
  (let* ((speech (one-shot-subscribe
                  "speech_to_text_julius"
                  speech_recognition_msgs::SpeechRecognitionCandidates))
         (spot-name (car (send speech :transcript)))
         (pin-name (name-to-pin spot-name))
         (relative-rot (name-to-relative-rot spot-name))
         move-base-result
         (move-base-count 0))
    ;; Start navigation
    (send *ri* :speak-jp
          (format nil "~Aに移動します。動きますから注意してください。" spot-name))
    (store-params)
    (inflation-loose)
    (go-to-spot pin-name :relative-rot relative-rot)
    (while (and (or (not (= ret actionlib_msgs::GoalStatus::*succeeded*))
                    (not (= ret actionlib_msgs::GoalStatus::*aborted*)))
                (<= move-base-count move-base-retry))
      (setq move-base-result (send *ri* :move-to-wait-once :retry 0 :wait-for-result-duration-time 1)))
    (restore-params)))


(defun guide-pose ()
  (send *ri* :start-grasp)
  (send *fetch* :reset-pose)
  (send *fetch* :torso :waist-z :joint-angle 350)
  (send *fetch* :inverse-kinematics (make-coords :pos #f(500 -500 900) :rpy (float-vector -pi/2 0 0)) :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  (send *ri* :wait-interpolation))

(defun find-face ()
  (let (face-msg faces (find-face t))
    (dotimes (i 5)
      (setq face-msg
            (one-shot-subscribe "face_detection/faces" opencv_apps::FaceArrayStamped))
      (setq faces (send face-msg :faces))
      (setq find-face (and find-face faces))
      (unix:usleep (* 1000 100)))
    (if find-face faces nil)))


(defun find-people ()
  (one-shot-subscribe
   "/welcome_to_jsk/people_pose_aggregated_detector/output/detected"
   std_msgs::Bool
   :condition #'(lambda (msg) (send msg :data))))


(defun greeting (&key (use-people-detection nil))
  ;; Look up
  (send *fetch* :angle-vector
        (send *ri* :state :potentio-vector :wait-until-update t))
  (send *fetch* :head :neck-p :joint-angle -20)
  (send *fetch* :head :neck-y :joint-angle 0)
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)
  (if use-people-detection
      (progn
        (ros::ros-info "Searching for people...")
        (find-people))
      (let (faces face-x face-y)
        ;; Wait until face is found
        (do-until-key
            (format t "Searching for people ...~%")
          (if (setq faces (find-people))
              (return)
              (unix:sleep 1)))
        ;; Look at the found face
        (format t "find faces~%")
        (send *fetch* :angle-vector
              (send *ri* :state :potentio-vector :wait-until-update t))
        (setq face-x (send (send (car faces) :face) :x))
        (setq face-y (send (send (car faces) :face) :y))
        (send *fetch* :head :neck-y :joint-angle (- (send *fetch* :head :neck-y :joint-angle) (/ (max -40 (min 40 (- face-x 320))) 5.0)))
        (send *fetch* :head :neck-p :joint-angle (+ (send *fetch* :head :neck-p :joint-angle) (/ (max -20 (min 20 (- face-y 240))) 5.0)))
        (send *ri* :angle-vector (send *fetch* :angle-vector) 300)
        (send *ri* :wait-interpolation)))
  (send *ri* :speak-jp "JSKへようこそ。見学はこちらになります。" :wait t))

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
