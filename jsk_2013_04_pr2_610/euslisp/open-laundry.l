;; 1.irteusgl$ (send *ri* :state :worldcoords)
;; (:worldcoords "/world")
;; #<cascaded-coords #X94e3948 /world  -26787.743 17924.335 19999.995 / 1.645 0.002 -0.001>
;; 2.irteusgl$ (send *ri* :state :worldcoords "/map")
;; (:worldcoords "/map")
;; #<cascaded-coords #X9541540 /map  -26787.743 17924.335 -0.005 / 1.645 0.003 -0.002>


(load "models/room610-scene.l")
(load "package://pr2eus/pr2-interface.l")

(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))

(objects (list *pr2*))
(send *irtviewer* :draw-objects)
(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :start-grasp :rarm)

(setq *laundry* (room610-laundry-machine))
(send *laundry* :move-to (make-cascoords :pos #f(1240 -180 -85) :rpy (float-vector pi 0 0)) :world)

(setq release (send *laundry* :handle-contents))
(setq handle (send *laundry* :handle-handle-cap))
(setq button (send *laundry* :handle-open-button))

(let ((r 500) (depth 500) (near 80) (cap_point (make-cascoords)))

  ;; STEP1 Push the Button
  (let ((dst (make-coords)) (front 120))
    ;; modify the model's parameters
    (send dst :init :pos (send (send (send button :copy-worldcoords) :translate (float-vector (- near) 0 0) :world) :worldpos) :rot (send *pr2* :worldrot))

    ;; ready to push
    (send *pr2* :rarm :inverse-kinematics dst :look-at-target t)
    (objects (list *pr2* *laundry*))
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; push
    (send *pr2* :rarm :move-end-pos (float-vector near 0 0))
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    
    ;; pull the right arm for opening the cap
    (send *pr2* :rarm :move-end-pos (float-vector (- front) 0 0))
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; store current position of right arm
    (setq former (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    )
  (speak-en "STEP1 complete")

  ;; STEP2 Open the Cap
  (let ((dst (make-coords)))
    ;; modify the model's parameters
    (send cap_point :init :pos (send (send (send handle :copy-worldcoords) :translate #f(-60 0 0) :world) :worldpos) :rot (send handle :worldrot))

    ;; insert the right arm into the gap
    (send *pr2* :rarm :inverse-kinematics cap_point :look-at-target t)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    
    ;; open the cap
    (send dst :init :pos (send (send (send cap_point :copy-worldcoords) :translate (float-vector (- near) (* 0.7 r) 0) :world) :worldpos) :rot (send cap_point :worldrot))
    (objects (list *pr2* *laundry*))
    (send *pr2* :rarm :inverse-kinematics dst :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; move the right arm for putting the cloth
    (send *pr2* :rarm :move-end-pos (float-vector (- (* r 0.3)) 0 0))
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  (speak-en "STEP2 complete")

  ;; STEP3 Put the Cloth into the Laundry Machine
  (let ((dst (make-coords)))
    ;; modify the model's parameters
    (send dst :init :pos (send (send (send release :copy-worldcoords) :translate #f(130 -50 90) :world) :worldpos) :rot (send *pr2* :worldrot))

    ;; ready to put
    (send *pr2* :rarm :inverse-kinematics (send (send dst :copy-worldcoords) :translate (float-vector (- (* depth 0.4)) 0 near) :world) :look-at-target t)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; put the cloth
    (send *pr2* :rarm :inverse-kinematics dst :rotation-axis nil :look-at-target t)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :rarm)
    (send *pr2* :rarm :wrist-r :joint-angle -180)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :rarm)

    ;; pull the right arm
    (send *pr2* :rarm :inverse-kinematics (send (send dst :copy-worldcoords) :translate (float-vector (- (* depth 0.7)) 0 (* near 1.2)) :world) :look-at-target t)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    )
  (speak-en "STEP3 complete")

  ;; STEP4 Close the Cap
  (let ()
    ;; move the right arm in order to avoid the cap
    (send *pr2* :rarm :inverse-kinematics (send (send former :copy-worldcoords) :translate #f(-100 -200 -100) :world) :rotational-axis nil)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; ready to close
    (send *pr2* :larm :move-end-pos (float-vector 0 (- (* r 0.6)) 0) :world)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; close the cap
    (send *pr2* :larm :inverse-kinematics (send (send cap_point :copy-worldcoords) :translate (float-vector (+ near 50) 0 0) :world) :rotation-axis nil)
    (objects (list *pr2* *laundry*))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  (speak-en "STEP4 complete")

  ;; initialize the pose
  (send *pr2* :reset-pose)
  (objects (list *pr2* *laundry*))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (speak-en "MISISON COMPLETE")
  )

;; (load "package://pr2eus/pr2-interface.l")
;; (setq *ri* (instance pr2-interface :init))
;; (pr2)
;; (objects (list *pr2*))
;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;; (send *irtviewer* :draw-objects)
;; (send *pr2* :reset-pose)
;; ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
;; ;;(send *ri* :start-grasp :rarm)
;; (speak-en "initialize")

;; (setq banzai-potentio-vector #f(50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

;; (load "models/room610-laundary-machine-object.l")
;; (setq *laundry* (room610-laundary-machine))
;; (send *laundry* :move-to (make-cascoords :pos #f(18000 0 0) :rpy (float-vector pi 0 0)) :world)

;; (send *laundry* :move-to (make-cascoords :pos #f(1250 100 0) :rpy (float-vector pi 0 0)) :world)
;; (setq bottun (make-cascoords :pos #f(628.202 -413.612 846.468) :rpy #f(-0.043 0.724 -0.527)))
;; (send *pr2* :rarm :inverse-kinematics bottun)
;; (objects (list *pr2* *laundry*))

;; (defun open-laundary-door ()
;;   (warn "open-laundary-door(kurowa)~%")

;;   (laod "models/room610-laundry-machine-object.l")
;;   (setq *laundry* (room610-laundary-machine))

;;   (setq bottun (make-cascoords :pos #f(628.202 -413.612 846.468) :rpy #f(-0.043 0.724 -0.527)))
;;   (send *pr2* :head :look-at (send bottun :pos))

;;   (setq tmp (make-cascoords :pos #f(-26876.13 19710.332 19993.417) :rpy #f(1.765 -0.003 0.004)))
;;   ;;(clear-costmap)
;;   (setq inFrontOfLaundry (send tmp :worldcoords))
;;   ;;(send *ri* :move-to inFrontOfLaundry)

;;   ;; ready to push
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(489.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;;   (send *ri* :wait-interpolation)
;;   (speak-en "ready to push")

;;   ;; push
;;   (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(641.627 403.386 915.397) :rpy #f(-0.89 0.899 -0.942)))
;;   (send *pr2* :rarm :inverse-kinematics bottun) 
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (speak-en "finish pushing")

;;   ;; ready to open
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(606.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(606.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;;   (send *ri* :wait-interpolation)
;;   (speak-en "ready to open")

;;   ;; open
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(407.257 11.884 811.732) :rpy #f(1.78 1.044 0.771)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(407.257 11.884 811.732) :rpy #f(1.78 1.044 0.771)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (speak-en "finish openning")

;;   ;; move rarm in front of the center of the laundry 
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)

;;   ;; ready to put
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18)))
;;   (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(273.03 193.055 795.998) :rpy #f(-2.492 0.752 0.61)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (send *pr2* :rarm :wrist-r :joint-angle 0)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (send *ri* :stop-grasp :rarm)
;;   (send *pr2* :rarm :wrist-p :joint-angle -50)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (send *pr2* :rarm :wrist-p :joint-angle 0)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (speak-en "finish putting")

;;   ;; (let ((i 0) (x 360))
;;   ;;   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;   ;;   (send *pr2* :rarm :wrist-p :joint-angle -50)
;;   ;;   (while (< i x)
;;   ;;     (send *pr2* :rarm :wrist-r :joint-angle i)
;;   ;;     (send *irtviewer* :draw-objects)
;;   ;;     (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   ;;     ;;(send *ri* :wait-interpolation)
;;   ;;     (print i)
;;   ;;     ;;(unix:usleep 1000000)
;;   ;;     (setq i (1+ i))))

;;   ;; undo
;;   ;; (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18))) 
;;   ;; (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   ;; (send *ri* :wait-interpolation)

;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)

;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(435.985 -561.193 849.837) :rpy #f(-0.28 -0.837 0.231)))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)
;;   (speak-en "ready to close")

;;   ;; ;; pull
;;   ;; (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(273.277 193.941 795.83) :rpy #f(-2.496 0.754 0.606)))
;;   ;; (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   ;; (send *ri* :wait-interpolation)
;;   ;; (speak-en "pulling complete")

;;   ;; close
;;   ;; close-step1
;;   (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(261.821 -119.162 755.257) :rpy #f(-2.297 1.259 1.554)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(261.821 -119.162 755.257) :rpy #f(-2.297 1.259 1.554)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector))
;;   (send *ri* :wait-interpolation)

;;   ;; close-step2
;;   (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(574.386 -304.799 730.777) :rpy #f(-0.938 1.148 2.28)))
;;   (send *pr2* :head :look-at (send (make-cascoords :pos #f(574.386 -304.799 730.777) :rpy #f(-0.938 1.148 2.28)) :pos))
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
;;   (send *ri* :start-grasp :rarm)
;;   (send *ri* :wait-interpolation)

;;   #|
;;   ;; close-step3
;;   (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(422.049 230.184 766.429) :rpy #f(-1.199 1.348 1.869)))
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(491.883 -388.081 752.017) :rpy #f(0.202 0.284 2.908))) 
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
;;   (send *ri* :wait-interpolation)

;;   ;; close-step4
;;   (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(601.883 -388.081 752.017) :rpy #f(0.202 0.284 2.908))) 
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
;;   (send *ri* :wait-interpolation)
;;   |#

;;   (send *pr2* :reset-pose)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
;;   (speak-jp "ミッションコンプリート")
;;   )
