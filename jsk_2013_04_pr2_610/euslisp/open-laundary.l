(load "package://pr2eus/pr2-interface.l")
(setq *ri* (instance pr2-interface :init))
(pr2)
(objects (list *pr2*))
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(send *irtviewer* :draw-objects)
(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :start-grasp :rarm)
(speak-en "initialize")

(setq banzai-potentio-vector #f(50.1549 4.8215 -12.3808 61.5356 -8.60842 158.704 -19.4601 -42.9146 -50.9129 13.8802 -60.8123 -101.186 -137.428 -75.7661 117.379 2.60076 -21.2682))

(defun open-laundary-door ()
  (warn "open-laundary-door(kurowa)~%")

  (setq bottun (make-cascoords :pos #f(628.202 -413.612 846.468) :rpy #f(-0.043 0.724 -0.527)))
  (send *pr2* :head :look-at (send bottun :pos))

  (setq tmp (make-cascoords :pos #f(-26876.13 19710.332 19993.417) :rpy #f(1.765 -0.003 0.004)))
  ;;(clear-costmap)
  (setq inFrontOfLaundry (send tmp :worldcoords))
  ;;(send *ri* :move-to inFrontOfLaundry)

  ;; ready to push
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(489.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (speak-en "ready to push")

  ;; push
  (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(641.627 403.386 915.397) :rpy #f(-0.89 0.899 -0.942)))
  (send *pr2* :rarm :inverse-kinematics bottun) 
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (speak-en "finish pushing")

  ;; ready to open
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(606.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(606.869 -414.927 884.227) :rpy #f(0.167 0.434 1.487)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (speak-en "ready to open")

  ;; open
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(407.257 11.884 811.732) :rpy #f(1.78 1.044 0.771)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(407.257 11.884 811.732) :rpy #f(1.78 1.044 0.771)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (speak-en "finish openning")

  ;; move rarm in front of the center of the laundry 
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; ready to put
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18)))
  (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(273.03 193.055 795.998) :rpy #f(-2.492 0.752 0.61)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :wrist-r :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :rarm :wrist-p :joint-angle -50)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :wrist-p :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (speak-en "finish putting")

  ;; (let ((i 0) (x 360))
  ;;   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;   (send *pr2* :rarm :wrist-p :joint-angle -50)
  ;;   (while (< i x)
  ;;     (send *pr2* :rarm :wrist-r :joint-angle i)
  ;;     (send *irtviewer* :draw-objects)
  ;;     (send *ri* :angle-vector (send *pr2* :angle-vector))
  ;;     ;;(send *ri* :wait-interpolation)
  ;;     (print i)
  ;;     ;;(unix:usleep 1000000)
  ;;     (setq i (1+ i))))

  ;; undo
  ;; (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(891.571 -175.366 615.85) :rpy #f(0.124 0.003 1.18))) 
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector))
  ;; (send *ri* :wait-interpolation)

  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(535.506 -185.384 768.899) :rpy #f(0.381 -0.701 0.433)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(435.985 -561.193 849.837) :rpy #f(-0.28 -0.837 0.231)))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (speak-en "ready to close")

  ;; ;; pull
  ;; (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(273.277 193.941 795.83) :rpy #f(-2.496 0.754 0.606)))
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector))
  ;; (send *ri* :wait-interpolation)
  ;; (speak-en "pulling complete")

  ;; close
  ;; close-step1
  (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(261.821 -119.162 755.257) :rpy #f(-2.297 1.259 1.554)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(261.821 -119.162 755.257) :rpy #f(-2.297 1.259 1.554)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; close-step2
  (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(574.386 -304.799 730.777) :rpy #f(-0.938 1.148 2.28)))
  (send *pr2* :head :look-at (send (make-cascoords :pos #f(574.386 -304.799 730.777) :rpy #f(-0.938 1.148 2.28)) :pos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation)

  #|
  ;; close-step3
  (send *pr2* :larm :inverse-kinematics (make-cascoords :pos #f(422.049 230.184 766.429) :rpy #f(-1.199 1.348 1.869)))
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(491.883 -388.081 752.017) :rpy #f(0.202 0.284 2.908))) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
  (send *ri* :wait-interpolation)

  ;; close-step4
  (send *pr2* :rarm :inverse-kinematics (make-cascoords :pos #f(601.883 -388.081 752.017) :rpy #f(0.202 0.284 2.908))) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
  (send *ri* :wait-interpolation)
  |#

  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5)
  (speak-jp "ミッションコンプリート")
  )