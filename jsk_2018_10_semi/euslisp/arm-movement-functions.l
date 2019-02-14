(ros::load-ros-manifest "jsk_recognition_msgs")
(setq *co* (instance collision-object-publisher :init))
(setq *baskets* (make-cube 300 800 200))

;;台の前にたどり着いた時に, はおどぉ俯瞰できるようなポーズ
(setq look-down-pose #f(386.15 75.6359 80.1929 -11.4118 98.5702 0.060285 95.1322 -0.060071 -0.895055 42.9474))

;;箱の上空に腕を持ってくるための関節角度を計算しておく
(setq basket-red-angle (send *fetch* :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(200 300 700) :rpy (float-vector (/ pi 2) 0.0 0.0)))))
(setq basket-green-angle (send *fetch* :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(200 0 700) :rpy (float-vector (/ pi 2) 0.0 0.0)))))
(setq basket-blue-angle (send *fetch* :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(200 -300 700) :rpy (float-vector (/ pi -2) (/ pi 2) 0.0)))))


(defun goto-kitchenside (&optional (move-real t))
  (if move-real
      (send *ri* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
				:transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))))
    )
  (send *fetch* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
			       :transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))) :world)
  (if move-real
      (send *ri* :angle-vector look-down-pose 5000)
    )
  (send *fetch* :angle-vector look-down-pose)
  (send *irtviewer* :look-all *fetch*)
  )

(defun store-box-to-basket (address  &optional (set-collision-object t))
  (if set-collision-object
      (progn
	(make-collision-cube)
	(send *co* :add-object *baskets*)
	)
    )
  (progn
    ;;描画
    (cond
     ((string= address "red") (send *ri* :angle-vector basket-red-angle 5000)  (send *fetch* :angle-vector basket-red-angle ))
     ((string= address "green") (send *ri* :angle-vector basket-green-angle 5000)  (send *fetch* :angle-vector basket-green-angle ))
     ((string= address "blue") (send *ri* :angle-vector basket-blue-angle 5000)  (send *fetch* :angle-vector basket-blue-angle ))
     )
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    ;;(send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
    ;;(send *ri* :wait-interpolation)
    (if set-collision-object (send *co* :wipe-all))
    )
  )


(defun make-collision-cube nil
  (progn
    (send *baskets* :newcoords  (send (send (send *fetch* :base_link_lk) :copy-worldcoords) :translate #f(200 0 450)))
    (objects (list *fetch* *7f* *baskets*))
    (send *irtviewer* :look-all *fetch*)
;;    (send *baskets* :translate #f(1000 0 400))
    )
)
