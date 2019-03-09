(ros::load-ros-manifest "jsk_recognition_msgs")
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(setq *co* (instance collision-object-publisher :init))
(setq *baskets* (make-cube 400 500 200))

;;台の前にたどり着いた時に, 俯瞰できるようなポーズ
(setq look-down-pose #f(386.15 75.6359 80.1929 -11.4118 98.5702 0.060285 95.1322 -0.060071 -0.895055 42.9474))

(setq mid-point-angle #f(381.933 -43.4998 14.3629 92.2113 128.973 -99.8494 101.329 81.9859 0.521157 2.82537))
(setq basket-red-angle #f(361.506 2.33516 22.5147 61.6253 128.973 -100.992 84.8051 -32.3159 0.147273 75))
(setq basket-green-angle #f(362.231 -37.699 41.0157 67.4041 122.015 -135.972 96.1869 87.6768 0.577807 75))
(setq basket-blue-angle #f(362.238 -86.3245 41.0816 67.3822 128.973 -135.687 101.065 48.4995 0.566469 75))

(setq look-angle-1 #f(361.903 -39.4788 41.1036 100.693 92.0004 -131.446 -78.6275 137.489 -4.32794 11.5485))
(setq look-angle-2 #f(361.895 -39.1712 41.0596 100.715 92.6156 -130.04 -78.6495 -130.182 -2.54918 15.5695))
(setq look-angle-3 #f(361.895 -37.8968 41.0377 100.429 92.6376 -130.04 -78.7154 -49.0591 -2.52651 15.5695))
(setq look-angle-4 #f(361.888 -37.8528 41.0377 100.385 92.6156 -129.996 -78.7594 52.0591 -2.49254 15.6574))
(setq look-angle-5 #f(361.529 -24.4935 43.4547 142.419 57.2836 -164.471 -123.759 74.8008 -10.9444 34.3781))
(setq *cube* (make-cube 100 100 100))
(send *cube* :newcoords (make-coords :pos #f(3237.32 8094.795 30749.2 ) :rpy #f(1.563 0.003 0.003)))


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
    	(send *co* :add-object *baskets* :frame-id "base_link" :relative-pose (make-coords :pos #f(250 0 450)))
    )    
  (progn
;;    (send *ri* :angle-vector mid-point-angle 10000)
;;    (send *ri* :wait-interpolation)
    ;;address に応じて, カゴの上空まで腕を伸ばす
    (cond
     ((string= address "red") (send *ri* :angle-vector basket-red-angle 10000)  (send *fetch* :angle-vector basket-red-angle ))
     ((string= address "green") (send *ri* :angle-vector basket-green-angle 10000)  (send *fetch* :angle-vector basket-green-angle ))
     ((string= address "blue") (send *ri* :angle-vector basket-blue-angle 10000)  (send *fetch* :angle-vector basket-blue-angle ))
     )
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    ;;描画    
    (dv)
    ;;(send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
    ;;(send *ri* :wait-interpolation)
;;    (if set-collision-object (send *co* :wipe-all))
    )
  )

(defun address-color-check nil
  (print "checking color...")
  (cond ((> (send (one-shot-subscribe "/address_detection/red/euclidean_clustering/cluster_num" jsk_recognition_msgs::Int32Stamped) :data) 0)
	 (return-from address-color-check "red"))
	((> (send (one-shot-subscribe "/address_detection/green/euclidean_clustering/cluster_num" jsk_recognition_msgs::Int32Stamped) :data) 0)
	 (return-from address-color-check "green"))
	;; ((> (one-shot-subscribe "/address_detection/red/euclidean_clustering/cluster_num" jsk_recognition_msgs::Int32Stamped) 0)
	;;  (return "blue"))
	 (t nil)
	)
 )

(defun check-address nil
  (progn
    (let (address)
      (send *fetch* :angle-vector look-angle-1)
      (send *ri* :angle-vector look-angle-1 10000)
      (send *ri* :wait-interpolation)
      (setq address (address-color-check))
      (if address (return-from check-address address))
      
      (send *fetch* :angle-vector look-angle-2)
      (send *ri* :angle-vector look-angle-2 3000)
      (send *ri* :wait-interpolation)
      (setq address (address-color-check))
      (if address (return-from check-address address))
      
      (send *fetch* :angle-vector look-angle-3)
      (send *ri* :angle-vector look-angle-3 3000)
      (send *ri* :wait-interpolation)
      (setq address (address-color-check))
      (if address (return-from check-address address))
      
      (send *fetch* :angle-vector look-angle-4)
      (send *ri* :angle-vector look-angle-4 3000)
      (send *ri* :wait-interpolation)
      (setq address (address-color-check))
      (if address (return-from check-address address))
      

      (send *fetch* :angle-vector look-angle-5)
      (send *ri* :angle-vector look-angle-5 10000)
      (send *ri* :wait-interpolation)
      (setq address (address-color-check))
      (if address (return address))      
      )
    )
)
