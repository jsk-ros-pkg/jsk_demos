(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (load "package://fetcheus/fetch.l")
  (load "package://fetcheus/fetch-interface.l")
  ;;(setq *fetch* (fetch))
  (fetch-init)
  )

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(objects (list *room73b2* *nao* *fetch* *pr2*))

(defun wait-for-human ()
  ;;(send *pr2* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  
  (send *irtviewer* :draw-objects)
  )

(defun write()
  (send *fetch* :move-to (make-coords :pos #f(5000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *fetch* :reset-pose)
  (send *fetch* :rarm :move-end-pos #f(0 -400 250) :local :rotation-axis t :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :z :local :debug-view t)
  (send *fetch* :rarm :move-end-rot -90 :x :local :debug-view t)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  )

;; (send *fetch* :rarm :move-end-pos #f(50 0 50) :local :rotation-axis t :debug-view t)
;; (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)

(defun write-x-y-z (x y z &key (debug-view t)) ;; 
  ;;(send *fetch* :rarm :move-end-pos (float-vector x y z) :local :rotation-axis t :debug-view debug-view)
  (let ((base-pos (float-vector 750 0 850)))
    ;;(send *fetch* :reset-pose)
    (send *fetch* :rarm :inverse-kinematics
	  (send (make-cascoords :pos (v+ base-pos (float-vector x y z))) :transform *fetch* :parent)
	  :rotation-axis t
	  :look-at-target t
	  :use-torso nil
	  :debug-view debug-view)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)))

;;(defun write_I()
  ;;(send *fetch* :rarm :move-end-pos #f(0 0 50) :local :rotation-axis t :debug-view t)
  ;;(send *fetch* :rarm :move-end-pos #f(50 50 0) :local :rotation-axis t :debug-view t)
  ;;(send *fetch* :rarm :move-end-pos #f(0 0 -50) :local :rotation-axis t :debug-view t)
  ;;(send *fetch* :rarm :move-end-pos #f(50 0 0) :local :rotation-axis t :debug-view t)
  ;;(write-x-y-z 50 0 -50)
  ;;(write-x-y-z -50 0 0)
  ;;(write-x-y-z 0 50 0)
  ;;(write-x-y-z 0 -50 50)
  ;;)

(defun write-line (command-list) ;; defun write_line (command_list):
  (ros::ros-warn "command-list ~A" command-list)
  (when (car command-list) ;; car x == x[0]
    (send *fetch* :rarm :move-end-pos (float-vector (elt (car command-list) 0)
						    (elt (car command-list) 1)
						    50)
	  :local :rotation-axis t :debug-view nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation))
  (dolist (command command-list) ;;   for command in command_list: ;; for i in [1,2,3]:
    (ros::ros-warn "    command ~A" command)
    (write-x-y-z
     (elt command 0) ;; command[0]
     (elt command 1) ;; command[1]
     0
     :debug-view nil
     )
    );; dolist
  (send *fetch* :rarm :move-end-pos #f(0 0 50) :local :rotation-axis t :debug-view nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ) ;; defun


;;(defun write-I ()
  ;;(write-line (list  (list 0 50)))
  ;;(write-x-y-z 0 -25 0 :debug-view nil)
 ;; (write-line (list  (list -50 0)))
  ;;(write-x-y-z 0 25 0 :debug-view nil)
  ;;(write-line (list  (list 0 50)))
  ;;)


(defun move-x-y (x y) ;; 
  ;;(send *fetch* :rarm :move-end-pos (float-vector x y z) :local :rotation-axis t :debug-view debug-view)
  ;;(let ((base-pos (float-vector 750 0 850)))
    ;;(send *fetch* :reset-pose)
    (send *fetch* :rarm :move-end-pos #f(0 0 50) :local :rotation-axis t :debug-view nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *fetch* :rarm :move-end-pos (float-vector x y 0) :local :rotation-axis t :debug-view nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    (send *fetch* :rarm :move-end-pos #f(0 0 -50) :local :rotation-axis t :debug-view nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    )
(defun line-x-y (x y)
   (send *fetch* :rarm :move-end-pos (float-vector x y 0) :local :rotation-axis t :debug-view nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    )

(defun write_I()
  (line-x-y 0 -50)
  ;;(move-x-y (-25 0))
  )
