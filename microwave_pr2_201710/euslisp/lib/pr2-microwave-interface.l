;;(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(require :pr2 "package://pr2eus/pr2-interface.l")
(require :microwave "package://microwave_pr2_201710/euslisp/lib/microwave.l")
;;(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(load "package://jsk_maps/src/eng2-scene.l")
(load "package://microwave_pr2_201710/euslisp/lib/util.l")

(defclass pr2-microwave-interface
    :super object
  :slots (use-ri)
  )

(defmethod pr2-microwave-interface
    (:init
     (&optional (use-ri- nil))

        (setq use-ri use-ri-)
        ;; init ROS Node
        (ros::load-ros-manifest "pr2-microwave")

        ;; init detection
        ;;(defparameter *detection-topic* "/ObjectDetection")

        ;; init 73B2 room
        (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

        ;; init PR2
        (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
        (if use-ri
            (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init))))
        (if use-ri
            (send *pr2* :move-to (send *ri* :state :worldcoords) :world))

        ;; init microwave
        (unless (boundp '*microwave*) (setq *microwave* (microwave-init)))
        (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
        (send *microwave* :move-to (make-coords :pos #f(0 0 900)))

        ;; show objects in irtviewer
        (objects (list *scene* *microwave* *pr2*))
        ;;(send *irtviewer* :look-all *pr2*)
        )

  (:go-to-microwave
   ()
   (if use-ri
       (progn
         )
       (progn
         (send *pr2* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
         (send *pr2* :move-to (make-coords :pos #f(-600 0 0)))
         (send *irtviewer* :look-all *pr2*)
         (send *irtviewer* :look-all *pr2*)
         ))
   )

  (:open-microwave
   (&optional (wait? nil))
   (let* ((handle-coords))
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
           ;;:revert-if-fail nil
           :rotation-axis t)

     (let ((i 180))
       (while (> i 99)
         (setq i (- i 10))

         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
               ;;:revert-if-fail nil
               :rotation-axis t)

         (wait-enter)
         )
       )
     )
   )

  (:close-microwave
   (&optional (wait? nil))
   (let* ((handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
          (handle-dest-coords (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)))
     (send *pr2* :larm :inverse-kinematics handle-dest-coords
           :rotation-axis t)

     (let ((i 100))
       (while (< i 181)
         (send *microwave* :angle-vector (float-vector i))
         (send *pr2* :larm :inverse-kinematics handle-dest-coords
               :rotation-axis t)

         (wait-enter)
         (setq i (+ i 10))
         )
       )
     )
   )

  #|
(:put-object-in-microwave
()
)

(:take-object-in-microwave
()
)

(:push-switch
()
)
|#
)

(defun execute ()
  (setq *pmi* (instance pr2-microwave-interface :init))
  (send *pmi* :go-to-microwave)
  (send *pmi* :open-microwave t)
  ;;(send *pmi* :close-microwave t)
  )
