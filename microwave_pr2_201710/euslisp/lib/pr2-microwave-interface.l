(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :microwave "package://microwave_pr2_201710/euslisp/lib/microwave.l")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :util "package://microwave_pr2_201710/euslisp/lib/util.l")

(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(defparameter *detection-topic* "/ObjectDetection")


(defclass pr2-microwave-interface
    :super object
  :slots (use-ri
          microwave-pos microwave-rpy)
  )

(defmethod pr2-microwave-interface
    (:init
     (&optional (use-ri- nil))

        (setq use-ri use-ri-)
        ;; init ROS Node
        (ros::roseus "pr2-microwave")

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
        (send *microwave* :move-to (make-coords :pos #f(0 0 1000)))
        ;;(send *microwave* :move-to (make-coords :pos #f(2000 7457 31000) :rpy (float-vector (/ pi 2.0) 0 0)) :world)

        ;; show objects in irtviewer
        (objects (list *scene* *microwave* *pr2*))
        (send *irtviewer* :look-all *pr2*)

        ;; register spot of microwave
        (let ((microwave-front-spots (make-cascoords :pos #f(2280 7100 30000) :rpy (float-vector (/ pi 2.0) 0 0) :name "microwave-front-spot")))
          (send *scene* :add-spots (list microwave-front-spots))
          )
        )

  (:go-to-microwave-roughly
   ()
   (if use-ri (send *ri* :move-to (send *scene* :spot "microwave-front-spot") :frame-id "world"))

   (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *microwave* :move-to (make-coords :pos #f(0 0 1000)))

   (send *pr2* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *pr2* :move-to (make-coords :pos #f(-700 0 0)))
   (send *pr2* :reset-pose)

   (send *irtviewer* :look-all *pr2*)
   (send *irtviewer* :look-all *pr2*)

   ;;(if use-ri (send *ri* :go-pos-unsafe -0.5 1.5 0))
   )

  (:go-to-microwave-accurately
   ()
   (let ((recog-flag nil)
         (cds))
     (if (null use-ri)
         (progn
           (setq microwave-pos (send *microwave* :pos))
           (setq microwave-rpy (send *microwave* :rpy-angle))

           (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
           (send *microwave* :translate (float-vector -185 450 -270))  ;; x奥行き y横 z高さ  TODO paratun
           )
         (progn
           (while (null recog-flag)
             (send *microwave* :move-to (send *ri* :state :worldcoords) :world)
             (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

             (send *pr2* :head :neck-p :joint-angle 18)
             (send *ri* :angle-vector (send *pr2* :angle-vector))
             (send *ri* :wait-interpolation)

             (setq cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
             (send *microwave* :move-to (make-coords :pos (send cds :pos)))
             (setq microwave-pos (send *microwave* :pos))
             (setq microwave-rpy (send *microwave* :rpy-angle))

             (let* ((local-microwave-x (elt (send cds :worldpos) 0))
                    (local-microwave-y (elt (send cds :worldpos) 1))
                    (local-microwave-theta (car (car (rpy-angle (send cds :worldrot)))))
                    (local-microwave-without-z (make-coords :pos (float-vector local-microwave-x local-microwave-y 0) :rpy (list local-microwave-theta 0 0)))
                    (world-microwave-without-z ;; pose of microwave in world when z = 0
                     (send (send (send *pr2* :worldcoords) :copy-worldcoords) :transform local-microwave-without-z))
                    (relative-coords
                     (make-coords :pos (float-vector 600 0 0)))
                    (reach-world-microwave-without-z ;; pose in front of microwave in world when z = 0
                     (send (send world-microwave-without-z :copy-worldcoords) :transform relative-coords))
                    (reach-local-microwave-local-without-z
                     (send (send (send *pr2* :worldcoords) :copy-worldcoords)
                           :transformation reach-world-microwave-without-z))
                    )
               (send world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(1 0 0))
               (send reach-world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(0 1 0))
               (let* ((go-pos-x (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 0)))
                      (go-pos-y (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 1)))
                      (go-pos-theta (+ (rad2deg (car (car (rpy-angle (send reach-local-microwave-local-without-z :worldrot))))) 180))
                      )
                 (if (> go-pos-theta 180)
                     (setq go-pos-theta (- go-pos-theta 360)))
                 (if (< go-pos-theta -180)
                     (setq go-pos-theta + go-pos-theta 360))

                 (format t "go-pos x:~a y:~a theta:~a~%"
                         go-pos-x go-pos-y go-pos-theta)
                 (if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 0.8))
                     (setq recog-flag t)
                     (when (y-or-n-p "send to real robot?")
                       (send *ri* :go-pos-unsafe go-pos-x go-pos-y go-pos-theta)
                       )
                     )
                 )
               )
             )
           (if use-ri (send *ri* :go-pos-unsafe 0.15 0 0))
           )
         )
     )
   )

  (:open-microwave
   (&optional (wait? nil))
   (when use-ri
     (send *ri* :stop-grasp :larm)
     (send *ri* :wait-interpolation))

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :translate (float-vector -185 450 -270))  ;; x奥行き y横 z高さ  TODO paratun
   (send *microwave* :angle-vector (float-vector 180))

   (let* ((handle-coords))
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send *pr2* :reset-pose)
     (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
           :rotation-axis t)
     (if wait? (wait-enter))
     (when use-ri
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation))

     (let ((i 180))
       (while (> i 99)

         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
               ;;:revert-if-fail nil
               :rotation-axis t)

         (if wait? (wait-enter))
         (when use-ri
           (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
           (send *ri* :wait-interpolation))
         (setq i (- i 10))
         )
       )

     (progn   ;; pull left hand from microwave
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
             :rotation-axis t)
       (if wait? (wait-enter))
       (when use-ri
         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

       ;;(send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :translate #f(-10 0 60))
       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-10 0 60))
             :rotation-axis t)
       (if wait? (wait-enter))
       (when use-ri
         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-50 -10 60))
             :rotation-axis t)
       (if wait? (wait-enter))
       (when use-ri
         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-200 0 10))
             :rotation-axis t)
       (if wait? (wait-enter))
       (when use-ri
         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))
       )
     )
   )

  (:close-microwave
   (&optional (wait? nil))
   (let* ((handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
          (ri-coords))
     (if use-ri
         (setq ri-coords (send (send *ri* :state :worldcoords) :copy-worldcoords))
         (setq ri-coords (send (send *pr2* :worldcoords) :copy-worldcoords)))

     (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
     (send *microwave* :translate (float-vector -185 450 -270))  ;; x奥行き y横 z高さ  TODO paratun
     ;; (send *microwave* :translate (float-vector 0 600 -120))
     (send *microwave* :angle-vector (float-vector 100))

     (send *pr2* :reset-pose)
     (send *pr2* :larm :inverse-kinematics (send (send ri-coords :copy-worldcoords) :move-to (make-coords :pos #f(100 500 700)))
           :rotation-axis nil)
     (if wait? (wait-enter))
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
           (send *ri* :wait-interpolation))

     (send *pr2* :larm :inverse-kinematics (send (send ri-coords :copy-worldcoords) :move-to (make-coords :pos #f(200 600 1000)))
           :rotation-axis nil)
     (if wait? (wait-enter))
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
           (send *ri* :wait-interpolation))

     (let ((i 110))
       (while (< i 161)
         (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
         (send *microwave* :translate (float-vector -185 450 -270))  ;; x奥行き y横 z高さ  TODO paratun
         ;;(send *microwave* :translate (float-vector 0 600 -120))

         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate #f(-100 -100 150)) :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))

         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
               ;;:debug-view t
               :rotation-axis nil)

         (if wait? (wait-enter))
         (when use-ri
           (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
           (send *ri* :wait-interpolation))
         (setq i (+ i 10))
         )
       )
     )
   )

  (:put-object
   (&optional (wait? nil))
   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :translate (float-vector -185 450 -270))  ;; x奥行き y横 z高さ  TODO paratun
   (send *microwave* :angle-vector (float-vector 100))

   ;; pre-put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(-100 -200 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   ;; put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(100 -200 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   ;; post-put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(-100 -200 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))
   )

  (:push-button
   (&optional (wait? nil))
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
  ;;(setq *pmi* (instance pr2-microwave-interface :init t))   ;;if use real robot

  (send *pmi* :go-to-microwave-roughly)
  (send *pmi* :go-to-microwave-accurately)
  (send *pmi* :open-microwave t)
  (send *pmi* :put-object t)
  (send *pmi* :close-microwave t)
  )
