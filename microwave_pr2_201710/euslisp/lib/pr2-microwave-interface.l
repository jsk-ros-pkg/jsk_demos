(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :microwave "package://microwave_pr2_201710/euslisp/lib/microwave.l")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :util "package://microwave_pr2_201710/euslisp/lib/util.l")

(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
;;(defparameter *detection-topic* "/ObjectDetection")   ;; TODO


(defclass pr2-microwave-interface
  :super object
  :slots (microwave-pos microwave-rpy)
  )

(defmethod pr2-microwave-interface
  (:init
   ()

   ;; init ROS Node
   (ros::roseus "pr2-microwave")

   ;; init 73B2 room
   (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

   ;; init PR2
   (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
   (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
   (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

   ;; init microwave
   (unless (boundp '*microwave*) (setq *microwave* (microwave-init)))
   (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *microwave* :move-to (make-coords :pos #f(0 0 1000)))

   ;; show objects in irtviewer
   (objects (list *scene* *microwave* *pr2*))
   (send *irtviewer* :look-all *pr2*)

   ;; register spot of microwave
   (let ((microwave-front-spots (make-cascoords :pos #f(2280 7100 30000) :rpy (float-vector (/ pi 2.0) 0 0) :name "microwave-front-spot")))
     (send *scene* :add-spots (list microwave-front-spots))
     )
   )

  (:go-forward
   ()
   (send *ri* :go-pos-unsafe 0.15 0 0)
   )

  (:go-backward
   ()
   (send *ri* :go-pos-unsafe -0.15 0 0)
   )

  (:go-to-microwave-roughly
   ()
   (send *ri* :move-to (send *scene* :spot "microwave-front-spot") :frame-id "world")

   (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *microwave* :move-to (make-coords :pos #f(0 0 1000)))

   (send *pr2* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *pr2* :move-to (make-coords :pos #f(-700 0 0)))
   (pr2-left-reset-pose)

   (send *irtviewer* :look-all *pr2*)
   (send *irtviewer* :look-all *pr2*)
   )

  (:go-to-microwave-accurately
   ()
   (let ((recog-flag nil)
         (cds))
     (while (null recog-flag)
       (send *microwave* :move-to (send *ri* :state :worldcoords) :world)
       (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

       (send *pr2* :head :neck-p :joint-angle 28)
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       (send *ri* :wait-interpolation)

       (setq cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
       (send *microwave* :move-to (make-coords :pos (send cds :pos)))
       (send *microwave* :translate (float-vector 115 450 -220))  ;; x奥行き y横 z高さ
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
           (if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 1.2))
               (setq recog-flag t)
             (progn
               (let (go-pos-x- go-pos-y- go-pos-theta-)
                 ;; clipping how robot moves because using :go-pos-`unsafe`
                 (setq go-pos-x- (min go-pos-x 0.1))
                 (setq go-pos-x- (max go-pos-x- -0.1))
                 (setq go-pos-y- (min go-pos-y 0.1))
                 (setq go-pos-y- (max go-pos-y- -0.1))
                 (setq go-pos-theta- (min go-pos-theta 3))
                 (setq go-pos-theta- (max go-pos-theta- -3))

                 (send *ri* :go-pos-unsafe go-pos-x- go-pos-y- go-pos-theta-)
                 )
               )
             )
           )
         )
       )
     (send *ri* :go-pos-unsafe -0.15 0 0)
     )
   )

  (:open-microwave
   (&optional (wait? nil))
   (send *ri* :stop-grasp :larm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

   (let ((handle-coords))
     (pr2-left-reset-pose)

     (progn   ;; pre-open1
       (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
       (send handle-coords :translate (float-vector 0 -30 -50))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)

       (progn   ;; pre-open2
         (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send handle-coords :translate (float-vector 0 0 50))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
         (if wait? (wait-enter))
         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation)
         )
       )

     (let ((i 170)   ;; open
           (av)
           (tms (list))
           (avs (list)))
       (while (> i 109)
         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
         (setq av (send *pr2* :angle-vector))
         (setq avs (append avs (list av)))
         (setq tms (append tms (list 100)))

         (setq i (- i 5))
         )
       (send *ri* :angle-vector-sequence avs tms)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; detach left hand from microwave
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-10 0 60)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-50 -10 60)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate #f(-200 0 10)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)

       (pr2-left-reset-pose)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )
     )
   )

  (:close-microwave
   (&optional (wait? nil))
   (let* ((handle-coords))
     (pr2-left-reset-pose)

     (progn   ;; pre-hang left hand to microwave
       (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate #f(-100 -100 100)) :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; hang left hand to microwave
       (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate #f(-100 -100 130)) :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

     (let ((i 110)   ;; open
           (av)
           (tms (list))
           (avs (list)))
       (while (< i 216)
         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate #f(-100 -100 130)) :worldpos)
                                          :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
         (setq av (send *pr2* :angle-vector))
         (setq avs (append avs (list av)))
         (setq tms (append tms (list 100)))
         (setq i (+ i 10))
         )
       (send *ri* :angle-vector-sequence avs tms)
       (send *ri* :wait-interpolation)
       )
     )

   (pr2-left-reset-pose)
   (if wait? (wait-enter))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
   (send *ri* :wait-interpolation)
   )

  ;; TODO
  (:put-object
   (&optional (wait? nil))
   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 100))

   (progn   ;; pre-put
     (send *pr2* :rarm :angle-vector #f(-23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 120)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; put
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 120)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *ri* :stop-grasp :rarm)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put1
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 150)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put2
     (send *pr2* :reset-pose)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )
   )

  ;; TODO
  (:take-object
   (&optional (wait? nil))
   (send *ri* :stop-grasp :rarm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 100))

   (progn   ;; pre-put
     (send *pr2* :rarm :angle-vector #f(-23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 150)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; put
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *ri* :start-grasp :rarm)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put1
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 150)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put2
     (pr2-left-reset-pose)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :angle-vector #f(-23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
     (send *ri* :wait-interpolation)
     )
   )

  ;; TODO
  (:push-heat-button
   (&optional (wait? nil))
   (send *pr2* :reset-pose)
   (send *ri* :start-grasp :rarm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

   (let ((heat-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -505 115))))
         (heat-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector 10 -505 115)))))

     (progn   ;; pre-push
       (send *pr2* :rarm :inverse-kinematics heat-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; push
       (send *pr2* :rarm :inverse-kinematics heat-button :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; post-push
       (send *pr2* :rarm :inverse-kinematics heat-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
       (send *ri* :wait-interpolation)
       )
     )
   )

  ;; TODO
  (:push-start-button
   (&optional (wait? nil))
   (send *pr2* :reset-pose)
   (send *pr2* :start-grasp :rarm)
   (send *ri* :start-grasp :rarm)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

   (let ((start-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -510 170))))
         (start-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -5 -510 170)))))

     (progn ;; pre-push
       (send *pr2* :rarm :inverse-kinematics start-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; push
       (send *pr2* :rarm :inverse-kinematics start-button :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; post-push
       (send *pr2* :rarm :inverse-kinematics start-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
       (send *ri* :wait-interpolation)
       )
     )

   (send *pr2* :reset-pose)
   (if wait? (wait-enter))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
   (send *ri* :wait-interpolation)
   )
  )
