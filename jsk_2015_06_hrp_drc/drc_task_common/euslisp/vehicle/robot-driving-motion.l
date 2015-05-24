#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(defclass robot-driving-motion
  :super propertied-object
  :slots (robot ;; robot model
          car ;; car model
          real ;; use real robot or not
          steering-arm-ik-seed
          ) 
  )

(defmethod robot-driving-motion
  (:init
   (&key ((:robot tmp-robot) nil) ((:car tmp-car) nil) ((:real tmp-real) nil)
         (grasp-arm :rarm) (grasp-point :handle-top))
   (setq *robot* tmp-robot)
   (setq *car* tmp-car)
   (setq robot tmp-robot
         car tmp-car
         real tmp-real
         steering-arm-ik-seed nil
         )
   (when real
     ;; (send *ri* :set-interpolation-mode :linear) ;; avoid sudden change in joint angles when steering
     (send *ri* :set-interpolation-mode :hoffarbib)
     ;; disable balancer plugins
     (send *ri* :stop-st)
     (send *ri* :stop-auto-balancer)
     )
   (send self :put :grasp-arm grasp-arm)
   (send self :put :grasp-point grasp-point)
   (send self :ride :init-pose t)
   (objects (list robot car))   
   )  

  ;; initial methods
  (:drive-init-pose ;; initial pose for driving task
   (&key (tm 10000))
   (warn ":drive-init-pose should be impremented for each robots~%")
   )
  (:ride ;; move robot onto the car in model environment
   (&key (init-pose t))
   (warn ":ride should be impremented for each robots~%")
   (when init-pose
     (send self :drive-init-pose)
     )
   )
  (:drive-init-pose-crank
   (&key (limb (send self :get :grasp-arm)) (tm 5000) (drive-init-pose nil) (move-arm t))
   (when (not (equal limb (send self :get :grasp-arm)))
     (send self :grasp-arm limb)
     )
   (send self :grasp-point :handle-crank)
   (when drive-init-pose
     (send self :drive-init-pose)
     )
   (when move-arm
     (send robot limb :move-end-pos #f(100 0 100) :world)
     )
   (when (and real move-arm)
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )

  ;; wrapper function for hand
  (:open-hand
   (&optional (limb (send self :grasp-arm)))
   (warn ":open-hand should be impremented for each robots~%")
   )
  (:close-hand
   (&optional (limb (send self :grasp-arm)))
   (warn ":close-hand should be impremented for each robots~%")
   )

  ;; handling
  (:approach-handle
   (&rest args &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point)) (sync-controller nil)
          (tm 7500) (offset #f(-100 0 0)) (rot-offset 0)
          (debug-view nil) (rotation-axis t) (start-impedance nil) (stop-impedance nil)
          (null-space #f(-0.06 -0.3 0.0 -0.18 0.16 0.0 0.0 0.0))
          (allowable-theta 180) (iteration-theta 5)
          &allow-other-keys
          )
   (when start-impedance
     (send self :set-impedance-for-handle limb)
     )
   (send self :open-hand limb)
   (when (and sync-controller (not (send *ri* :simulation-modep)))
     (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
     )
   (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
         (body-link (elt (send robot :links) 0))
         (handle-link (find-if #'(lambda (h) (eq (send h :name) :handle-link)) (send (send *car* :vehicle-handle) :links)))
         (target-handle-coords-org (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) :copy-worldcoords))
         (old-angle-vector (send robot limb :angle-vector))
         (rot-angle 0) (ik-result nil) target-handle-coords
         )
     (labels ((approach-ik
              (target)
              (let (result)
                ;; first, move roughly without collision detection
                (setq result
                      (send robot :inverse-kinematics target
                            :move-target (send robot limb :end-coords)
                            :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                            :rotation-axis nil :debug-view debug-view
                            )
                      )
                (when result ;; execute only if approach ik is succeed
                  ;; second, re-solve inverse kinematics considering collision
                  (setq result
                        (send robot :inverse-kinematics target
                              :move-target (send robot limb :end-coords)
                              :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                              :rotation-axis rotation-axis :debug-view debug-view
                              :avoid-collision-null-gain 100
                              :avoid-collision-joint-gain 0.1
                              :collision-avoidance-link-pair (concatenate cons
                                                                          (mapcar #'(lambda (x) (list x body-link)) check-links)
                                                                          (mapcar #'(lambda (x) (list x handle-link)) check-links))
                              :additional-check
                              #'(lambda ()
                                  (not (some #'null (mapcar #'(lambda (x) (> (car (pqp-collision-distance x body-link)) distance-threshold))
                                                            check-links))))
                              )
                        )
                  )
                (unless result  ;; revert first ik when second ik failed
                  (send robot limb :angle-vector old-angle-vector)
                  )
                result
                )
              ))
       ;; define target-handle-coords
       (while (and (< rot-angle allowable-theta) (not ik-result))
         (setq target-handle-coords
               (send (send (send target-handle-coords-org :copy-worldcoords) :rotate (deg2rad rot-angle) :z :local)
                     :translate offset :local)
               )
         (setq ik-result (approach-ik target-handle-coords))
         (when (not ik-result)
           (setq target-handle-coords
                 (send (send (send target-handle-coords-org :copy-worldcoords) :rotate (deg2rad (- rot-angle)) :z :local)
                       :translate offset :local)
                 )
           (setq ik-result (approach-ik target-handle-coords))
           )
         (setq rot-angle (+ rot-angle iteration-theta))
         )
       )
     (when (and real ik-result)
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       (when stop-impedance
         (send *ri* :stop-impedance limb)
         )
       )
     ik-result ;; return result of ik (nil is failed)
     )
   )
  ;; old approach-handle
  ;; (:approach-handle
  ;;  (&rest args &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point)) (sync-controller nil)
  ;;         (tm 7500) (offset #f(-150 0 0)) (rot-offset 0)
  ;;         (debug-view nil) (rotation-axis t) (start-impedance nil) (stop-impedance nil)
  ;;         (null-space #f(-0.06 -0.3 0.0 -0.18 0.16 0.0 0.0 0.0))
  ;;         &allow-other-keys
  ;;         )
  ;;  (when start-impedance
  ;;    (send self :set-impedance-for-handle limb)
  ;;    )
  ;;  (send self :open-hand limb)
  ;;  (when (and sync-controller (not (send *ri* :simulation-modep)))
  ;;    (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
  ;;    )
  ;;  (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
  ;;        (body-link (elt (send robot :links) 0))
  ;;        (handle-link (find-if #'(lambda (h) (eq (send h :name) :handle-link)) (send (send *car* :vehicle-handle) :links)))
  ;;        (offset-target-handle-coords
  ;;         (send (send (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle))
  ;;                           :copy-worldcoords)
  ;;                     :translate offset)
  ;;               :rotate rot-offset :z (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) ;; grasp-point is assumed to be a pole and axis :z is to be grasped (:x, :y cannot be rotated to grasp) 
  ;;               )
  ;;         )
  ;;        (old-angle-vector (send robot limb :angle-vector))
  ;;        ik-result
  ;;        )
  ;;    ;; first, move roughly without collision detection
  ;;    (setq ik-result
  ;;          (send robot :inverse-kinematics offset-target-handle-coords
  ;;                :move-target (send robot limb :end-coords)
  ;;                :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
  ;;                :rotation-axis nil :debug-view debug-view
  ;;                ;; :null-space null-space ;; avoid rarm to outside
  ;;                ;; :null-space #f(30.0 -150.0 0.0 90.0 80.0 0.0 20.0 0.0) ;; avoid rarm to outside
  ;;                )
  ;;          )
  ;;    (when ik-result ;; execute only if approach ik is succeed
  ;;      ;; second, re-solve inverse kinematics considering collision
  ;;      (setq ik-result
  ;;            (send robot :inverse-kinematics offset-target-handle-coords
  ;;                  :move-target (send robot limb :end-coords)
  ;;                  :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
  ;;                  :rotation-axis rotation-axis :debug-view debug-view :stop 1000
  ;;                  :avoid-collision-null-gain 100
  ;;                  :avoid-collision-joint-gain 0.1
  ;;                  :collision-avoidance-link-pair (concatenate cons
  ;;                                                              (mapcar #'(lambda (x) (list x body-link)) check-links)
  ;;                                                              (mapcar #'(lambda (x) (list x handle-link)) check-links))
  ;;                  :additional-check
  ;;                  #'(lambda ()
  ;;                      (not (some #'null (mapcar #'(lambda (x) (> (car (pqp-collision-distance x body-link)) distance-threshold))
  ;;                                                check-links))))
  ;;                  )
  ;;            )
  ;;      (unless ik-result
  ;;        (send robot limb :angle-vector old-angle-vector) ;; revert when second ik failed
  ;;        )
  ;;      )
  ;;    (when (and real ik-result)
  ;;      (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;      (send *ri* :wait-interpolation)
  ;;      (when stop-impedance
  ;;        (send *ri* :stop-impedance limb)
  ;;        )
  ;;      )
  ;;    ik-result ;; return result of ik (nil is failed)
  ;;    )
  ;;  )

  (:grasp-handle
   (&rest args &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
          (debug nil) (tm 7000) (debug-view nil) (start-impedance t) (rotation-axis :z) (adjust nil)
          (update-ik-seed t) (offset #f(0 0 0)) (offset-wrt :local)
          &allow-other-keys)
   ;; (send self :open-hand limb)
   (when (and real start-impedance)
     (send self :set-impedance-for-handle limb)
     )
   (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
         (body-link (elt (send robot :links) 0)) (old-angle-vector (send robot limb :angle-vector))
         ik-result
         )
     (setq ik-result
           (send robot :inverse-kinematics 
                 (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) :copy-worldcoords)
                 :move-target (send robot limb :end-coords)
                 :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                 :rotation-axis rotation-axis :debug-view debug-view
                 :avoid-collision-null-gain 100
                 ;;:avoid-collision-distance 200
                 :avoid-collision-joint-gain 0.1
                 :collision-avoidance-link-pair (mapcar #'(lambda (x) (list x body-link)) check-links)
                 :additional-check
                 #'(lambda ()
                     (not (some #'null (mapcar #'(lambda (x) (> (car (pqp-collision-distance x body-link)) distance-threshold))
                                               check-links))))
                 )
           )
     (when ik-result ;; execute only when ik is succeed
       (setq ik-result (send robot limb :move-end-pos offset offset-wrt))
       (if ik-result
           (progn 
             (when real
               (send *ri* :angle-vector (send robot :angle-vector) tm)
               (send *ri* :wait-interpolation)
               )
             (send self :close-hand limb)
             ;; for exchange
             (send self :put :grasp-arm limb)
             (send self :put :grasp-point target-handle)
             (when adjust
               (send self :adjust-real-handle-coords)
               )
             (when update-ik-seed ;; update steering-arm-ik-seed when grasp
               (send self :update-steering-arm-ik-seed)
               )
             )
         (send robot limb :angle-vector old-angle-vector) ;; resume
         )
       )
     ik-result ;; return result of ik (nil is failed)
     )
   )
  (:release-handle
   (&rest args &key (limb (send self :get :grasp-arm))
          (tm 7000) (debug-view nil) (rotation-axis :z) (sync-controller t)
          (offset #f(-100 0 0)) ;; offset should be local in end-coords
          &allow-other-keys
          )
   (when (and sync-controller (not (send *ri* :simulation-modep)))
     (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
     )
   (let (ik-result)
     (send self :open-hand)
     (setq ik-result (send robot limb :move-end-pos offset :local
                           :debug-view debug-view :rotation-axis rotation-axis))
     (when (and real ik-result)
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       )
     ik-result
     )
   )
  (:fix-hand-position
   (diff-position &key (wrt :world)
                  (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
                  (tm 10000) (offset #f(-150 0 0)) (debug-view nil) (rotation-axis t)
                  ;; (null-space #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
                  )
   (send self :move-robot-position (scale -1 diff-position) :wrt wrt)
   (send self :approach-handle
         :limb limb :target-handle target-handle
         :tm tm :offset offset :debug-view debug-view
         :rotation-axis rotation-axis
         ;; :null-space null-space
         )
   )
  (:adjust-real-handle-coords ;; assuming real robot is exactly grasping handle-coords (6DoF coords is exactly same as target handle-coords)
   (&key (use-real-rpy nil))
   (let (model-handle-coords real-handle-coords transform-from-model-to-real)
     ;; get real robot state
     (send robot :angle-vector (send *ri* :state :potentio-vector))
     (when use-real-rpy
       (let* ((root-coords (send (car (send robot :links)) :worldcoords))
              (ref-yaw (caar (send root-coords :rpy-angle)))
              (ref-pos (send (funcall func robot) :worldpos))
              (act-ypr (car (send (send (car (send (send *ri* :robot) :links)) :worldcoords) :rpy-angle)))
              )
           (send robot :move-coords
                 (make-coords :pos ref-pos
                              :rpy (list ref-yaw (elt act-ypr 1) (elt act-ypr 2)))
                 root-coords
                 )
         )
       )
     ;; get handle coords in real world
     (setq real-handle-coords (send robot (send self :get :grasp-arm) :end-coords :copy-worldcoords))
     ;; get handle coords in model world
     (setq model-handle-coords (send (send self :rotate-grasp-point-by-grasp-arm (send car (send self :get :grasp-point))) :copy-worldcoords))
     ;; coordinate transformation
     (send robot :transform (send real-handle-coords :transformation model-handle-coords :world))
     )
   )
  (:overwrite-handle-angle ;; CAUTION: this function do not consider appropriateness between handle-coords and end-coords of grasp-arm
   (new-angle &key (use-real-robot-angle t) (use-controller t) (regrasp-handle nil) (time 5000))
   (let ((real-robot-angle-vector (send *ri* :state :potentio-vector))
         (limb-controller (read-from-string (format nil ":~A-controller" (string-downcase (send self :grasp-arm)))))
         )     
     (send car :handle-joint :joint-angle new-angle)
     (when use-real-robot-angle
       (send robot :angle-vector real-robot-angle-vector)
       (when (and real (not regrasp-handle))
         (if use-controller
             (send *ri* :angle-vector (send robot :angle-vector) time limb-controller)
           (send *ri* :angle-vector (send robot :angle-vector) time)
           )
         )
       )
     (if regrasp-handle
         (send self :grasp-handle)       
       )
     (send car :handle-joint :joint-angle new-angle) ;; return new handle angle
     )
   )
  (:fix-handle-overturn
   (overturn &key (use-real-robot-angle nil))   
   (let ((current-handle-angle (send car :handle-joint :joint-angle))
         new-angle
         )
     (setq new-angle (max (send (send car :handle-joint) :min-angle)
                          (min (send (send car :handle-joint) :max-angle)
                               (+ current-handle-angle (* overturn 360))
                               )
                          )
           )
     (send self :overwrite-handle-angle new-angle :use-real-robot-angle use-real-robot-angle)
     )
   )
  (:turn-handle
   (deg  &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
         (time 50) (relative t) (debug-view nil) (use-controller t) (stop 50)
         (rotation-axis 0) (resolution 5.0) (use-ik-seed t)
         )
   (let* ((steering-coords nil) (avs nil) (tms nil) (remain-angle)
          (target-degree (if relative deg (- deg (send car :handle-joint :joint-angle))))
          real-target-degree steering-ik-result controller
          )
     ;; define controller
     (if use-controller
         (setq controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
       (setq controller :default-controller)
       )
     ;; check interpolation
     (when (and real (send *ri* :interpolatingp controller))
       (warn "Skip target-angle ~A[deg] because interpolating now~%" deg)
       (return-from :turn-handle target-degree) ;; return remain-angle (= current target-degree)
       )
     
     ;; set default rotation-axis
     (when (equal rotation-axis 0)
       (case (send self :get :grasp-point)
         (:handle-crank (setq rotation-axis :z))
         (t (setq rotation-axis t))
         )
       )
     ;; turn angle limitation based on joint min/max
     (setq real-target-degree (max (- (send (send car :handle-joint) :min-angle) (send (send car :handle-joint) :joint-angle))
                                   (min (- (send (send car :handle-joint) :max-angle) (send (send car :handle-joint) :joint-angle))
                                        target-degree)
                                   )
           )
     (unless (= real-target-degree target-degree)
       (warn "target-angle is limited by handle min/max: ~A[deg] -> ~A[deg]~%" target-degree real-target-degree)
       )
     ;; calculate target-coords for steering
     (setq steering-coords
           (send self :get-steering-coords
                 real-target-degree
                 (send car :handle-joint) limb target-handle
                 :resolution resolution)
           )
     ;; solve steering ik
     (setq steering-ik-result
           (dolist (coord steering-coords)
             (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
                   (body-link (elt (send robot :links) 0)) (ik-ret)
                   (handle-link (find-if #'(lambda (h) (eq (send h :name) :handle-link)) (send (send *car* :vehicle-handle) :links)))
                   old-limb-angle-vector
                   )
               (when (and steering-arm-ik-seed use-ik-seed)
                 (setq old-limb-angle-vector (send robot limb :angle-vector))
                 (send robot limb :angle-vector steering-arm-ik-seed) ;; use same seed for steering ik
                 )
               (setq ik-ret
                     (send robot :inverse-kinematics coord
                           :move-target (send robot limb :end-coords)
                           :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                           :rotation-axis rotation-axis :debug-view debug-view :stop stop
                           :avoid-collision-null-gain 100
                           :avoid-collision-distance 80
                           :avoid-collision-joint-gain 0.1
                           :collision-avoidance-link-pair (concatenate cons
                                                                       (mapcar #'(lambda (x) (list x body-link)) check-links)
                                                                       (mapcar #'(lambda (x) (list x handle-link)) check-links))
                           :additional-check
                           #'(lambda ()
                               (not (some #'null (mapcar #'(lambda (x) (> (car (pqp-collision-distance x body-link)) distance-threshold))
                                                         check-links))))
                           )
                     )
               (if ik-ret
                   (progn (push (send robot :angle-vector) avs)
                          (if tms
                              (push time tms)
                            (push (* time 5) tms)
                            )
                          )
                 (progn (when (and steering-arm-ik-seed use-ik-seed)
                          (send robot limb :angle-vector old-limb-angle-vector) ;; resume angle-vector
                          )
                        (return nil) ;; ik failed
                        )
                 )
               )
             t ;; all ik succeed
             )
           )
     ;; send angle-vector-sequence to real robot
     (when real
       (send *ri* :angle-vector-sequence (reverse avs) (reverse tms) controller)
       ;; (send *ri* :wait-interpolation controller)
       (send *viewer* :draw-objects)
       )
     ;; calc remaining handle degree
     (setq remain-angle (abs real-target-degree))
     (dolist (av avs)
       (setq remain-angle (max (- remain-angle resolution) 0))
       )
     ;; (if (> target-degree 0) remain-angle (- remain-angle))
     (when (< real-target-degree 0)
       (setq remain-angle (- remain-angle))
       )
     (send car :handle-joint :joint-angle (- remain-angle) :relative t) ;; resume handle-joint by remain-angle
     remain-angle ;; return remaining handle angle
     )
   )
  (:turn-handle-once
   (deg  &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
         (time 100) (relative t) (debug-view nil) (use-controller t) (stop 100)
         (rotation-axis 0) (use-ik-seed t) (offset #f(0 0 0)) (offset-wrt :local)
         )
   (let ((target-degree (if relative deg (- deg (send car :handle-joint :joint-angle))))
         (check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
         (body-link (elt (send robot :links) 0)) (ik-ret)
         (handle-link (find-if #'(lambda (h) (eq (send h :name) :handle-link)) (send (send *car* :vehicle-handle) :links)))
         old-limb-angle-vector ik-result
         )
     ;; set default rotation-axis
     (when (equal rotation-axis 0)
       (case (send self :get :grasp-point)
         (:handle-crank (setq rotation-axis :z))
         (t (setq rotation-axis t))
         )
       )
     ;; turn angle limitation based on joint min/max
     (setq real-target-degree (max (- (send (send car :handle-joint) :min-angle) (send (send car :handle-joint) :joint-angle))
                                   (min (- (send (send car :handle-joint) :max-angle) (send (send car :handle-joint) :joint-angle))
                                        target-degree)
                                   )
           )
     (unless (= real-target-degree target-degree)
       (warn "target-angle is limited by handle min/max: ~A[deg] -> ~A[deg]~%" target-degree real-target-degree)
       )
     (when (and steering-arm-ik-seed use-ik-seed)
       (setq old-limb-angle-vector (send robot (send self :get :grasp-arm) :angle-vector))
       (send robot (send self :get :grasp-arm) :angle-vector steering-arm-ik-seed) ;; use same seed for steering ik
       )
     ;; update handle-angle
     (send car :handle-joint :joint-angle real-target-degree :relative t)

     ;; solve ik
     (setq ik-result
           (send robot :inverse-kinematics (send self :rotate-grasp-point-by-grasp-arm (send car target-handle))
                 :move-target (send robot limb :end-coords)
                 :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                 :rotation-axis rotation-axis :debug-view debug-view :stop stop
                 :avoid-collision-null-gain 100
                 :avoid-collision-distance 80
                 :avoid-collision-joint-gain 0.1
                 :collision-avoidance-link-pair (concatenate cons
                                                             (mapcar #'(lambda (x) (list x body-link)) check-links)
                                                             (mapcar #'(lambda (x) (list x handle-link)) check-links))
                 :additional-check
                 #'(lambda ()
                     (not (some #'null (mapcar #'(lambda (x) (> (car (pqp-collision-distance x body-link)) distance-threshold))
                                               check-links))))
                 )
           )

     (unless ik-result ;; ik failed
       (when (and steering-arm-ik-seed use-ik-seed)
         (send robot (send self :get :grasp-arm) :angle-vector old-limb-angle-vector) ;; resume angle-vector
         )
       (send car :handle-joint :joint-angle (- real-target-degree) :relative t) ;; resume joint angle
       (return-from :turn-handle-once real-target-degree) ;; return remain-angle (= target-angle)
       )
     
     ;; reflect grasp offset
     (setq ik-result (send robot limb :move-end-pos offset offset-wrt))

     (unless ik-result ;; ik failed
       (when (and steering-arm-ik-seed use-ik-seed)
         (send robot (send self :get :grasp-arm) :angle-vector old-limb-angle-vector) ;; resume angle-vector
         )
       (send car :handle-joint :joint-angle (- real-target-degree) :relative t) ;; resume joint angle
       (return-from :turn-handle-once real-target-degree) ;; return remain-angle (= target-angle)
       )

     ;; define controller
     (if use-controller
         (setq controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
       (setq controller :default-controller)
       )
     ;; send angle-vector-sequence to real robot
     (when real
       (send *ri* :angle-vector (send self :robot :angle-vector) time controller)
       ;; (send *ri* :wait-interpolation controller)
       (send *viewer* :draw-objects)
       )
     0.0 ;; return remain-angle (= zero)
     )
   )
  (:turn-handle-velocity
   (deg omega-deg &key (rate 10) (time-offset 0.1) (relative t) (use-controller t))
   (let* ((control-cycle (/ 1.0 rate)) ;; [sec]
          (target-deg (if relative (+ deg (send car :handle-joint :joint-angle)) deg)) ;; use absolute [deg]
          (current-deg (send car :handle-joint :joint-angle))
          (target-omega (/ (- target-deg current-deg) control-cycle)) ;; use [deg/sec]
          (max-omega (abs omega-deg))
          handle-deg remain-angle
         )
     (when (> (abs target-omega) max-omega)
       (setq target-omega (if (> target-omega 0) max-omega (- max-omega)))
       )
     (setq handle-deg (+ current-deg (* target-omega control-cycle)))
     (setq remain-angle 
           (send self :turn-handle-once handle-deg
                 :relative nil :use-controller use-controller
                 :time (* (+ control-cycle time-offset) 1000)) ;; interpolation time is [msec]
           )
     remain-angle
     )
   )
  
  (:get-steering-coords
   (relative-deg joint limb target-handle
                 &key (resolution 1.0))
   (let ((total-diff 0) (avs nil) (tms nil)
         (target-diff (abs relative-deg)) (ret-coords nil) (step (abs resolution))
         )
     ;; floor
     (while (< (+ total-diff step) target-diff)
       (send joint :joint-angle (if (> relative-deg 0) step (- step)) :relative t)
       (push (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) :copy-worldcoords) ret-coords)
       (setq total-diff (+ total-diff step))
       )
     ;; mod
     (send joint :joint-angle (- relative-deg (if (> relative-deg 0) total-diff (- total-diff))) :relative t)
     (push (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) :copy-worldcoords) ret-coords)
     (reverse ret-coords)
     )
   )

  ;; pedaling
  (:approach-pedal
   (limb target-pedal &key (tm 2000) (debug-view nil))
   ;; (send robot limb :inverse-kinematics (send (send car target-pedal) :copy-worldcoords) :rotation-axis t :debug-view debug-view)
   (let (approach-result)
     (setq approach-result
           (send robot :inverse-kinematics (send (send car target-pedal) :copy-worldcoords)
                 :move-target (send robot limb :end-coords)
                 :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :knee-p :child-link))
                 :debug-view debug-view :rotation-axis t :thre 80
                 )
           )
     (when (and approach-result real)
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       )
     approach-result
     )
   )
  (:approach-accel-pedal
   (&rest args &key (limb :rleg) (tm 2500) (debug-view nil)
          (reach-until-touch nil) (reach-until-touch-thre 10) (sync-controller nil))
   (let (approach-result)
     (when (and sync-controller (not (send *ri* :simulation-modep)))
       (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
       )
     (setq approach-result
           (send self :approach-pedal limb :accel-pedal :tm tm :debug-view debug-view)
           )
     (when (and real reach-until-touch approach-result (not (send *ri* :simulation-modep)))
       (send *ri* :reach-until-touch limb
             (send (send robot limb :end-coords :copy-worldcoords) :rotate-vector #f(0 0 -1))
             :coords-system :world :thre reach-until-touch-thre :time-step 1000 :max-dist 50 :dist-step 5
             :rotation-axis :z  :debug debug-view
             ) ;; todo: reach-until-touch uses copy-object
       (send *ri* :state) ;; update 
       (send robot limb :angle-vector (send *ri* :robot limb :angle-vector)) ;; reflect reach-until-touch result
       (send robot limb :move-end-pos #f(0 0 10) :local) ;; pedal should not be touched at first
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       )
     approach-result
     )
   )
  (:step-accel-pedal
   (command &key (time 500) (relative nil) (use-controller t) (no-wait t))
   (send robot :rleg-ankle-p :joint-angle command :relative relative)
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (unless no-wait (send *ri* :wait-interpolation))
     )
   )
  (:detatch-accel-pedal
   (&key (time 5000) (use-controller t) (no-wait t) (distance 50))
   (send robot :rleg :move-end-pos (float-vector 0 0 distance) :local)
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (unless no-wait (send *ri* :wait-interpolation))
     )
   )
  (:approach-brake-pedal
   (&rest args &key (limb :lleg) (tm 2500) (debug-view nil) 
          (reach-until-touch nil) (reach-until-touch-thre 10) (sync-controller nil))
   (when (and sync-controller (not (send *ri* :simulation-modep)))
       (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase limb))))
       )
   (let (approach-result)
     (setq approach-result
           (send self :approach-pedal limb :brake-pedal :tm tm :debug-view debug-view)
           )
     (when (and real reach-until-touch approach-result (not (send *ri* :simulation-modep)))
       (send *ri* :reach-until-touch limb
             (send (send robot limb :end-coords :copy-worldcoords) :rotate-vector #f(0 0 -1))
             :coords-system :world :thre reach-until-touch-thre
             :rotation-axis :z :time-step 1000 :max-dist 50 :dist-step 5 :debug debug-view
             ) ;; todo: reach-until-touch uses copy-object
       (send *ri* :state) ;; update 
       (send robot limb :angle-vector (send *ri* :robot limb :angle-vector)) ;; reflect reach-until-touch result
       (send robot limb :move-end-pos #f(0 0 10) :local) ;; pedal should not be touched at first
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       )
     approach-result
     )
   )
  (:step-brake-pedal
   (command &key (time 500) (relative nil) (use-controller t))
   (send robot :lleg-ankle-p :joint-angle command :relative relative)
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :lleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (send *ri* :wait-interpolation)
     )
   )

  ;; exchange grasp point
  (:exchange-handle
   (next-limb next-handle &key (tm 15000) (debug-view nil) (interactive nil))
   (let ((old-limb (send self :get :grasp-arm)) (old-handle (send self :get :grasp-point))
         (dist) (min-dist)
         )
     ;; decide next-handle
     (warn "exchange: ~A ~A~%" next-limb next-handle)
     ;; execute
     (when interactive
       (warn "release") (read-line) (send *viewer* :draw-objects)
       )
     (send self :approach-handle :limb old-limb :target-handle old-handle
           :tm tm :debug-view debug-view :sync-controller t :stop-impedance nil)
     (when interactive
       (warn "approach") (read-line) (send *viewer* :draw-objects)
       )
     (send self :approach-handle :limb next-limb :target-handle next-handle
           :tm tm :debug-view debug-view :stop-impedance nil)
     (when interactive
       (warn "grasp") (read-line) (send *viewer* :draw-objects)
       )
     (send self :grasp-handle :tm tm :limb next-limb :target-handle next-handle :debug-view debug-view)
     )
   )
  (:turn-handle-with-exchange
   (deg  &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
         (time 150) (relative t) (debug-view nil) (use-controller t) (interactive nil))
   (let ((handle-list (list :handle-right :handle-top :handle-left))
         remain-angle next-handle
         )
     (setq remain-angle (send self :turn-handle deg :limb limb :target-handle target-handle
                              :time time :relative relative :debug-view debug-view :use-controller use-controller))
     (while (not (= remain-angle 0))
       ;; decide next handle
       (let* ((old-handle-index (position (send self :get :grasp-point) handle-list))
              next-handle-index)         
         (setq next-handle-index (if (> remain-angle 0) (- old-handle-index 1) (+ old-handle-index 1)))
         (when (< next-handle-index 0)
           (setq next-handle-index (+ next-handle-index (length handle-list)))
           )
         (when (> next-handle-index (- (length handle-list) 1))
           (setq next-handle-index 0)
           )
         (warn "remain ~A ~%" remain-angle)
         (send self :exchange-handle limb (elt handle-list next-handle-index) :debug-view debug-view :interactive interactive)
         ;; turn remaining degree (remain-angle is relative)
         (setq remain-angle
               (send self :turn-handle remain-angle :limb (send self :get :grasp-arm) :target-handle (send self :get :grasp-point)
                     :time time :relative t :debug-view debug-view :use-controller use-controller))
         )
       )
     )
   )

  ;; holding frame
  (:approach-frame
   (&rest args &key (limb :larm) (target-handle :left-frame-center) (rotation-axis :z)
          (time 10000) (offset #f(-100 0 0)) (offset-wrt :local) (debug-view nil)
          (grasp nil) (reach-until-touch nil) (reach-until-touch-thre 10) (max-dist 50)
          )
   (send self :open-hand limb)
   (let ((check-links (subseq (send robot limb :links) 2 4))
         (car-base-link (send car :vehicle-base))
         ik-ret)
     (setq ik-ret
           (send robot limb :inverse-kinematics
                 (send (send (send car target-handle) :copy-worldcoords) :translate offset offset-wrt)
                 :debug-view debug-view :rotation-axis rotation-axis :stop 100
                 :avoid-collision-null-gain 100
                 :avoid-collision-joint-gain 1.0
                 :collision-avoidance-link-pair (mapcar #'(lambda (x) (list x car-base-link)) check-links)
                 )
           )
     (when (and real ik-ret)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       (send *ri* :wait-interpolation)
       (send *viewer* :draw-objects)
       (when (and reach-until-touch (not (send *ri* :simulation-modep)))
         (send *ri* :reach-until-touch limb
               (send (send robot limb :end-coords :copy-worldcoords) :rotate-vector #f(1 0 0))
               :coords-system :world :thre reach-until-touch-thre :max-dist max-dist :dist-step 5
               :rotation-axis :z :time-step 1000 :debug debug-view
               ) ;; todo: reach-until-touch uses copy-object
         (send *ri* :state) ;; update
         (send robot limb :angle-vector (send *ri* :robot limb :angle-vector)) ;; reflect reach-until-touch result
         )
       )
     (when grasp
       (send self :close-hand limb)
       )
     )
   )
  (:approach-fist
   (&rest args &key (limb :rarm) (target-handle :seat-left) (rotation-axis t) (offset #f(0 0 100))
          (offset-wrt :world) (debug-view nil) (time 10000) (reach-until-touch nil) (reach-until-touch-thre 10)
          (start-impedance nil) (set-ref-force nil) (max-dist 100))
   (send self :close-hand limb)
   (let (ik-ret)
     (setq ik-ret (send robot limb :inverse-kinematics (send (send (send car target-handle) :copy-worldcoords) :translate offset offset-wrt)
                        :debug-view debug-view :rotation-axis rotation-axis)
           )
     (when (and real ik-ret)
       (when start-impedance
         (send self :set-impedance-for-support limb)
         )
       (send *ri* :angle-vector (send robot :angle-vector) time)
       (send *ri* :wait-interpolation)
       (send *viewer* :draw-objects)
       (when (and reach-until-touch (not (send *ri* :simulation-modep)))
         (send *ri* :reach-until-touch limb
               (send (send robot limb :end-coords :copy-worldcoords) :rotate-vector #f(1 0 0))
               :coords-system :world :thre reach-until-touch-thre :max-dist max-dist :dist-step 5
               :rotation-axis :z :time-step 1000 :debug debug-view
               ) ;; todo: reach-until-touch uses copy-object
         (send *ri* :state) ;; update
         (send robot limb :angle-vector (send *ri* :robot limb :angle-vector)) ;; reflect reach-until-touch result
         )
       (when set-ref-force
         (send self :set-ref-force-for-support limb)
         )
       )
     )
   )

  ;; leg support
  (:approach-floor
   (&rest args &key (limb :lleg) (target-handle :floor) (time 10000) (debug-view nil)
          (offset #f(0 0 50)) (offset-wrt :local)
          (start-impedance t) (stop-ref-force nil) (set-ref-force nil) (reach-until-touch-thre 10)
          (reach-until-touch nil) (use-imu nil) (max-dist 50)
          &allow-other-keys)
   (when (and real stop-ref-force)
     (send *ri* :set-ref-force #f(0 0 0) 10000 limb)
     )
   (let* ((limb-endcoords (send robot limb :end-coords :copy-worldcoords))
          (floor-coords (send (send car target-handle) :copy-worldcoords))
          ik-result old-angle-vector floor-rot robot-ypr floor-ypr
          )
     (setq old-angle-vector (send robot :angle-vector)) ;; preserve original angle-vector
     ;; calculate floor-rotation
     (if use-imu
         (progn 
           (send *ri* :state) ;; update
           (setq robot-ypr (car (send (send *ri* :robot :worldcoords) :rpy-angle)))
           (setq floor-ypr (car (send floor-coords :rpy-angle)))
           (setq floor-rot (rpy-matrix (elt floor-ypr 0) (- (elt floor-ypr 1) (elt robot-ypr 1)) (- (elt floor-ypr 2) (elt robot-ypr 2))))
           )
       (setq floor-rot (send floor-coords :rot))
       )
     ;; solve ik
     (setq ik-result
           (send robot limb :inverse-kinematics
                 (make-coords :pos (send limb-endcoords :pos)
                              :rot floor-rot)
                 :rotation-axis :z
                 ) ;; fix x and y axis of legs to floor
           )
     (when ik-result
       (setq ik-result
             (send robot limb :move-end-pos
                   (v+ (float-vector 0 0 (- (elt (send floor-coords :pos) 2) (elt (send limb-endcoords :pos) 2)))
                       offset)
                   offset-wrt
                   ) ;; move to floor
             )
       )
     (if ik-result
         (when real 
           (send *ri* :angle-vector (send robot :angle-vector) time)
           (send *ri* :wait-interpolation)
           ;; impedance
           (when start-impedance
             (send self :set-impedance-for-support limb)
             )
           ;; reach-until-touch
           (when (and reach-until-touch (not (send *ri* :simulation-modep)))
             (send *ri* :reach-until-touch limb
                   (send (send robot limb :end-coords :copy-worldcoords) :rotate-vector #f(0 0 -1))
                   :coords-system :world :thre reach-until-touch-thre :max-dist max-dist :dist-step 5
                   :rotation-axis :z :time-step 1000 :debug debug-view
                   ) ;; todo: reach-until-touch uses copy-object
             (send *ri* :state) ;; update 
             (send robot limb :angle-vector (send *ri* :robot limb :angle-vector)) ;; reflect reach-until-touch result
             )
           ;; ref-force
           (when set-ref-force
             (send self :set-ref-force-for-support limb)
             )
           )
       (send robot :angle-vector old-angle-vector) ;; resume
       )
     ik-result
     )
   )

  ;; look around
  (:look-around
   (angle axis &key (relative nil) (time 5000) (use-controller nil) (rad nil))
   (when rad 
     (setq angle (rad2deg angle)) ;; robot angle-vector is [deg]
     )
   (send robot :head axis :joint-angle angle :relative relative)
   (when real
     (let ((controller :default-controller))
       (when use-controller
         (setq controller :head-controller)
         )
       (send *ri* :angle-vector (send robot :angle-vector) time controller)
       (send *viewer* :draw-objects)
       )
     )
   (send robot :head axis :joint-angle) ;; return current angle
   )

  ;; egress
  (:go-to-egress
   (&rest args)
   (warn ":go-to-egress should be impremented for each robots~%")
   )

  ;; utils
  (:sync-all-controller
   (&optional (sync-targets (list :larm :rarm :lleg :rleg)))
   (dolist (target sync-targets)
     (send *ri* :sync-controller (read-from-string (format nil ":~A-controller" (string-downcase target))))
     )
   )
  (:move-robot-position
   (position &key (relative t) (wrt :world))
   (send robot :translate position wrt)
   )
  (:rotate-grasp-point-by-grasp-arm ;; change coords of grasp-point for larm/rarm
   (handle-coords)
   (let (ret-coords axis)
     (case (send self :get :grasp-point)
       (:handle-crank (setq axis :z)) ;; handle-crank does not exists on handle
       (t (setq axis :x)) 
       )
     (case (send self :get :grasp-arm)
       (:larm (setq ret-coords
                    (send (send handle-coords :copy-worldcoords)
                          :rotate pi axis :local)
                    )
              )
       (:rarm (setq ret-coords handle-coords))
       (t (setq ret-coords handle-coords))
       )
     ret-coords
     )
   )  
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :M-p 200 :D-p 600 :K-p 1400)
   )
  (:set-impedance-for-support
   (limb)
   ;; (send *ri* :start-impedance limb :M-p 100 :D-p 600 :K-p 700)
   ;; (send *ri* :start-impedance limb :M-p 100 :D-p 2500 :K-p 3000)
   ;; (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 100 :D-r 2000 :K-r 3000)
   ;; (send *ri* :start-impedance :rarm :M-p 150 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 100 :D-r 1500 :K-r 2000)
   ;; (send *ri* :start-impedance :lleg :M-p 200 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 50 :D-r 250 :K-r 400)
   (cond
    ((member limb (list :larm :rarm :arms))
     (send *ri* :start-impedance limb :M-p 150 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 25 :D-r 150 :K-r 200)
     )
    ((member limb (list :lleg :rleg :legs))
     ;; (send *ri* :start-impedance limb :M-p 200 :D-p 3000 :K-p 5000 :moment-gain #(1 1 1) :M-r 50 :D-r 500 :K-r 400)
     (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 5000 :moment-gain #(1 1 1) :M-r 50 :D-r 500 :K-r 600)
     )
    (t
     (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 50 :D-r 250 :K-r 400) ;; default
     )
    )
   )
  (:set-ref-force-for-support
   (limb)
   (send *ri* :set-ref-force #f(0 0 80) 10000 limb)
   )
  (:draw-centroid-on
   (&key (mode nil) (use-real nil))
   (let (tmp-angle-vector centroid-pos centroid-coords)
     (if use-real
         (progn
           (setq tmp-angle-vector (send (send self :robot) :angle-vector)) ;; save original angle-vector
           (send (send self :robot) :angle-vector (send *ri* :state :potentio-vector))
           (setq centroid-pos (send (send self :robot) :centroid))
           (send (send self :robot) :angle-vector tmp-angle-vector) ;; resume
           )
       (setq centroid-pos (send (send self :robot) :centroid))
       )

     (case mode
       (:seat (setf (elt centroid-pos 2) (elt (send (send (send car :seat) :copy-worldcoords) :pos) 2)))
       (:foot (setf (elt centroid-pos 2) (elt (send (apply #'midcoords 0.5 (send robot :legs :end-coords :copy-worldcoords)) :pos) 2)))
       )
     (setq centroid-coords (make-coords :pos centroid-pos))
     (send centroid-coords :draw-on :flush t :size 300)
     )
   )
  (:update-steering-arm-ik-seed
   ()
   (let ((tmp-angle-vector (send (send self :robot) :angle-vector)) ;; save original angle-vector
         ik-ret
         )
     (setq ik-ret (send (send self :robot) (send self :get :grasp-arm) :inverse-kinematics
                        (send self :rotate-grasp-point-by-grasp-arm
                              (send (send (send car :vehicle-handle :steering-center) :copy-worldcoords) :translate #f(0 0 100) :local)
                              )
                        :rotation-axis nil
                        )
           )
     (if ik-ret
         (setq steering-arm-ik-seed (send (send self :robot) (send self :get :grasp-arm) :angle-vector))
       (warn "Failed to get steering-arm-ik-seed~%")
       )
     (send (send self :robot) :angle-vector tmp-angle-vector) ;; resume
     ik-ret
     )
   )
  (:calc-error-of-grasp-arm
   (&rest args &key (mode :angle-vector) (display-result nil))
   (let ((original-angle-vector (send (send self :robot) :angle-vector)) ;; save original angle-vector
         (original-arm-angle-vector (send (send self :robot) (send self :grasp-arm) :angle-vector))
         (original-end-coords (send (send self :robot) (send self :grasp-arm) :end-coords :copy-worldcoords))
         diff-angle-vector diff-coords retval
         )
     (send (send self :robot) :angle-vector (send *ri* :state :potentio-vector))
     (case mode
       (:angle-vector
        (setq diff-angle-vector (v- (send robot (send self :grasp-arm) :angle-vector) original-arm-angle-vector))
        (when display-result
          (warn "diff-angle-vector: ~A~%" diff-angle-vector)
          (warn "diff-angle-vector-norm: ~A~%" (norm diff-angle-vector))
          )
        (setq retval diff-angle-vector)
        )
       (:coords
         (setq diff-coords (send original-end-coords :transformation (send robot (send self :grasp-arm) :end-coords :copy-worldcoords) :local))
         (when display-result
           (warn "diff-coords: ~A~%" diff-coords)
           )
         (setq retval diff-coords)
         )
       )
     (send (send self :robot) :angle-vector original-angle-vector) ;; resume
     retval
     )
   )
  
  ;; accessor
  (:robot (&rest args) (forward-message-to robot args))
  (:car (&rest args) (forward-message-to car args))
  (:handle-angle () (send car :handle-joint :joint-angle))
  (:real
   (&optional (val 0) &key (use-real-robot-angle t))
   (unless (equal val 0)
     (setq real val)
     (when use-real-robot-angle
       (send robot :angle-vector (send *ri* :state :potentio-vector)) ;; sync model angle-vector to real
       )
     )
   real
   )
  (:model2real
   (&key (time 10000) (controller :default-controller) (wait-interpolation t) ((:real tmp-real) real))
   (let ((retval nil))
     (when tmp-real
       (setq retval (send *ri* :angle-vector (send self :robot :angle-vector) time controller))
       (when wait-interpolation
         (send *ri* :wait-interpolation controller)
         )
       (send *viewer* :draw-objects)
       )
     retval
     )
   )
  (:grasp-arm 
   (&optional val) 
   (when val
     (send self :put :grasp-arm val)
     (warn "modified grasp arm to ~A~%" (send self :get :grasp-arm))
     )
   (send self :get :grasp-arm)
   )
  (:grasp-point 
   (&optional val) 
   (when val
     (send self :put :grasp-point val)
     (warn "modified grasp point to ~A~%" (send self :get :grasp-point))
     )
   (send self :get :grasp-point)
   )
  )
