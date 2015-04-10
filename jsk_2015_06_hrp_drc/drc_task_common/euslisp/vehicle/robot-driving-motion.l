#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass robot-driving-motion
  :super propertied-object
  :slots (robot ;; robot model
          car ;; car model
          real ;; use real robot or not
          handle-angle-coords-table
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
         real tmp-real)
   (when real
     (send *ri* :set-interpolation-mode :linear) ;; avoid sudden change in joint angles when steering
     (unix::sleep 10) ;; wait activation
     )
   (send self :put :grasp-arm grasp-arm)
   (send self :put :grasp-point grasp-point)
   (send self :ride :init-pose t)
   (send self :update-handle-angle-coords-table)
   (objects (list robot car))   
   )  

  ;; initial methods
  (:drive-init-pose ;; initial pose for driving task
   (&key (tm 20000))
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
   (&key (limb (send self :get :grasp-arm)) (tm 10000) (drive-init-pose nil) (move-arm t))
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
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   (send self :update-handle-angle-coords-table)
   )
  (:drive-init-pose-support-by-leg
   (&key (tm 10000) (limb :lleg))
   ;; (send robot limb :angle-vector #f(0.0 0.0 -86.0 65.0 15.0 0.0))
   ;; (send robot limb :angle-vector #f(-2.592494e-06 3.642189e-05 -84.2409 62.8969 15.344 -3.722199e-05)) ;; move-end-pos #f(0 0 -10)
   (when real
     (send self :set-impedance-for-support limb)
     )
   (send robot limb :move-end-pos #f(0 0 -10) :world)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     (send *ri* :set-ref-force #f(0 0 80) 10000 :lleg)
     )
   )

  ;; wrapper function for hand
  (:open-hand
   (limb)
   (warn ":open-hand should be impremented for each robots~%")
   )
  (:close-hand
   (limb)
   (warn ":close-hand should be impremented for each robots~%")
   )

  ;; handling
  (:approach-handle
   (&key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point)) (sync-controller nil)
         (tm 15000) (offset #f(-150 0 0)) (rot-offset 0)
         (debug-view nil) (rotation-axis t) (start-impedance nil) (stop-impedance nil)
         (null-space #f(-0.06 -0.3 0.0 -0.18 0.16 0.0 0.0 0.0))
         )
   (when start-impedance
     (send self :set-impedance-for-handle limb)
     )
   (send self :open-hand limb)
   (when sync-controller
     (if (equal limb :larm)
         (send *ri* :sync-controller :larm-controller)
       (send *ri* :sync-controller :rarm-controller)
       )
     )
   (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
         (body-link (elt (send robot :links) 0))
         (handle-link (find-if #'(lambda (h) (eq (send h :name) :handle-link)) (send (send *car* :vehicle-handle) :links)))
         (offset-target-handle-coords
          (send (send (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle))
                            :copy-worldcoords)
                      :locate offset)
                :rotate rot-offset :z (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) ;; grasp-point is assumed to be a pole and axis :z is to be grasped (:x, :y cannot be rotated to grasp) 
                )
          )
         ik-result
         )
     ;; first, move roughly without collision detection
     (setq ik-result
           (send robot :inverse-kinematics offset-target-handle-coords
                 :move-target (send robot limb :end-coords)
                 :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                 :rotation-axis nil :debug-view debug-view
                 ;; :null-space null-space ;; avoid rarm to outside
                 ;; :null-space #f(30.0 -150.0 0.0 90.0 80.0 0.0 20.0 0.0) ;; avoid rarm to outside
                 )
           )
     (when ik-result ;; execute only if approach ik is succeed
       ;; second, re-solve inverse kinematics considering collision
       (setq ik-result
             (send robot :inverse-kinematics offset-target-handle-coords
                   :move-target (send robot limb :end-coords)
                   :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                   :rotation-axis rotation-axis :debug-view debug-view :stop 1000
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
  (:grasp-handle
   (&key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
         (debug nil) (tm 10000) (debug-view nil) (start-impedance t) (rotation-axis t) (adjust nil))
   ;; (send self :open-hand limb)
   (when (and real start-impedance)
     (send self :set-impedance-for-handle limb)
     )
   (let ((check-links (subseq (send robot limb :links) 3 6)) (distance-threshold 100)
         (body-link (elt (send robot :links) 0)) ik-result)
     (setq ik-result
           (send robot :inverse-kinematics (send (send self :rotate-grasp-point-by-grasp-arm (send car target-handle)) :copy-worldcoords)
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
       )
     ik-result ;; return result of ik (nil is failed)
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
  (:overwrite-handle-angle-forcely ;; CAUTION: this function do not consider appropriateness between handle-coords and end-coords of grasp-arm
   (new-angle)
   (let ((real-robot-angle-vector (send *ri* :state :potentio-vector))
         )
     (send robot :angle-vector real-robot-angle-vector)
     (send car :handle-joint :joint-angle new-angle)
     )
   )
  (:turn-handle
   (deg  &key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point))
         (time 100) (relative t) (debug-view nil) (use-controller t)
         (rotation-axis 0) (resolution 5.0)
         )
   (let* ((steering-coords nil) (avs nil) (tms nil) (remain-angle)
          (target-degree (if relative deg (- deg (send car :handle-joint :joint-angle))))
          real-target-degree steering-ik-result
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
                   )
               (setq ik-ret
                     (send robot :inverse-kinematics coord
                           :move-target (send robot limb :end-coords)
                           :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :root-link))
                           :rotation-axis rotation-axis :debug-view debug-view
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
                          (push time tms)
                          )
                 (return nil) ;; ik failed
                 )
               )
             t ;; all ik succeed
             )
           )
     ;; define controller
     (if use-controller
         (case limb
           (:larm (setq controller :larm-controller))
           (:rarm (setq controller :rarm-controller))
           (:lleg (setq controller :lleg-controller))
           (:rleg (setq controller :rleg-controller))
           (t (setq controller :default-controller))
           )
       (setq controller :default-controller)
       )
     ;; send angle-vector-sequence to real robot
     (when real
       (send *ri* :angle-vector-sequence (reverse avs) (reverse tms) controller)
       (send *viewer* :draw-objects)
       )
     ;; calc remaining handle degree
     (setq remain-angle (abs target-degree))
     (dolist (av avs)
       (setq remain-angle (max (- remain-angle resolution) 0))
       )
     ;; (if (> target-degree 0) remain-angle (- remain-angle))
     (when (< target-degree 0)
       (setq remain-angle (- remain-angle))
       )
     (send car :handle-joint :joint-angle (- remain-angle) :relative t) ;; resume handle-joint by remain-angle
     remain-angle ;; return remaining handle angle
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
   (limb target-pedal &key (tm 2500) (debug-view nil))
   ;; (send robot limb :inverse-kinematics (send (send car target-pedal) :copy-worldcoords) :rotation-axis t :debug-view debug-view)
   (send robot :inverse-kinematics (send (send car target-pedal) :copy-worldcoords)
            :move-target (send robot limb :end-coords)
            :link-list (send robot :link-list (send robot limb :end-coords :parent) (send robot limb :knee-p :child-link))
            :debug-view debug-view :rotation-axis t :thre 80
            )
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:approach-accel-pedal
   (&key (tm 3000) (debug-view nil))
   (send self :approach-pedal :rleg :accel-pedal :tm tm :debug-view debug-view)
   )
  (:step-accel-pedal
   (deg &key (time 1000) (relative t) (use-controller t) (no-wait))
   (send robot :rleg-ankle-p :joint-angle deg :relative relative)
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (unless no-wait (send *ri* :wait-interpolation))
     )
   )
  (:approach-brake-pedal
   (&key (tm 3000) (debug-view nil))
   (send self :approach-pedal :lleg :brake-pedal :tm tm :debug-view debug-view)
   )
  (:step-brake-pedal
   (deg &key (time 1000) (relative t) (use-controller t))
   (send robot :lleg-ankle-p :joint-angle deg :relative relative)
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
   (&key (limb :larm) (target-handle :left-frame-center) (time 10000) (offset #f(-150 50 0)) (debug-view nil))
   (send self :open-hand :rarm)
   (let (ik-ret)
     (setq ik-ret (send robot limb :inverse-kinematics (send (send (send car target-handle) :copy-worldcoords) :locate offset)
                        :debug-view debug-view :rotation-axis :z)
           )
     (when (and real ik-ret)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       (send *ri* :wait-interpolation)
       (send *viewer* :draw-objects)
       )
     )
   )
  (:grasp-frame
   (&key (limb :larm) (target-handle :left-frame-center) (time 5000) (debug-view nil))
   (let (ik-ret)
     (send robot limb :inverse-kinematics (send (send car target-handle) :copy-worldcoords)
           :debug-view debug-view :rotation-axis :z :stop 1000)
     (when (and real ik-ret)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       (send *ri* :wait-interpolation)
       (send *viewer* :draw-objects)
       )
     (send self :close-hand :rarm)
     )
   )
  
  ;; utils
  (:sync-all-controller
   ()
   (send *ri* :sync-controller :larm-controller)
   (send *ri* :sync-controller :rarm-controller)
   (send *ri* :sync-controller :lleg-controller)
   (send *ri* :sync-controller :rleg-controller)
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
  (:look-around
   (angle &key (real nil) (relative nil) (tm 5000))
   (send robot :head :neck-p :joint-angle angle :relative relative)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *viewer* :draw-objects)
     )
   )
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :M-p 200 :D-p 600 :K-p 1400)
   )
  (:set-impedance-for-support
   (limb)
   (send *ri* :start-impedance limb :M-p 100 :D-p 600 :K-p 700)
   )
  (:update-handle-angle-coords-table
   (&optional (step 5))
   (let ((handle-joint (send car :handle-joint))
         (handle-angle-table-max 360)
         (handle-angle 0.0)
         current-handle-angle
         )
     (setq handle-angle-coords-table nil)
     (setq current-handle-angle (send handle-joint :joint-angle))
     (send handle-joint :joint-angle 0.0)
     (while (<= handle-angle handle-angle-table-max)
       (send handle-joint :joint-angle handle-angle :relative nil)
       (push (cons handle-angle (send (send self :rotate-grasp-point-by-grasp-arm (send car (send self :grasp-point))) :copy-worldcoords)) handle-angle-coords-table)
       (setq handle-angle (+ handle-angle step))
       )
     (send handle-joint :joint-angle current-handle-angle) ;; resume handle angle
     )
   )
  (:estimate-current-handle-angle
   ()
   (let ((tmp-robot (copy-object (send self :robot)))         
         (dist 0)
         (min-dist nil)
         (estimated-angle nil)
         current-end-coords
         )
     (send tmp-robot :angle-vector (send *ri* :state :potentio-vector))
     (setq current-end-coords (send tmp-robot (send self :grasp-arm) :end-coords :copy-worldcoords))
     ;; search nearest handle-coods to end-coords
     (dolist (handle-angle-coords handle-angle-coords-table)
       (setq dist (norm (v- (send (cdr handle-angle-coords) :worldpos) (send current-end-coords :worldpos))))
       (unless min-dist
         (setq min-dist dist)
         )
       (when (<= dist min-dist)
         (setq min-dist dist)
         (setq estimated-angle (car handle-angle-coords))
         )
       )
     estimated-angle
     )
   )
  (:draw-centroid-on
   (&key (mode nil) (use-real nil))
   (let (tmp-robot centroid-pos centroid-coords)
     (if use-real
         (progn (setq tmp-robot (copy-object (send self :robot)))
                (send tmp-robot :angle-vector (send *ri* :state :potentio-vector))
                )
       (setq tmp-robot (send self :robot))
       )
     (setq centroid-pos (send tmp-robot :centroid)) 
     (case mode
       (:seat (setf (elt centroid-pos 2) (elt (send (send (send car :seat) :copy-worldcoords) :pos) 2)))
       (:foot (setf (elt centroid-pos 2) (elt (send (apply #'midcoords 0.5 (send robot :legs :end-coords :copy-worldcoords)) :pos) 2)))
       )
     (setq centroid-coords (make-coords :pos centroid-pos))
     (send centroid-coords :draw-on :flush t :size 300)
     )
   )
  
  ;; accessor
  (:robot (&rest args) (forward-message-to robot args))
  (:car (&rest args) (forward-message-to car args))
  (:handle-angle () (send car :handle-joint :joint-angle))
  (:real
   (&optional (val 0))
   (unless (equal val 0)
     (setq real val)
     (send robot :angle-vector (send *ri* :state :potentio-vector)) ;; sync model angle-vector to real
     )
   real
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
