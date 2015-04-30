#!/usr/bin/env roseus

(load "package://gazebo_drive_simulator/euslisp/drive-simulator-environment-ranger-xp900-size.l")

(defclass kenshiro-car
  :super drive-simulator
  :slots ()
  )
(defmethod kenshiro-car
  (:init
   (&rest args)
   (send-super* :init args)
   (setq seat nil
         handle nil
         pedal nil
         grasp-point nil)
   
   ;; make links links and assoc all links
   (setq handle (instance drive-sim-handle :init -150 400 -35 (/ 290 2) 400
                          ;; (arg-handle-x -60) (arg-handle-z 100) (arg-handle-angle -70) (arg-handle-ring-r 110) (arg-handle-axis-length 100)
                          ))
   (setq pedal (instance drive-sim-pedal-xp900 :init))
   (setq seat (instance drive-sim-playseat :init :white))
   (setq base (send self :make-kenshiro-car-base))

   ;; move simulator
   (let ((pedal-x-in-drive-sim (- (/ (send base :get :floor-length) 2) 100))
         (pedal-y-in-drive-sim -135) ;; (/ pedal-base-width 2)
         ;; (pedal-z-in-drive-sim (+ (send base :get :floor-height) -40)) ;; fix to seat-accel distination 420
         (pedal-z-in-drive-sim (send base :get :floor-height)) ;; fix to seat-accel distination 420
         (seat-x-in-drive-sim (- (/ (send base :get :floor-length) 2) 930))
         (seat-y-in-drive-sim 0)
         (seat-z-in-drive-sim 120)
         (handle-x-in-drive-sim (+ (/ (send base :get :floor-length) 2) -160))
         (handle-y-in-drive-sim 0)
         (handle-z-in-drive-sim 500) ;; (+ floor-thickness (/ pipe-thickness 2) 890) = handle center
         )
     (setq pedal-x-in-drive-sim pedal-x-in-drive-sim)
     (send pedal :translate (float-vector pedal-x-in-drive-sim pedal-y-in-drive-sim pedal-z-in-drive-sim) :world)
     (send seat :translate (float-vector seat-x-in-drive-sim seat-y-in-drive-sim (+ seat-z-in-drive-sim (send base :get :floor-height))) :world)
     (send handle :translate (float-vector handle-x-in-drive-sim handle-y-in-drive-sim (+ handle-z-in-drive-sim (send base :get :floor-height))) :world)
     )
   
   ;; assoc links
   ;; Root link should be associated with "self".
   (send self :assoc base)
   (send base :assoc handle)
   (send base :assoc pedal)
   (send base :assoc seat)
   (setq links (list base handle pedal seat))

   ;; make all joints
   ;; Before making joints, you should :assoc all links.

   ;; define slots for robot class
   ;; links and joint-list for cascaded-link.
   (send self :reset)
  
   ;; call :init-ending after defining links and joint-list and return "self"
   (send self :init-ending)
   self
   )

  (:make-kenshiro-car-base
   ()
   (let* ((floor-height 30) (floor-width 850) (floor-length 1300) (floor-thickness 25)
          (pipe-thickness 30) (pipe-height 750)
          (door-pole-offset 400)
          frame-list
          frame-pipe-list
          )
     ;; make frame-list ;;;;;;;;;;;;;;;;;
     (let* ((floor-corner-x (- (/ floor-length 2) (/ pipe-thickness 2)))
            (floor-corner-y (- (/ floor-width 2) (/ pipe-thickness 2)))            
            (frame-floor (make-cube floor-length floor-width floor-thickness))
            (frame-leg-origin (make-cube pipe-thickness pipe-thickness pipe-height))
            (frame-leg-front-left (copy-object frame-leg-origin))
            (frame-leg-front-right (copy-object frame-leg-origin))
            (frame-leg-back-left (copy-object frame-leg-origin))
            (frame-leg-back-right (copy-object frame-leg-origin))
            (frame-horizontal-origin (make-cube pipe-thickness floor-width pipe-thickness))
            (frame-horizontal-front-upper (copy-object frame-horizontal-origin))
            (frame-horizontal-front-lower (copy-object frame-horizontal-origin))
            (frame-horizontal-rear-upper (copy-object frame-horizontal-origin))
            (frame-horizontal-rear-lower (copy-object frame-horizontal-origin))
            (frame-vertical-origin (make-cube floor-length pipe-thickness pipe-thickness))
            (frame-vertical-left-upper (copy-object frame-vertical-origin))
            (frame-vertical-left-lower (copy-object frame-vertical-origin))
            (frame-vertical-right-upper (copy-object frame-vertical-origin))
            (frame-vertical-right-lower (copy-object frame-vertical-origin))
            )
       ;; make frame by pipe
       (push (send frame-leg-front-left :translate (float-vector floor-corner-x floor-corner-y (/ pipe-height 2))) frame-pipe-list)
       (push (send frame-leg-front-right :translate (float-vector floor-corner-x (- floor-corner-y) (/ pipe-height 2))) frame-pipe-list)
       (push (send frame-leg-back-left :translate (float-vector (- floor-corner-x) floor-corner-y (/ pipe-height 2))) frame-pipe-list)
       (push (send frame-leg-back-right :translate (float-vector (- floor-corner-x) (- floor-corner-y) (/ pipe-height 2))) frame-pipe-list)

       (push (send frame-horizontal-front-upper :translate (float-vector floor-corner-x 0 pipe-height)) frame-pipe-list)
       (push (send frame-horizontal-front-lower :translate (float-vector floor-corner-x 0 0)) frame-pipe-list)
       (push (send frame-horizontal-rear-upper :translate (float-vector (- floor-corner-x) 0 pipe-height)) frame-pipe-list)
       (push (send frame-horizontal-rear-lower :translate (float-vector (- floor-corner-x) 0 0)) frame-pipe-list)

       (push (send frame-vertical-left-upper :translate (float-vector 0 floor-corner-y pipe-height)) frame-pipe-list)
       (push (send frame-vertical-left-lower :translate (float-vector 0 floor-corner-y 0)) frame-pipe-list)
       ;; (push (send frame-vertical-right-upper :translate (float-vector 0 (- floor-corner-y) pipe-height)) frame-pipe-list) ;; currently right-upper frame does not exist
       (push (send frame-vertical-right-lower :translate (float-vector 0 (- floor-corner-y) 0)) frame-pipe-list)
       
       ;; make floor
       (send frame-floor :translate (float-vector 0 0 floor-height))
       (push frame-floor frame-list)
       (dolist (pipe frame-pipe-list)
         (send frame-floor :assoc pipe)
         )

       ;; make bodyset-link ;;;;;;;;;;;;;;;;;
       (setq base-link (instance bodyset-link :init (make-cascoords)
                                 :bodies (append frame-list frame-pipe-list)
                                 :name :base-link))
       (dolist (l (send base-link :bodies))
         (send l :set-color :gray60))

       ;; set parameters for base-link
       (send base-link :put :floor-length floor-length)
       (send base-link :put :floor-width floor-width)
       (send base-link :put :floor-height floor-height)
       (send base-link :put :pipe-thickness pipe-thickness)

       base-link
       )
     )
   )  
  )
