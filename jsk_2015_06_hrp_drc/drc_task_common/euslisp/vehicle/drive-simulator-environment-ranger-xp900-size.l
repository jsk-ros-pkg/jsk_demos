#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/drive-simulator-environment.l")
(load "package://drc_task_common/euslisp/vehicle/misumi-ranger-xp900-chair.l")

;; front: the position which have handle/pedal and display
;; left: y+ when front is defined as x+
(defclass drive-simulator-ranger-xp900-size
  :super drive-simulator
  :slots (cushion)
  )
(defmethod drive-simulator-ranger-xp900-size
  (:init
   (&rest args &key (handle-mode :center))
   (send-super* :init args)
   (setq seat nil
         handle nil
         pedal nil
         door nil
         cushion nil
         grasp-point nil)

   ;; make simulator links
   (setq base (send self :make-drive-simulator-base 570 340)) ;; fix to handle height from seat (340), ranger height (330) included cushion offset -> real distance from ground to seat is 340
   (setq handle (instance drive-sim-handle-xp900 :init))
   (setq pedal (instance drive-sim-pedal-xp900 :init))
   (setq seat (instance misumi-ranger-xp900-chair :init :mode :single))
   (setq door (send self :make-drive-simulator-door))

   ;; move simulator parts
   (let* ((cushion-height 10)
          (pedal-x-in-drive-sim (+ (- (- (/ (send base :get :floor-length) 2) 340) 25) 235)) ;; move -25 to adjust table-seat distination to 320 -> modify pedal pose and add +385 to fix accel position
          (pedal-y-in-drive-sim (+ (- 270 (/ (send base :get :floor-width) 2)) -40)) ;; fix to handle-pedal_center distination (- 130 (- 465 295)) (130: ranger_edge-handle, 295: ranger_center-pedal_center)
          (pedal-z-in-drive-sim (+ (send base :get :floor-height) (+ cushion-height))) ;; -40 is offset to fix seat-accel distination (420) 
          (seat-x-in-drive-sim (- (- (/ (send base :get :floor-length) 2) 875) 25)) ;; fix to seat-brake distination in real xp900 (475) and move -25 to adjust table-seat distination to 320
          (seat-y-in-drive-sim -65) ;; fix side-to-seat position(140)
          (seat-z-in-drive-sim 420) ;; real seat-z size is 490 but whole objects offseted +40 (seat relative size is fixed but floor relative size is not adjusted) -> seat modified and new ideal size is 430 (but real seat size is 420 because of machining error)
          (handle-x-in-drive-sim (+ (- (- (/ (send base :get :floor-length) 2) 250) 95) 240)) ;; 95: to fix real xp900 distance from seat to handle center (200)
          (handle-y-in-drive-sim (- 130 35)) ;; fix to ranger_edge-handle distination (- (/ 850 2) (- (/ 1520 2) 465)) (850: table_width, 1520: ranger_width, 465: ranger_center-handle) and fix 35 to side-handle_center distance
          (handle-z-in-drive-sim (send base :get :table-height))
          (door-x-in-drive-sim (/ (send base :get :floor-length) 2))
          (door-y-in-drive-sim (+ (/ (send base :get :floor-width) 2) (send base :get :pipe-thickness)))
          (door-z-in-drive-sim (send base :get :floor-height))
          )
     (setq cushion (send self :make-cushion 460 710 cushion-height))
     (send cushion :translate (float-vector seat-x-in-drive-sim seat-y-in-drive-sim (+ (send base :get :floor-height) seat-z-in-drive-sim (/ cushion-height 2))))
     ;; (setq pedal-x-in-drive-sim (- pedal-x-in-drive-sim 150)) ;; offset for staro
     (setq handle-y-in-drive-sim (+ handle-y-in-drive-sim 50)) ;; offset for staro
     (send pedal :translate (float-vector pedal-x-in-drive-sim pedal-y-in-drive-sim pedal-z-in-drive-sim) :world)
     (send seat :translate (float-vector seat-x-in-drive-sim seat-y-in-drive-sim (+ seat-z-in-drive-sim (send base :get :floor-height))) :world)
     (send handle :translate (float-vector handle-x-in-drive-sim handle-y-in-drive-sim (+ handle-z-in-drive-sim (send base :get :floor-height))) :world)
     (send door :translate (float-vector door-x-in-drive-sim door-y-in-drive-sim door-z-in-drive-sim) :world)
     )
   
   ;; assoc links
   ;; Root link should be associated with "self".
   (send self :assoc base)
   (send base :assoc handle)
   (send base :assoc pedal)
   (send base :assoc seat)
   (send base :assoc door)
   (send seat :assoc cushion)
   (setq links (list base handle pedal seat door cushion))
   
   ;; make all joints
   ;; Before making joints, you should :assoc all links.
   (setq door-joint (instance rotational-joint :init
                               :parent-link base :child-link door
                               :name :door-joint :axis :z
                               :min *-inf* :max *inf*))
   ;; define slots for robot class
   ;; links and joint-list for cascaded-link.
   (send self :reset)
  
   ;; call :init-ending after defining links and joint-list and return "self"
   (send self :init-ending)
   self
   )

  (:make-cushion
   (cushion-x cushion-y cushion-z)
   (let* ((cushion-body (make-cube cushion-x cushion-y cushion-z))
          cushion-link
          )
     (setq cushion-link (instance bodyset-link :init
                                  (make-cascoords :pos (float-vector 0 0 0))
                                  :bodies (list cushion-body)
                                  :name :cushion-link))
     (dolist (l (send cushion-link :bodies))
       (send l :set-color :gray30))
     cushion-link
     )
   )
  
  ;; methods to send message to joints ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:slide-joint (&rest args) nil)
  )

(defclass drive-sim-handle-xp900
  :super cascaded-link
  :slots (handle-obj handle rotation-joint handle-angle)
  )
(defmethod drive-sim-handle-xp900
  (:init
   (&optional (tmp-handle-angle 30) &rest args)
   (send-super* :init args)
   (setq handle nil)
   (setq handle-obj (instance drive-sim-handle :init))
   (setq handle-angle tmp-handle-angle)
   ;; 1. make links links and assoc all links
   (let ((base-link (send self :make-base-link)))     
     (send handle-obj :rotate (deg2rad handle-angle) :y :local)
     (send handle-obj :translate (float-vector (- (send base-link :get :base-length)) 0 (/ (send base-link :get :base-thickness) 2)))
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc base-link)
     (send base-link :assoc handle-obj)
     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (list base-link handle-obj))
     (setq joint-list (list handle-joint))
     (send self :reset)
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self
     )
   )
  (:make-base-link
   ()
   (let* ((base-length 400) ;; old 225
          (base-width 280) ;; old 280
          (base-thickness 9)
          (angle-length 80)
          base-link
          base-flame-list
          handle-base-flame-list)
     ;; make handle-flame-list ;;;;;;;;;;;;;;;;;
     (let* ((body-plate (make-cube base-length base-width base-thickness))
            (angle-org (make-prism (list (float-vector 0.0 0.0 0.0)
                                         (float-vector angle-length 0.0 0.0)
                                         (float-vector angle-length angle-length 0.0))
                                   2.0))
            angle-left angle-right)
       ;; move each body
       ;; set list and assoc
       (setq handle-base-flame-list (list body-plate))
       ;; move
       (send (car handle-base-flame-list) :rotate (deg2rad handle-angle) :y :world)
       (send (car handle-base-flame-list) :translate (float-vector (- (/ base-length 2)) 0 (/ base-thickness 2)) :local)
       ;; make bodyset-link ;;;;;;;;;;;;;;;;;
       (setq base-link (instance bodyset-link :init (make-cascoords)
                                 :bodies (append handle-base-flame-list)
                                 :name :base-link))
       (dolist (l (send base-link :bodies))
         (send l :set-color :gray30))
       (send base-link :put :base-length base-length)
       (send base-link :put :base-width base-width)
       (send base-link :put :base-thickness base-thickness)
       base-link
       )
     )
   )
  (:reset
   ()
   (let ((handle-joint-default-angle 0))
     (send self :handle-joint :joint-angle handle-joint-default-angle)
     )
   )
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:handle-top () (send handle-obj :handle-top))
  (:handle-left () (send handle-obj :handle-left))
  (:handle-right () (send handle-obj :handle-right))
  (:handle-left-upper () (send handle-obj :handle-left-upper))
  (:handle-right-upper () (send handle-obj :handle-right-upper))
  ;; methods to send message to joints ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:handle-joint (&rest args) (forward-message-to (find-if #'(lambda (h) (eq (send h :name) :handle-joint)) joint-list) args))
  )

;; origin of coords: lower right corner of pedal-base
;; exchange accel and brake position and rotated #f(0 pi/2 pi)
(defclass drive-sim-pedal-xp900
  :super drive-sim-pedal
  :slots ()
  )
(defmethod drive-sim-pedal-xp900
  (:init
   (&rest args)
   (send-super* :init args)
   (setq handle nil)
   ;; 1. make links links and assoc all links
   (let ((base (send self :make-base-link))
         (accel (send self :make-accel-pedal-link))
         (brake (send self :make-brake-pedal-link)))
     (let ((brake-pos-x 230)
           (brake-pos-y 60)
           (brake-pos-z 80)
           (accel-pos-x 230)
           (accel-pos-y 200)
           (accel-pos-z 98))
       (send accel :translate (float-vector accel-pos-x accel-pos-y accel-pos-z) :world)
       (send brake :translate (float-vector brake-pos-x brake-pos-y brake-pos-z) :world)
       )
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc base)
     (send base :assoc accel)
     (send base :assoc brake)
     ;; modify pose of pedal base 
     (send self :rotate (/ pi 2) :y :world)
     (send self :rotate pi :z :world)
     (send self :translate #f(0 270 320) :world) ;; #f(0 pedal-base-width pedal-base-length)
     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (setq accel-joint (instance rotational-joint :init
                                 :parent-link base :child-link accel
                                 :name :accel-joint :axis :y
                                 :min *-inf* :max *inf*))
     (setq brake-joint (instance rotational-joint :init
                                 :parent-link base :child-link brake
                                 :name :brake-joint :axis :y
                                 :min *-inf* :max *inf*))
     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (list base accel brake))
     (setq joint-list (list accel-joint brake-joint))
     (send self :reset)
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self)
   )
  ;; reset joint angle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:reset
   (&optional (pedal nil))
   (let ((accel-joint-default-angle -45)
         (brake-joint-default-angle -55)
         (reset-pedal-list (list :accel :brake))
         )
       (case pedal
         (:accel (setq reset-pedal-list (list :accel)))
         (:brake (setq reset-pedal-list (list :brake)))
         )
       (when (member :accel reset-pedal-list)
         (send self :accel-joint :joint-angle accel-joint-default-angle)
         )
       (when (member :brake reset-pedal-list)
         (send self :brake-joint :joint-angle brake-joint-default-angle)
         )
       )
   )
  ;; methods to define playseat brake-pedal-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:make-brake-pedal-link
   ()
   (let (brake-pedal-list
         brake-pedal-handle
         brake-pedal-link)
     ;; make brake-pedal-list ;;;;;;;;;;;;;;;;;
     (let* ((brake-pedal-length 112)
            (brake-pedal-height 50)
            (brake-pedal-width 61)
            (brake-pedal-thickness 4)
            (brake-pedal (make-cube brake-pedal-length brake-pedal-width brake-pedal-thickness)))
       ;; move each body
       ;; define handle
       (setq brake-pedal-handle (make-cascoords :coords (send (send brake-pedal :copy-worldcoords) :rotate pi :z :local)
                                                :name :brake-pedal))
       ;; set list and assoc
       (setq brake-pedal-list (list brake-pedal))
       (send brake-pedal :assoc brake-pedal-handle)
       (push brake-pedal-handle handle)
       (dolist (b (cdr brake-pedal-list)) (send (car brake-pedal-list) :assoc b))
       ;; move
       (send (car brake-pedal-list) :translate (float-vector (- (/ brake-pedal-length 2) brake-pedal-height) 0 (/ brake-pedal-thickness 2)))
       ;; make bodyset-link ;;;;;;;;;;;;;;;;;
       (setq brake-pedal-link (instance bodyset-link :init (make-cascoords :pos (float-vector (- (/ brake-pedal-length 2)) 0 0))
                                        :bodies (append brake-pedal-list nil)
                                        :name :brake-pedal-link))
       (dolist (l (send brake-pedal-link :bodies))
         (send l :set-color :gray30))
       brake-pedal-link
       )
     )
   )
  ;; methods to define playseat accel-pedal-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:make-accel-pedal-link
   ()
   (let (accel-pedal-list
         accel-pedal-handle
         accel-pedal-link)
     ;; make accel-pedal-list ;;;;;;;;;;;;;;;;;
     (let* ((accel-pedal-length 91)
            (accel-pedal-width 59)
            (accel-pedal-thickness 4)
            (accel-pedal-height 50)
            (accel-pedal (make-cube accel-pedal-length accel-pedal-width accel-pedal-thickness)))
       ;; move each body
       ;; define handle
       (setq accel-pedal-handle (make-cascoords :coords (send (send accel-pedal :copy-worldcoords) :rotate pi :z :local)
                                                :name :accel-pedal))
       ;; set list and assoc
       (setq accel-pedal-list (list accel-pedal))
       (send accel-pedal :assoc accel-pedal-handle)
       (push accel-pedal-handle handle)
       (dolist (b (cdr accel-pedal-list)) (send (car accel-pedal-list) :assoc b))
       (setq accel-pedal-list (list accel-pedal))
       (dolist (b (cdr accel-pedal-list)) (send (car accel-pedal-list) :assoc b))
       ;; move
       (send (car accel-pedal-list) :translate (float-vector (- (/ accel-pedal-length 2) accel-pedal-height) 0 (/ accel-pedal-thickness 2)))
       ;; make bodyset-link ;;;;;;;;;;;;;;;;;
       (setq accel-pedal-link (instance bodyset-link :init (make-cascoords :pos (float-vector (- (/ accel-pedal-length 2)) 0 0))
                                        :bodies (append accel-pedal-list nil)
                                        :name :accel-pedal-link))
       (dolist (l (send accel-pedal-link :bodies))
         (send l :set-color :gray30))
       accel-pedal-link
       )
     )
   )
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:accel-pedal () (find-if #'(lambda (h) (eq (send h :name) :accel-pedal)) handle))
  (:brake-pedal () (find-if #'(lambda (h) (eq (send h :name) :brake-pedal)) handle))
  ;; methods to send message to joitns ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:accel-joint (&rest args) (forward-message-to accel-joint args))
  (:brake-joint (&rest args) (forward-message-to brake-joint args))
  )
