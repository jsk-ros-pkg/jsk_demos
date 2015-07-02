;; origin of coords: center of front base frame
(defclass drive-sim-playseat
  :super cascaded-link
  :slots (handle reclining-joint slide-joint)
  )
(defmethod drive-sim-playseat
  (:init
   (&optional (color :red) &rest args)
   (send-super* :init args)
   (setq seat nil)
   ;; 1. make links links and assoc all links
   (let ((base (send self :make-base-link))
         (seat (send self :make-seat-link color))
         (back (send self :make-back-link color)))
     (let (;; (seat-pos-x -350) (seat-pos-z 280)
           ;; (back-pos-x -600) (back-pos-z 300)
           (seat-pos-x -165) (seat-pos-z 280)
           (back-pos-x -315) (back-pos-z 300)
           )
       (send seat :translate (float-vector seat-pos-x 0 seat-pos-z) :world)
       (send back :translate (float-vector back-pos-x 0 back-pos-z) :world)
       ;; 2. assoc links
       ;;    Root link should be associated with "self".
       (send self :assoc base)
       (send base :assoc seat)
       (send seat :assoc back)
       ;; 3. make all joints
       ;;    Before making joints, you should :assoc all links.
       (setq slide-joint (instance linear-joint :init
                                   :parent-link base :child-link seat
                                   :name :slide-joint :axis :x
                                   :min *-inf* :max *inf*))
       (setq reclining-joint (instance rotational-joint :init
                                       :parent-link seat :child-link back
                                       :name :reclining-joint :axis :y
                                       :min *-inf* :max *inf*))
       ;; 4. define slots for robot class
       ;;    links and joint-list for cascaded-link.
       (setq links (list base seat back))
       (setq joint-list (list slide-joint reclining-joint))
       (send self :reset)
       ;; 5. call :init-ending after defining links and joint-list and return "self"
       (send self :init-ending)
       self
       )
     )
   )
    ;; reset joint angle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:reset
   ()
   (let ((slide-joint-default-angle 190) ;; front limit of slide joint
         ;; (slide-joint-default-angle 50)
         (reclining-joint-default-angle -20))
     (send self :reclining-joint :joint-angle reclining-joint-default-angle)
     (send self :slide-joint :joint-angle slide-joint-default-angle)
     )  
   )
  ;; methods to define playset base-links ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:make-base-link
   ()
   (let* ((pipe-thickness 30)
          (base-width 450)
          (base-length 1250)
          (plate-thickness 2)
          base-link
          base-flame-list
          pedal-plate-list
          handle-flame-list)
     ;; make base-flame-list ;;;;;;;;;;;;;;;;;
     (let* ((middle-pipe-pos-x (- 730 (/ base-length 2)))
            (seat-base-length 320)
            (seat-base-height 140)
            (seat-base-width 40)
            ;; (seat-base-pos-x (- (+ 120 seat-base-length) (/ base-length 2)))
            (seat-base-pos-x 0)
            (seat-base-pos-y (- 200 (/ seat-base-width 2)))
            (tmp1 (make-cube seat-base-length seat-base-width seat-base-height))
            (tmp2 (make-cube (+ seat-base-length 100) seat-base-width (- seat-base-height (* plate-thickness 2))))
            seat-base-left
            seat-base-right
            seat-base-front
            seat-base-back
            )
       ;; move each body
       (send tmp2 :translate (float-vector 0 (- plate-thickness) 0))
       (setq seat-base-left (body- tmp1 tmp2))
       (send seat-base-left :translate (float-vector (- seat-base-pos-x (/ seat-base-length 2)) seat-base-pos-y (+ pipe-thickness (/ seat-base-height 2))) :world)
       (setq seat-base-right (body- tmp1 tmp2))
       (send seat-base-right :rotate (deg2rad 180) :z)
       (send seat-base-right :translate (float-vector (- seat-base-pos-x (/ seat-base-length 2)) (- seat-base-pos-y) (+ pipe-thickness (/ seat-base-height 2))) :world)
       (setq seat-base-front (make-cube pipe-thickness base-width pipe-thickness))
       (send seat-base-front :translate (float-vector (- seat-base-pos-x (/ pipe-thickness 2)) 0 (/ pipe-thickness 2)))
       (setq seat-base-back (make-cube pipe-thickness base-width pipe-thickness))
       (send seat-base-back :translate (float-vector (- seat-base-pos-x (- seat-base-length (/ pipe-thickness 2))) 0 (/ pipe-thickness 2)))
       ;; set list and assoc
       (setq base-flame-list (list seat-base-left seat-base-right seat-base-front seat-base-back))
       (dolist (b (cdr base-flame-list)) (send (car base-flame-list) :assoc b))
       )
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq base-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append base-flame-list)
                               :name :base-link))
     (dolist (l (send base-link :bodies))
       (send l :set-color :gray30))
     base-link
     )
   )
  ;; methods to define playset seat-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:make-seat-link
   (&optional (color :red))
   (let ((seat-width 335)
         (seat-length 500)
         (seat-thickness 100)
         (seat-angle -5)
         (seat-side-thickness 80)
         (seat-side-depth 75)
         (seat-side-angle -5)
         seat-list
         seat-link
         )
     ;; make seat-list ;;;;;;;;;;;;;;;;;
     (let* ((seat-middle (make-cube seat-length seat-width seat-thickness))
            (seat-left (make-cube seat-length seat-side-thickness (+ seat-thickness seat-side-depth)))
            (seat-right (make-cube seat-length seat-side-thickness (+ seat-thickness seat-side-depth))))
       ;; move each body
       (send seat-left :translate (float-vector 0 (/ (+ seat-width  seat-side-thickness) 2) (/ seat-side-depth 2)))
       (send seat-left :rotate (deg2rad seat-side-angle) :x)
       (send seat-right :translate (float-vector 0 (- (/ (+ seat-width  seat-side-thickness) 2)) (/ seat-side-depth 2)))
       (send seat-right :rotate (- (deg2rad seat-side-angle)) :x)
       ;; define handle
       (setq seat-handle (make-cascoords :coords (send (send seat-middle :copy-worldcoords)
                                                       :translate (float-vector 25 0 (+ (/ seat-thickness 2) 75)) :world) :name :seat))
       (setq seat-handle-left (make-cascoords :coords (send (send (send seat-left :copy-worldcoords) :rotate (deg2rad 90) :x)
                                                            :translate (float-vector 175 0 (+ (/ seat-thickness 2) 75)) :world) :name :seat-left))
       (setq seat-handle-right (make-cascoords :coords (send (send (send seat-right :copy-worldcoords) :rotate (deg2rad -90) :x)
                                                            :translate (float-vector 175 0 (+ (/ seat-thickness 2) 75)) :world) :name :seat-right))

       (send seat-middle :assoc seat-handle)
       (push seat-handle handle)
       (send seat-left :assoc seat-handle-left)
       (push seat-handle-left handle)
       (send seat-right :assoc seat-handle-right)
       (push seat-handle-right handle)

       ;; set list and assoc
       (setq seat-list (list seat-middle seat-left seat-right))
       (dolist (b (cdr seat-list)) (send (car seat-list) :assoc b))
       ;; move
       (send (car seat-list) :rotate (deg2rad seat-angle) :y)
       )
     ;; make bodyset-link
     (setq seat-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append seat-list nil)
                               :name :seat-link))
     (dolist (l (send seat-link :bodies))
       (send l :set-color color))
     seat-link
     )
   )
  (:make-back-link
   (&optional (color :red))
   (let ((back-width 325)
         (back-height 620)
         (back-thickness 30)
         (back-side-thickness 75)
         (back-side-depth 110)
         (back-side-angle 20)
         (back-top-height 240)
         (back-top-width 180)
         (back-top-angle 5)
         back-left-handle
         back-right-handle
         back-list
         back-link)
     ;; make back-list ;;;;;;;;;;;;;;;;;
     (let* ((back-middle (make-cube back-thickness back-width back-height))
            (back-left (make-cube (+ back-thickness back-side-depth) back-side-thickness back-height))
            (back-right (make-cube (+ back-thickness back-side-depth) back-side-thickness back-height))
            (back-top (make-prism (list (float-vector 0 (/ back-width 2) 0) (float-vector 0 (/ back-top-width 2) back-top-height)
                                        (float-vector 0 (- (/ back-top-width 2)) back-top-height) (float-vector 0 (- (/ back-width 2)) 0))
                                  (float-vector (- back-thickness) 0 0))))
       ;; define handle
       (setq back-left-handle (make-cascoords :coords (send (send (send back-left :copy-worldcoords)
                                                                  :translate (float-vector (/ (+ back-thickness back-side-depth) 2)  0 (* back-height 0.25)) :world)
                                                            :rotate (deg2rad 180) :y) :name :back-left))
       (setq back-right-handle (make-cascoords :coords (send (send (send back-right :copy-worldcoords)
                                                                   :translate (float-vector (/ (+ back-thickness back-side-depth) 2)  0 (* back-height 0.25)) :world)
                                                             :rotate (deg2rad 180) :y) :name :back-right))
       (send back-left :assoc back-left-handle)
       (push back-left-handle handle)
       (send back-right :assoc back-right-handle)
       (push back-right-handle handle)
       ;; move each body
       (send back-left :translate (float-vector (/ back-side-depth 2) (/ (+ back-width back-side-thickness) 2) 0))
       (send back-left :rotate (deg2rad back-side-angle) :z)
       (send back-right :translate (float-vector (/ back-side-depth 2) (- (/ (+ back-width back-side-thickness) 2)) 0))
       (send back-right :rotate (- (deg2rad back-side-angle)) :z)
       (send back-top :rotate (deg2rad back-top-angle) :y)
       (send back-top :translate (float-vector (/ back-thickness 2) 0 (- (/ back-height 2) 10)) :world)
       ;; set list and assoc
       (setq back-list (list back-middle back-left back-right back-top))
       (dolist (b (cdr back-list)) (send (car back-list) :assoc b))
       ;; move
       (send (car back-list) :translate (float-vector 0 0 (/ back-height 2)))
       )
     ;; make bodyset-link
     (setq back-link (instance bodyset-link :init (make-cascoords)
                               :bodies back-list
                               :name :back-link))
     (dolist (l (send back-link :bodies))
       (send l :set-color color))
     back-link
     )
   )
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:seat () (find-if #'(lambda (h) (eq (send h :name) :seat)) handle))
  (:seat-left () (find-if #'(lambda (h) (eq (send h :name) :seat-left)) handle))
  (:seat-right () (find-if #'(lambda (h) (eq (send h :name) :seat-right)) handle))
  (:back-left () (find-if #'(lambda (h) (eq (send h :name) :back-left)) handle))
  (:back-right () (find-if #'(lambda (h) (eq (send h :name) :back-right)) handle)) 
  ;; methods to send message to joitns ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:slide-joint (&rest args) (forward-message-to slide-joint args))
  (:reclining-joint (&rest args) (forward-message-to reclining-joint args))
  )

;; origin of coords: front center of handle base
(defclass drive-sim-handle
  :super cascaded-link
  :slots (handle steering-center steering-radius reclining-joint slide-joint)
  )
(defmethod drive-sim-handle
  (:init
   (&optional (arg-handle-x -60) (arg-handle-z 100) (arg-handle-angle -70) (arg-handle-ring-r 110) (arg-handle-axis-length 100)
              &rest args)
   (send-super* :init args)
   (setq handle nil)
   ;; 1. make links and assoc all links
   (let ((base-link (send self :make-base-link))
         (handle-link (send self :make-handle-link arg-handle-ring-r arg-handle-axis-length))
         )
     (let ((handle-pos-x arg-handle-x)
           (handle-pos-z arg-handle-z)
           (handle-angle arg-handle-angle)
           )
       (send handle-link :translate (float-vector handle-pos-x 0 handle-pos-z) :world)
       (send handle-link :rotate (deg2rad handle-angle) :y)
       (send (send self :handle-top) :rotate (deg2rad (- 270 handle-angle)) :y) ;; make handle coords horizontal to ground (must rotate 90deg around x when rarm is used)
       ;; fix handle coords to robot hand coords (based on left hand coords)
       (dolist (hndl (mapcar #'(lambda (h) (send h :name)) handle))
         (send (send self hndl) :rotate (deg2rad 90) :x)
        )
       ;; (send (send self :handle-top) :rotate (deg2rad -90) :z)
       )
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc base-link)
     (send base-link :assoc handle-link)
     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (setq handle-joint (instance rotational-joint :init
                                  :parent-link base-link :child-link handle-link
                                  :name :handle-joint :axis :z
                                  :min -540.0 :max 540.0))
     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (list base-link handle-link))
     (setq joint-list (list handle-joint))
     (send self :reset)
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self)
   )
  (:make-handle-link
   (&optional (arg-handle-ring-r 110) (arg-handle-axis-length 100))
   (let ((handle-pipe-r (/ 35 2))
         (handle-axis-length arg-handle-axis-length) ;; defualt: 100
         (handle-axis-r (/ 52 2))
         (handle-bar-thickness 26)
         (handle-bar-hor-height 59)
         (handle-bar-ver-width 50)
         (handle-upper-handle-angle 35)
         handle-list
         handle-left-handle
         handle-right-handle
         handle-left-upper-handle
         handle-right-upper-handle
         handle-link)
     (setq steering-radius arg-handle-ring-r) ;; set handle radius 
     ;; make handle-list ;;;;;;;;;;;;;;;;;
     (let* ((handle-ring (make-ring steering-radius handle-pipe-r))
            (handle-axis (make-cylinder handle-axis-r handle-axis-length))
            (handle-bar-hor (make-cube handle-bar-hor-height (* steering-radius 2) handle-bar-thickness))
            (handle-bar-ver (make-cube steering-radius handle-bar-ver-width handle-bar-thickness))
            (handle-grasp-offset (+ handle-pipe-r 0))
            (handle-rotation-r (+ steering-radius handle-grasp-offset))
            (lr-upper-handle-angle (deg2rad 45))
            (lr-handle-angle (deg2rad 135))
            )
       ;; move each body
       (send handle-axis :translate (float-vector 0 0 (- handle-axis-length)))
       (send handle-bar-ver :translate (float-vector (- (/ steering-radius 2)) 0 0))
       ;; define handle
       (setq handle-center-handle
             (make-cascoords :coords
                             (send (send handle-ring :copy-worldcoords) :rotate (deg2rad 90) :y)
                             :name :handle-center))
       (setq handle-top-handle
             (make-cascoords :coords (send (send (send handle-ring :copy-worldcoords)
                                                 :rotate (deg2rad 90) :y)
                                           :translate (float-vector 0 0 handle-rotation-r))
                             :name :handle-top))
       (setq handle-left-upper-handle 
             (make-cascoords :coords (send (send (send handle-top-handle :copy-worldcoords)
                                                 :rotate (- lr-upper-handle-angle) :x)
                                           :translate (float-vector 0 (* handle-rotation-r (sin lr-upper-handle-angle)) (* handle-rotation-r (- 1.0 (cos (- lr-upper-handle-angle))))))
                             :name :handle-left-upper))
       (setq handle-right-upper-handle 
             (make-cascoords :coords (send (send (send handle-top-handle :copy-worldcoords)
                                                 :rotate lr-upper-handle-angle :x)
                                           :translate (float-vector 0 (* handle-rotation-r (sin (- lr-upper-handle-angle))) (* handle-rotation-r (- 1.0 (cos lr-upper-handle-angle)))))
                             :name :handle-right-upper))
       (setq handle-left-handle 
             (make-cascoords :coords (send (send (send handle-top-handle :copy-worldcoords)
                                                 :rotate (- lr-handle-angle) :x)
                                           :translate (float-vector 0 (* handle-rotation-r (sin lr-handle-angle)) (* handle-rotation-r (- 1.0 (cos (- lr-handle-angle))))))
                             :name :handle-left))
       (setq handle-right-handle 
             (make-cascoords :coords (send (send (send handle-top-handle :copy-worldcoords)
                                                 :rotate lr-handle-angle :x)
                                           :translate (float-vector 0 (* handle-rotation-r (sin (- lr-handle-angle))) (* handle-rotation-r (- 1.0 (cos lr-handle-angle)))))
                             :name :handle-right))

       (setq handle-top-side-handle
             (make-cascoords :coords (send (send handle-top-handle :copy-worldcoords) :rotate (/ pi 2) :y :local)
                             :name :handle-top-side))
       (setq handle-left-upper-side-handle
             (make-cascoords :coords (send (send handle-left-upper-handle :copy-worldcoords) :rotate (/ pi 2) :y :local)
                             :name :handle-left-upper-side))
       (setq handle-right-upper-side-handle
             (make-cascoords :coords (send (send handle-right-upper-handle :copy-worldcoords) :rotate (/ pi 2) :y :local)
                             :name :handle-right-upper-side))
       (setq handle-left-side-handle
             (make-cascoords :coords (send (send handle-left-handle :copy-worldcoords) :rotate (/ pi 2) :y :local)
                             :name :handle-left-side))
       (setq handle-right-side-handle
             (make-cascoords :coords (send (send handle-right-handle :copy-worldcoords) :rotate (/ pi 2) :y :local)
                             :name :handle-right-side))
       (setq handle-crank-handle
             (make-cascoords :coords (send (send (send (send handle-ring :copy-worldcoords)                                                       
                                                       :rotate (deg2rad 90) :z)
                                                 :rotate (deg2rad -90) :x)
                                           :translate (float-vector 0 -130 105) :local)
                             :name :handle-crank))
       
       ;; old definitions
       ;; (setq handle-left-upper-handle 
       ;;       (make-cascoords :coords (send (send (send (send handle-ring :copy-worldcoords)
       ;;                                                 :rotate (deg2rad 90) :y)
       ;;                                           :translate (float-vector 0 (* (+ steering-radius handle-grasp-offset) (cos (deg2rad handle-upper-handle-angle)))
       ;;                                                                    (* (+ steering-radius handle-grasp-offset) (sin (deg2rad handle-upper-handle-angle)))))
       ;;                                     :rotate (deg2rad handle-upper-handle-angle) :x) :name :handle-left-upper))
       ;; (setq handle-right-upper-handle 
       ;;       (make-cascoords :coords (send (send (send (send handle-ring :copy-worldcoords)
       ;;                                                 :rotate (deg2rad 90) :y)
       ;;                                           :translate (float-vector 0 (- (* (+ steering-radius handle-grasp-offset) (cos (deg2rad handle-upper-handle-angle))))
       ;;                                                                    (* (+ steering-radius handle-grasp-offset) (sin (deg2rad handle-upper-handle-angle)))))
       ;;                                     :rotate (- (deg2rad handle-upper-handle-angle)) :x) :name :handle-right-upper))
       (send handle-ring :assoc handle-center-handle)
       (send handle-ring :assoc handle-top-handle)
       (send handle-ring :assoc handle-left-handle)
       (send handle-ring :assoc handle-right-handle)
       (send handle-ring :assoc handle-left-upper-handle)
       (send handle-ring :assoc handle-right-upper-handle)
       (send handle-ring :assoc handle-top-side-handle)
       (send handle-ring :assoc handle-left-side-handle)
       (send handle-ring :assoc handle-right-side-handle)
       (send handle-ring :assoc handle-left-upper-side-handle)
       (send handle-ring :assoc handle-right-upper-side-handle)
       (send handle-ring :assoc handle-crank-handle)
       (push handle-center-handle handle)
       (push handle-top-handle handle)
       (push handle-left-handle handle)
       (push handle-right-handle handle)
       (push handle-left-upper-handle handle)
       (push handle-right-upper-handle handle)
       (push handle-top-side-handle handle)       
       (push handle-left-side-handle handle)
       (push handle-right-side-handle handle)
       (push handle-left-upper-side-handle handle)
       (push handle-right-upper-side-handle handle)
       (push handle-crank-handle handle)
       ;; set list and assoc
       (setq handle-list (list handle-ring handle-axis handle-bar-hor handle-bar-ver))
       (dolist (b (cdr handle-list)) (send (car handle-list) :assoc b))
       ;; move
       )
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq handle-link (instance bodyset-link :init (make-cascoords)
				 :bodies (append handle-list nil)
				 :name :handle-link))
     (dolist (l (send handle-link :bodies))
       (send l :set-color :gray30))
     handle-link
     )
   )
  (:make-base-link
   ()
   (let* ((pipe-thickness 30)
          (base-width 450)
          (base-length 1250)
          (plate-thickness 2)
          base-link
          base-flame-list
          handle-flame-list)
     ;; make handle-flame-list ;;;;;;;;;;;;;;;;;
     (let* ((vertical-pipe-length 130)
            (pipe-length 600)
            (pipe-angle -35)
            (handle-flame-pos-x (- (/ base-length 2) 120))
            (handle-base-length 160)
            (handle-base-width 280)
            (handle-base-height 160)
            (handle-base-pos-x -60)
            (handle-upper-base (make-cube handle-base-length handle-base-width handle-base-height))
            )
       ;; move each body
       ;; (send handle-upper-base :translate (float-vector (+ (- (* pipe-length (cos (deg2rad (+ 90 pipe-angle))))) handle-upper-base-pos-x) 0 (+ (* pipe-length (sin (deg2rad (+ 90 pipe-angle)))) (/ vertical-pipe-length 2) (/ (- handle-upper-base-height handle-lower-base-height) 2))) :world)
       ;; set list and assoc
       (setq handle-flame-list (list handle-upper-base))
       ;; move
       (send (car handle-flame-list) :translate (float-vector (/ handle-base-length 2) 0 (/ handle-base-height 2)) :world)
       )
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq base-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append handle-flame-list)
                               :name :base-link))
     (dolist (l (send base-link :bodies))
       (send l :set-color :gray30))
     base-link
     )
   )
  (:reset
   ()
   (let ((handle-joint-default-angle 0))
     (send self :handle-joint :joint-angle handle-joint-default-angle)
     )
   )
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;
  (:handle-center () (find-if #'(lambda (h) (eq (send h :name) :handle-center)) handle))
  (:handle-top () (find-if #'(lambda (h) (eq (send h :name) :handle-top)) handle))
  (:handle-left () (find-if #'(lambda (h) (eq (send h :name) :handle-left)) handle))
  (:handle-right () (find-if #'(lambda (h) (eq (send h :name) :handle-right)) handle))
  (:handle-left-upper () (find-if #'(lambda (h) (eq (send h :name) :handle-left-upper)) handle))
  (:handle-right-upper () (find-if #'(lambda (h) (eq (send h :name) :handle-right-upper)) handle))
  (:handle-top-side () (find-if #'(lambda (h) (eq (send h :name) :handle-top-side)) handle))
  (:handle-left-side () (find-if #'(lambda (h) (eq (send h :name) :handle-left-side)) handle))
  (:handle-right-side () (find-if #'(lambda (h) (eq (send h :name) :handle-right-side)) handle))
  (:handle-left-upper-side () (find-if #'(lambda (h) (eq (send h :name) :handle-left-upper-side)) handle))
  (:handle-right-upper-side () (find-if #'(lambda (h) (eq (send h :name) :handle-right-upper-side)) handle))
  (:handle-crank () (find-if #'(lambda (h) (eq (send h :name) :handle-crank)) handle))
  ;; methods to send message to joints ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:handle-joint (&rest args) (forward-message-to handle-joint args))
  (:steering-center () (send (car (send (find-if #'(lambda (h) (eq (send h :name) :handle-link)) links) :bodies)) :copy-worldcoords))
  (:steering-radius () steering-radius)
  )


;; origin of coords: lower right corner of pedal-base
(defclass drive-sim-pedal
  :super cascaded-link
  :slots (handle accel-joint brake-joint)
  )
(defmethod drive-sim-pedal
  (:init
   (&rest args)
   (send-super* :init args)
   (setq handle nil)
   ;; 1. make links links and assoc all links
   (let ((base (send self :make-base-link))
         (accel (send self :make-accel-pedal-link))
         (brake (send self :make-brake-pedal-link)))
     (let ((accel-pos-x 230) ;; old 230
           (accel-pos-y 60)
           (accel-pos-z 80) ;; old 130
           (brake-pos-x 230) ;; old 230
           (brake-pos-y 200)
           (brake-pos-z 98)) ;; old 128
       (send accel :translate (float-vector accel-pos-x accel-pos-y accel-pos-z) :world)
       (send brake :translate (float-vector brake-pos-x brake-pos-y brake-pos-z) :world)
       )
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc base)
     (send base :assoc accel)
     (send base :assoc brake)
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
  (:make-base-link
   ()
   (let* ((pipe-thickness 30)
          (base-width 450)
          (base-length 1250)
          (plate-thickness 2)
          (pad-height (- 58 pipe-thickness))
          pedal-plate-list)
     ;; make pedal-plate-list ;;;;;;;;;;;;;;;;;
     (let* ((pedal-plate-pos-x (- 800 (/ base-length 2)))
            (pedal-plate-length 475)
            (pedal-base-width 270)
            (pedal-base-lengh 320)
            (pedal-base-height-1 74)
            (pedal-base-height-2 24)
            (pedal-base-pos-x 32)
            (pedal-base (make-prism (list (float-vector 0 0 0) (float-vector pedal-base-lengh 0 0)
                                          (float-vector pedal-base-lengh 0 pedal-base-height-1) (float-vector 0 0 pedal-base-height-2))
                                    (float-vector 0 pedal-base-width 0))))
       ;; move each body
       (setq pedal-plate-list (list pedal-base))
       )
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq base-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append pedal-plate-list)
                               :name :base-link))
     (dolist (l (send base-link :bodies))
       (send l :set-color :gray30))
     base-link
     )
   )
  ;; methods to define playseat accel-pedal-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:make-accel-pedal-link
   ()
   (let (accel-pedal-list
         accel-pedal-handle
         accel-pedal-link)
     ;; make accel-pedal-list ;;;;;;;;;;;;;;;;;
     (let* ((accel-pedal-length 112)
            (accel-pedal-height 50)
            (accel-pedal-width 61)
            (accel-pedal-thickness 4)
            (accel-pedal (make-cube accel-pedal-length accel-pedal-width accel-pedal-thickness)))
       ;; move each body
       ;; define handle
       (setq accel-pedal-handle (make-cascoords :coords (send accel-pedal :copy-worldcoords) :name :accel-pedal))
       ;; set list and assoc
       (setq accel-pedal-list (list accel-pedal))
       (send accel-pedal :assoc accel-pedal-handle)
       (push accel-pedal-handle handle)
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
  ;; methods to define playseat brake-pedal-link ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:make-brake-pedal-link
   ()
   (let (brake-pedal-list
         brake-pedal-handle
         brake-pedal-link)
     ;; make brake-pedal-list ;;;;;;;;;;;;;;;;;
     (let* ((brake-pedal-length 91)
            (brake-pedal-width 59)
            (brake-pedal-thickness 4)
            (brake-pedal-height 50)
            (brake-pedal (make-cube brake-pedal-length brake-pedal-width brake-pedal-thickness)))
       ;; move each body
       ;; define handle
       (setq brake-pedal-handle (make-cascoords :coords (send brake-pedal :copy-worldcoords) :name :brake-pedal))
       ;; set list and assoc
       (setq brake-pedal-list (list brake-pedal))
       (send brake-pedal :assoc brake-pedal-handle)
       (push brake-pedal-handle handle)
       (dolist (b (cdr brake-pedal-list)) (send (car brake-pedal-list) :assoc b))
       (setq brake-pedal-list (list brake-pedal))
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
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:accel-pedal () (find-if #'(lambda (h) (eq (send h :name) :accel-pedal)) handle))
  (:brake-pedal () (find-if #'(lambda (h) (eq (send h :name) :brake-pedal)) handle))
  ;; methods to send message to joitns ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:accel-joint (&rest args) (forward-message-to accel-joint args))
  (:brake-joint (&rest args) (forward-message-to brake-joint args))
  )

;; front: the position which have handle/pedal and display
;; left: y+ when front is defined as x+
(defclass drive-simulator
  :super cascaded-link
  :slots (grasp-point base seat handle pedal door door-joint)
  )
(defmethod drive-simulator
  (:init
   (&rest args &key (handle-mode :center))
   (send-super* :init args)
   (setq seat nil
         handle nil
         pedal nil
         door nil
         grasp-point nil)
   
   ;; make links links and assoc all links
   (setq handle (instance drive-sim-handle :init))
   (setq pedal (instance drive-sim-pedal :init))
   (setq seat (instance drive-sim-playseat :init))
   (setq base (send self :make-drive-simulator-base))
   (setq door (send self :make-drive-simulator-door))

   ;; move simulator
   (let ((pedal-x-in-drive-sim (- (/ (send base :get :floor-length) 2) 340))
         (pedal-y-in-drive-sim (- 270 (/ (send base :get :floor-width) 2)))
         (seat-x-in-drive-sim (- (/ (send base :get :floor-length) 2) 750))
         (seat-y-in-drive-sim 0)
         (handle-x-in-drive-sim (- (/ (send base :get :floor-length) 2) 360)) ;; old -400
         (handle-y-in-drive-sim 0) ;; default
         (handle-z-in-drive-sim 700)
         (door-x-in-drive-sim (/ (send base :get :floor-length) 2))
         (door-y-in-drive-sim (+ (/ (send base :get :floor-width) 2) (send base :get :pipe-thickness)))
         (door-z-in-drive-sim (send base :get :floor-height))
         )

     ;; handle position in drive-simulator
     (case handle-mode
       (:left (setq handle-y-in-drive-sim 50))
       (:center (setq handle-y-in-drive-sim 0))
       )
       
     (send pedal :translate (float-vector pedal-x-in-drive-sim pedal-y-in-drive-sim (send base :get :floor-height)) :world)
     (send seat :translate (float-vector seat-x-in-drive-sim seat-y-in-drive-sim (send base :get :floor-height)) :world)
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
   (setq links (list base handle pedal seat door))

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
  (:make-drive-simulator-door
   ()
   (let* ((door-pipe-thickness 30.0)
          (frame-door-front-height 630.0)
          (frame-door-rear-height 1250.0)
          (frame-door-upper-width 220.0)
          (frame-door-bottom-width 1070.0)
          (frame-door-diag-length 1060.0)
          (door-plate-height 500.0)
          (door-plate-width 1330.0)
          (door-plate-thickness 10.0)
          frame-list door-link
          )
     ;; make frame-list ;;;;;;;;;;;;;;;;;
     (let* ((frame-door-front (make-cube door-pipe-thickness door-pipe-thickness frame-door-front-height))
            (frame-door-rear (make-cube door-pipe-thickness door-pipe-thickness frame-door-rear-height))
            (frame-door-bottom (make-cube frame-door-bottom-width door-pipe-thickness door-pipe-thickness))
            (frame-door-upper (make-cube frame-door-upper-width door-pipe-thickness door-pipe-thickness))
            (frame-door-diag (make-cube frame-door-diag-length door-pipe-thickness door-pipe-thickness))
            (door-plate (make-cube door-plate-width door-plate-thickness door-plate-height))
            )
       ;; make frame
       (push (send frame-door-front :translate (float-vector 0 0 (/ frame-door-front-height 2))) frame-list)
       (push (send frame-door-rear :translate (float-vector (- (- frame-door-bottom-width door-pipe-thickness)) 0 (/ frame-door-rear-height 2))) frame-list)
       (push (send frame-door-bottom :translate (float-vector (- (- (/ frame-door-bottom-width 2) (/ door-pipe-thickness 2))) 0 0)) frame-list)
       (push (send (copy-object frame-door-bottom) :translate (float-vector 0 0 door-plate-height)) frame-list)
       (push (send frame-door-upper :translate (float-vector (- (- (- frame-door-bottom-width (/ frame-door-upper-width 2)) (/ door-pipe-thickness 2))) 0 frame-door-rear-height)) frame-list)
       (push (send (send frame-door-diag :rotate (asin (/ (- frame-door-rear-height frame-door-front-height) frame-door-diag-length)) :y)
                   :translate (float-vector (- (/ (- frame-door-bottom-width frame-door-upper-width) 2)) 0 (+ frame-door-front-height (/ (- frame-door-rear-height frame-door-front-height) 2)))
                   :world
                   )
             frame-list)
       (dolist (l frame-list)
         (send l :set-color :gray60)
         )
       ;; make door plate
       (push (send door-plate :translate (float-vector (- (/ door-plate-width 2)) (- (+ (/ door-pipe-thickness 2) (/ door-plate-thickness 2))) (/ door-plate-height 2))) frame-list)
       (send door-plate :set-color :yellow)

       ;; define grasp point
       (setq door-rear-grasp-point (make-cascoords :coords (send (send frame-door-rear :copy-worldcoords)
                                                                 :translate (float-vector 0 0 100) :world) :name :door-rear))
       (send frame-door-rear :assoc door-rear-grasp-point)
       (push door-rear-grasp-point grasp-point)
       
       ;; set list and assoc
       (dolist (frame (cdr frame-list)) (send (car frame-list) :assoc frame))
       ;; make bodyset-link ;;;;;;;;;;;;;;;;;
       (setq door-link (instance bodyset-link :init (make-cascoords)
                                 :bodies (append frame-list nil)
                                 :name :door-link))
       door-link
       )
     )
   )
  (:make-drive-simulator-base
   (&optional (arg-table-height 665) (arg-floor-height 310))
   (let* ((floor-height arg-floor-height)
          (floor-width 850)
          (floor-length 1670)
          (floor-thickness 25)
          (table-height arg-table-height)
          (table-width 850)
          (table-length 500)
          (table-thickness 35)
          (pipe-thickness 50)
          (pole-height 1555)
          (door-pole-offset 400)
          frame-list
          frame-pipe-list
          )
     ;; make frame-list ;;;;;;;;;;;;;;;;;
     (let* ((floor-corner-x (- (/ floor-length 2) (/ pipe-thickness 2)))
            (floor-corner-y (- (/ floor-width 2) (/ pipe-thickness 2)))            
            (frame-floor (make-cube floor-length floor-width floor-thickness))
            (frame-table (make-cube table-length table-width table-thickness))
            (frame-leg-origin (make-cylinder pipe-thickness (+ floor-height pole-height)))
            (frame-leg-front-left (copy-object frame-leg-origin))
            (frame-leg-front-right (copy-object frame-leg-origin))
            (frame-leg-back-left (copy-object frame-leg-origin))
            (frame-leg-back-right (copy-object frame-leg-origin))
            (frame-door-pole (make-cylinder pipe-thickness pole-height))
            (frame-seil-vertical (make-cylinder pipe-thickness floor-length))
            (frame-seil-vertical-left (copy-object frame-seil-vertical))
            (frame-seil-vertical-right (copy-object frame-seil-vertical))
            (frame-seil-horizontal (make-cylinder pipe-thickness floor-width))
            )
       ;; make frame by pipe
       (push (send frame-leg-front-left :translate (float-vector floor-corner-x floor-corner-y 0)) frame-pipe-list)
       (push (send frame-leg-front-right :translate (float-vector floor-corner-x (- floor-corner-y) 0)) frame-pipe-list)
       (push (send frame-leg-back-left :translate (float-vector (- floor-corner-x) floor-corner-y 0)) frame-pipe-list)
       (push (send frame-leg-back-right :translate (float-vector (- floor-corner-x) (- floor-corner-y) 0)) frame-pipe-list)
       (push (send frame-door-pole :translate (float-vector (+ (- floor-corner-x) door-pole-offset) floor-corner-y floor-height)) frame-pipe-list)
       (push (send (send frame-seil-vertical-left :translate (float-vector (- (+ floor-corner-x (/ pipe-thickness 2)))
                                                                           floor-corner-y (+ floor-height pole-height))) :rotate (/ pi 2) :y) frame-pipe-list)
       (push (send (send frame-seil-vertical-right :translate (float-vector (- (+ floor-corner-x (/ pipe-thickness 2)))
                                                                            (- floor-corner-y) (+ floor-height pole-height))) :rotate (/ pi 2) :y) frame-pipe-list)       
       (push (send (send frame-seil-horizontal :translate (float-vector floor-corner-x (+ floor-corner-y (/ pipe-thickness 2))
                                                                        (+ floor-height pole-height))) :rotate (/ pi 2) :x) frame-pipe-list)
       ;; make table
       (send frame-table :translate (float-vector (- (+ (/ floor-length 2) (/ table-length 2)) 340) 0 (+ floor-height table-height (/ table-thickness 2))))
       (push frame-table frame-list)
       ;; make floor
       (send frame-floor :translate (float-vector 0 0 floor-height))
       (push frame-floor frame-list)
       (dolist (pipe frame-pipe-list)
         (send frame-floor :assoc pipe)
         )
       (send frame-floor :assoc frame-table)

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
       (send base-link :put :table-height (+ table-height table-thickness))

       ;; define grasp point
       (setq leg-front-left-grasp-point (make-cascoords :coords (send (send frame-leg-front-left :copy-worldcoords)
                                                                 :translate (float-vector 0 0 (/ (+ floor-height pole-height) 2)) :world)
                                                        :name :leg-front-left))
       (send frame-leg-front-left :assoc leg-front-left-grasp-point)
       (push leg-front-left-grasp-point grasp-point)

       (setq leg-front-right-grasp-point (make-cascoords :coords (send (send frame-leg-front-right :copy-worldcoords)
                                                                 :translate (float-vector 0 0 (/ (+ floor-height pole-height) 2)) :world)
                                                        :name :leg-front-right))
       (send frame-leg-front-right :assoc leg-front-right-grasp-point)
       (push leg-front-right-grasp-point grasp-point)

       (setq leg-back-left-grasp-point (make-cascoords :coords (send (send frame-leg-back-left :copy-worldcoords)
                                                                 :translate (float-vector 0 0 (/ (+ floor-height pole-height) 2)) :world)
                                                        :name :leg-back-left))
       (send frame-leg-back-left :assoc leg-back-left-grasp-point)
       (push leg-back-left-grasp-point grasp-point)

       (setq leg-back-right-grasp-point (make-cascoords :coords (send (send frame-leg-back-right :copy-worldcoords)
                                                                 :translate (float-vector 0 0 (/ (+ floor-height pole-height) 2)) :world)
                                                        :name :leg-back-right))
       (send frame-leg-back-right :assoc leg-back-right-grasp-point)
       (push leg-back-right-grasp-point grasp-point)
       
       (setq seil-left-grasp-point (make-cascoords :coords (send (send frame-seil-vertical-left :copy-worldcoords)
                                                                 :translate (float-vector (/ floor-length 2) 0 0) :world)
                                                   :name :seil-left))
       (send frame-seil-vertical-left :assoc seil-left-grasp-point)
       (push seil-left-grasp-point grasp-point)
       
       (setq seil-right-grasp-point (make-cascoords :coords (send (send frame-seil-vertical-right :copy-worldcoords)
                                                                  :translate (float-vector (/ floor-length 2) 0 0) :world)
                                                    :name :seil-right))
       (send frame-seil-vertical-right :assoc seil-right-grasp-point)
       (push seil-right-grasp-point grasp-point)

       (setq seil-front-grasp-point (make-cascoords :coords (send (send frame-seil-horizontal :copy-worldcoords)
                                                                 :translate (float-vector 0 (- (/ floor-width 2)) 0) :world)
                                                   :name :seil-front))
       (send frame-seil-horizontal :assoc seil-front-grasp-point)
       (push seil-front-grasp-point grasp-point)

       (setq door-pole-grasp-point (make-cascoords :coords (send (send frame-door-pole :copy-worldcoords)
                                                                 :translate (float-vector 0 0 (/ pole-height 2)) :world)
                                                   :name :door-pole))
       (send frame-door-pole :assoc door-pole-grasp-point)
       (push door-pole-grasp-point grasp-point)

       (setq floor-point (make-cascoords :coords (send (send frame-floor :copy-worldcoords)
                                                       :translate (float-vector 0 0 0) :world) ;; old: (float-vector 400 350 0)
                                                   :name :floor))
       (send frame-floor :assoc floor-point)
       (push floor-point grasp-point)

       base-link
       )
     )
   )
  (:reset-door
   ()
   (let ((door-joint-default-angle 0))
     (send self :door-joint :joint-angle door-joint-default-angle)
     )
   )
  (:reset
   (&optional (target nil))
   (case target
     (:seat (send seat :reset))
     (:handle (send handle :reset))
     (:pedals (send pedal :reset))
     (:accel (send pedal :reset :accel))
     (:brake (send pedal :reset :brake))
     (:door (send self :reset-door))
     (nil (send seat :reset) ;; all reset
          (send handle :reset)
          (send pedal :reset)
          (send self :reset-door)
          )
     )
   )
  ;; methods to get drive seat objects
  (:vehicle-base (&rest args) (forward-message-to base args))
  (:vehicle-seat (&rest args) (forward-message-to seat args))
  (:vehicle-handle (&rest args) (forward-message-to handle args))
  (:vehicle-pedal (&rest args) (forward-message-to pedal args))
  (:vehicle-door (&rest args) (forward-message-to door args))
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:handle-center () (send handle :handle-center))
  (:handle-top () (send handle :handle-top))
  (:handle-left () (send handle :handle-left))
  (:handle-right () (send handle :handle-right))
  (:handle-left-upper () (send handle :handle-left-upper))
  (:handle-right-upper () (send handle :handle-right-upper))
  (:handle-top-side () (send handle :handle-top-side))
  (:handle-left-side () (send handle :handle-left-side))
  (:handle-right-side () (send handle :handle-right-side))
  (:handle-left-upper-side () (send handle :handle-left-upper-side))
  (:handle-right-upper-side () (send handle :handle-right-upper-side))
  (:handle-crank () (send handle :handle-crank))
  (:accel-pedal () (send pedal :accel-pedal))
  (:brake-pedal () (send pedal :brake-pedal))
  (:seat () (send seat :seat))
  (:seat-left () (send seat :seat-left))
  (:seat-right () (send seat :seat-right))
  (:back-left () (send seat :back-left))
  (:back-right () (send seat :back-right))
  (:door-rear () (find-if #'(lambda (h) (eq (send h :name) :door-rear)) grasp-point))
  (:leg-front-left () (find-if #'(lambda (h) (eq (send h :name) :leg-front-left)) grasp-point))
  (:leg-front-right () (find-if #'(lambda (h) (eq (send h :name) :leg-front-right)) grasp-point))
  (:leg-back-left () (find-if #'(lambda (h) (eq (send h :name) :leg-back-left)) grasp-point))
  (:leg-back-right () (find-if #'(lambda (h) (eq (send h :name) :leg-back-right)) grasp-point))
  (:seil-left () (find-if #'(lambda (h) (eq (send h :name) :seil-left)) grasp-point))
  (:seil-right () (find-if #'(lambda (h) (eq (send h :name) :seil-right)) grasp-point))
  (:seil-front () (find-if #'(lambda (h) (eq (send h :name) :seil-front)) grasp-point))
  (:door-pole () (find-if #'(lambda (h) (eq (send h :name) :door-pole)) grasp-point))
  (:floor () (find-if #'(lambda (h) (eq (send h :name) :floor)) grasp-point))
  ;; methods to send message to joints ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:slide-joint (&rest args) (forward-message-to (send seat :slide-joint) args))
  (:reclining-joint (&rest args) (forward-message-to (send seat :reclining-joint) args))
  (:handle-joint (&rest args) (forward-message-to (send handle :handle-joint) args))
  (:accel-joint (&rest args) (forward-message-to (send pedal :accel-joint) args))
  (:brake-joint (&rest args) (forward-message-to (send pedal :brake-joint) args))
  (:door-joint (&rest args) (forward-message-to door-joint args))
  )

;; (setq *ds* (instance drive-simulator :init))
;; (setq hoge (make-arrow))
;; (send hoge :newcoords (send (send *ds* :leg-front-left) :copy-worldcoords))
;; (setq fuga (make-arrow))
;; (send fuga :newcoords (send (send *ds* :leg-front-right) :copy-worldcoords))
;; (objects  (list *ds* hoge))

;; (setq *ds* (instance drive-simulator :init))
;; (setq fuga (make-arrow))
;; (setq hoge0 (make-arrow))
;; (send hoge0 :newcoords (send (send *ds* :handle-top) :copy-worldcoords))
;; (setq hoge1 (make-arrow))
;; (send hoge1 :newcoords (send (send *ds* :handle-right-upper) :copy-worldcoords))
;; (setq hoge2 (make-arrow))
;; (send hoge2 :newcoords (send (send *ds* :handle-right) :copy-worldcoords))
;; (setq hoge3 (make-arrow))
;; (send hoge3 :newcoords (send (send *ds* :handle-left-upper) :copy-worldcoords))
;; (setq hoge4 (make-arrow))
;; (send hoge4 :newcoords (send (send *ds* :handle-left) :copy-worldcoords))
;; (objects  (list *ds* hoge0 hoge1 hoge2 hoge3 hoge4 fuga))
