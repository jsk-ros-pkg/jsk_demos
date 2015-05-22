;;;;;;;;;;;;;;;;
;; DRC testbed models
;; (snozawa, mmurooka, s-noda)
;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;
;; Utilities
;;;;;;;;;;;;;;;;
(defclass single-link-model
  :super cascaded-link
  :slots ()
  )

(defmethod single-link-model
  (:init
   (&key (name) ((:bodies bs)))
   (prog1
       (send-super :init :name name)
     (setq links (list (instance bodyset-link :init (make-cascoords)
                                 :bodies bs :name :root-link)))
     (send self :assoc (car links))
     (send self :init-ending)
     ))
  )

(defun make-cylinder-shell
  (outer-radius inner-radius height)
  (let ((b0 (make-cylinder outer-radius height))
        (b1 (make-cylinder inner-radius (+ 1 height))))
    (body- b0 b1)
    ))

;;;;;;;;;;;;;;;;
;; Model classes
;;;;;;;;;;;;;;;;

;; copied from jskmapclsas
(defclass valve
  :super cascaded-link
  :slots (handle valve-joint)
  )

(defmethod valve
  (:init
   (&rest args
    &key (thick 20) (radius 115)
    (ring-color :gray10) (base-color :royalblue)
    (type :round) (with-root-pipe t))
    (send-super* :init args)
   (setq handle nil)
   ;; 1. make links links and assoc all links
   (let ((rl (send self :make-root-link
		   :base-color base-color :with-root-pipe with-root-pipe))
	 (vl (send self :make-valve-link
		   :thick thick :radius radius :ring-color ring-color :type type)))
     (send vl :translate #f(0 0 353) :world)  ;;vavle's hight (H)
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc rl)
     (send rl :assoc vl)
     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (setq valve-joint (instance rotational-joint :init
				 :parent-link rl :child-link vl
				 :name :crank-joint :axis :z
				 :min -3600 :max 3600))
     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (list rl vl))
     (setq joint-list (list valve-joint))
     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self))
  ;; Methods to define robot links
  (:make-root-link
   (&key base-color with-root-pipe)
   (let (base-cylinder
	 (r 90)
	 (L 240)
	 (H 353)
	 (thick 30)
	 tmp
	 cylinder1
	 cylinder2
	 cylinder3
	 cylinder4
	 upper-body
	 sphere
	 rl
	 )
     ;;base cylinder
     (setq base-cylinder (make-cylinder r L))
     (send base-cylinder :rotate (deg2rad 90) :x)
     (send base-cylinder :locate (float-vector 0 (* 0.5 L) 0) :world)
     (setq tmp (make-cube (* L 2) (- L (* thick 2))  (* L 2)))
     (setq base-cylinder (body- base-cylinder tmp))
     (setq sphere (make-gdome (make-icosahedron r)))
     (setq cylinder1 (make-cylinder (* r 0.8) (- L (* 2 thick))))
     (send cylinder1 :rotate (deg2rad 90) :x)
     (send cylinder1 :locate (float-vector 0 (* 0.5 (- L (* 2 thick))) 0) :world)
     ;;
     (setq tmp (make-cylinder (* r 0.5) (* L 2)))
     (send tmp :rotate (deg2rad 90) :x)
     (send tmp :locate (float-vector 0 0 (* -1 L)))
     (setq base-cylinder (body- base-cylinder tmp))
     (setq cylinder1 (body- cylinder1 tmp))
     ;;(setq sphere (body- sphere tmp))
     ;;
     (setq cylinder2 (make-cylinder r thick))
     (send cylinder2 :locate (float-vector 0 0 r))
     (setq cylinder3 (make-cylinder (* r 0.8) (* thick 2)))
     (send cylinder3 :locate (float-vector 0 0 (- r (* 2 thick))))
     ;;
     (setq cylinder4 (make-cylinder 20 H))
     ;;(send cylinder4 :locate (float-vector 0 0 (* 0.5 H)))
     ;;upper body
     (setq upper-body 
	   (make-solid-of-revolution
	    (mapcar #'(lambda (x) (scale (* 0.7 (/ (- H r) 3.5)) x)) (list #f(0 0 3.5) #f(0.6 0 3) #f(1.0 0 2) #f(1.4 0 1) #f(1.0 0 0)))))
     (send upper-body :locate (float-vector 0 0 (+ r thick)))
     ;;
     (send upper-body :assoc sphere)
     (send upper-body :assoc cylinder1)
     (send upper-body :assoc cylinder2)
     (send upper-body :assoc cylinder3)
     (send upper-body :assoc cylinder4)
     (send upper-body :assoc base-cylinder)
     (setq rl (instance bodyset-link :init (make-cascoords)
			:bodies (if with-root-pipe (list upper-body base-cylinder sphere cylinder1 cylinder2 cylinder3 cylinder4) (list upper-body cylinder4))
			:name :crank-root-link))
     (dolist (l (send rl :bodies))
       (send l :set-color base-color))
     rl
     ))
  (:make-valve-link
   (&key thick radius ring-color type)
   (cond ((equal type :round)
	  (let* ((segments 16)
		 (ring-radius radius)
		 (pipe-radius thick)
		 (ring (make-ring ring-radius pipe-radius :segments segments))
		 (cross-bar1 (make-cube pipe-radius (* ring-radius 2) pipe-radius))
		 (cross-bar2 (make-cube (* ring-radius 2) pipe-radius pipe-radius)))
	    (send ring :assoc cross-bar1)
	    (send ring :assoc cross-bar2)
	    (let ((valve-link (instance bodyset-link :init (make-cascoords)
					:bodies (list ring cross-bar1 cross-bar2) :name :valve-handle-link)))
	      ;;
	      (push (make-cascoords :coords (send (send (send ring :copy-worldcoords)
                                                        :translate (float-vector 0 (* -1 radius) 0))
                                                  :rotate (/ pi 2) :y)
				    :name :valve-handle) handle)
	      (send valve-link :assoc (car handle))
	      (dolist (l (send valve-link :bodies))
		(send l :set-color ring-color))
	      valve-link)))
	 ((equal type :bar)
	  (let* ((segments 16)
		 (bar-thick (/ 27 2))
		 (bar-length 393)
		 (bar-root-thick (/ 45.5 2))
		 (bar-root-length 100)
		 (bar (make-cylinder bar-thick bar-length :segments segments))
		 (bar-root (make-cylinder bar-root-thick bar-root-length :segments segments)))
	    (send bar-root :assoc bar)
	    (send bar-root :rotate pi/2 :x :world)
	    (send bar-root :translate (float-vector 0 (/ bar-root-length 2.0) 0) :world)
	    (let ((valve-link (instance bodyset-link :init (make-cascoords)
					:bodies (list bar-root bar) :name :valve-handle-link)))
	      ;;
	      (push (make-cascoords :coords (send (send (send bar-root :copy-worldcoords)
                                                        :translate (float-vector 0 0 350))
                                                  :rotate (/ -pi 2) :z)
				    :name :valve-handle) handle)
	      (send valve-link :assoc (car handle))
	      (dolist (l (send valve-link :bodies))
		(send l :set-color ring-color))
	      valve-link))))
   )
  (:handle (&rest args) (forward-message-to-all handle args))
  (:valve-handle () (car handle))
  (:valve-joint (&rest args) (forward-message-to valve-joint args))
  )

(defclass drc-stair
  :slots (stair-top-faces ground-body)
  :super single-link-model
  )

(defmethod drc-stair
  (:init
   (&key (add-groud-p nil))
   (setq stair-top-faces nil)
   (let* ((blist (append (send self :make-stair-bodies)
                         (send self :make-top-stair-bodies)
                         (send self :make-handrail-bodies)
                         (send self :make-small-handrail-bodies :l/r :right)
                         (send self :make-small-handrail-bodies)
                         (send self :make-top-handrail-bodies))))
     (if add-groud-p
         (let ((b (make-cube 1000 1200 10)))
           (send b :translate (float-vector 0 0 -5))
           (setq ground-body b)
           (setq stair-top-faces (append stair-top-faces
                                         (list (find-if #'(lambda (x) (memq :top (send x :id))) (send ground-body :faces)))))
           (push b blist)))
     (dolist (b blist) (send b :set-color :gray))
     (dolist (b (cdr blist)) (send (car blist) :assoc b))
     (send-super :init :name "drc-stair" :bodies blist)))
  (:make-stair-bodies
   ()
   (let ((s1 (make-cube 290 1080 225))
         (s2 (make-cube 290 1080 230))
         (s3 (make-cube 270 1080 230)))
     (send s1 :translate (float-vector (/ 290 2.0) 0 (/ 225 2.0)) :world)
     (send s2 :translate (float-vector (+ 290 (/ 290 2.0)) 0 (+ 225 (/ 230 2.0))) :world)
     (send s3 :translate (float-vector (+ 290 290 (/ 270 2.0)) 0 (+ 225 230 (/ 230 2.0))) :world)
     (setq stair-top-faces
           (append stair-top-faces
                   (mapcar #'(lambda (b)
                               (find-if #'(lambda (x) (memq :top (send x :id))) (send b :faces)))
                   (list s1 s2 s3))))
     (list s1 s2 s3)
     ))
  (:make-top-stair-bodies
   ()
   (let ((s1 (make-cube 1205 2430 85)))
     (send s1 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (+ (/ 1080 2.0) (- (/ 2430 2.0)))) (+ 225 230 230 (/ 85 2.0) -85 220)))
     (setq stair-top-faces
           (append stair-top-faces
                   (mapcar #'(lambda (b)
                               (find-if #'(lambda (x) (memq :top (send x :id))) (send b :faces)))
                   (list s1))))
     (list s1)
     ))
  (:make-small-handrail-bodies
   (&key (l/r :left))
   (let ((b0 (make-cylinder 20 100))
         (b1 (make-cylinder 20 100))
         (b2 (make-cylinder 20 195)))
     (send b0 :rotate -pi/2 :y)
     (send b1 :rotate -pi/2 :y)
     (send b0 :translate (float-vector 0 0 (/ (+ 850 890) 2.0)) :world)
     (send b1 :translate (float-vector 0 0 (/ (+ 1045 1005) 2.0)) :world)
     (send b2 :translate (float-vector -100 0 850) :world)
     (if (eq l/r :left)
         (send-all (list b0 b1 b2) :translate (float-vector 0 (/ 1080 2.0) 0))
       (send-all (list b0 b1 b2) :translate (float-vector 0 (/ 1080 -2.0) 0)))
     (list b0 b1 b2)
     ))
  (:make-handrail-bodies
   ()
   (let ((bf1 (make-cube 50 40 (+ 225 835)))
         (bf2 (make-cube 50 40 (+ 225 835)))
         (br1 (make-cube 50 40 995))
         (br2 (make-cube 50 40 995))
         (hr1 (make-cylinder 20 1040))
         (hl1 (make-cylinder 20 1040))
         (hr2 (make-cylinder 20 1040))
         (hl2 (make-cylinder 20 1040)))
     (send bf1 :translate (float-vector (/ 50 -2.0) (+ (/ 40 -2.0) (/ 1080 2.0)) (/ (+ 225 835) 2.0)))
     (send bf2 :translate (float-vector (/ 50 -2.0) (+ (/ 40 2.0) (/ 1080 -2.0)) (/ (+ 225 835) 2.0)))
     (send br1 :translate (float-vector (+ 290 290 230 (/ 50 -2.0)) (+ (/ 40 -2.0) (/ 1080 2.0)) (+ 225 230 230 (/ 995 2.0))))
     (send br2 :translate (float-vector (+ 290 290 230 (/ 50 -2.0)) (+ (/ 40 2.0) (/ 1080 -2.0)) (+ 225 230 230 (/ 995 2.0))))
     ;;(send hr1 :rotate (deg2rad (- 90 40)) :y)
     ;;(send hl1 :rotate (deg2rad (- 90 40)) :y)
     (send hr1 :rotate (deg2rad (- 90 38)) :y)
     (send hl1 :rotate (deg2rad (- 90 38)) :y)
     (send hr2 :rotate (deg2rad (- 90 38)) :y)
     (send hl2 :rotate (deg2rad (- 90 38)) :y)
     (send hr1 :translate (float-vector -40 (+ (/ 40 -2.0) (/ 1080 2.0)) (+ 225 835 -20)) :world)
     (send hl1 :translate (float-vector -40 (+ (/ 40 2.0) (/ 1080 -2.0)) (+ 225 835 -20)) :world)
     (send hr2 :translate (float-vector -40 (+ (/ 40 -2.0) (/ 1080 2.0)) (+ 225 385 -20)) :world)
     (send hl2 :translate (float-vector -40 (+ (/ 40 2.0) (/ 1080 -2.0)) (+ 225 385 -20)) :world)
     (list bf1 bf2 br1 br2 hr1 hl1 hr2 hl2)
     ))
  (:make-top-handrail-bodies
   ()
   (let ((b1 (make-cube 35 50 1065))
         (b2 (make-cube 35 50 1065))
         (b3 (make-cube 35 50 1065))
         (b4 (make-cube 35 50 1065))
         (b5 (make-cube 35 50 1065))
         (b6 (make-cube 35 50 1065))
         ;;
         (b7 (make-cube 1205 50 40))
         (b8 (make-cube 1205 50 40))
         (b9 (make-cube 1205 50 40))
         (b10 (make-cube 1205 50 40))
         (b11 (make-cube 50 2430 40))
         (b12 (make-cube 50 2430 40))
         ;;
         (b13 (make-cube 1205 10 105))
         (b14 (make-cube 1205 10 105))
         )
     (send b1 :translate (float-vector (+ 290 290 270 (/ 35 2.0)) (+ 85 (/ 1080 2.0) (/ 50 2.0)) (+ 225 230 230 220 (/ 1065 2.0))))
     (send b2 :translate (float-vector (+ 290 290 270 (/ 35 -2.0) 1205) (+ 85 (/ 1080 2.0) (/ 50 2.0)) (+ 225 230 230 220 (/ 1065 2.0))))
     (send b3 :translate (float-vector (+ 290 290 270 (/ 35 -2.0) 1205) (+ 85 (/ 1080 2.0) (/ 50 -2.0) -2430) (+ 225 230 230 220 (/ 1065 2.0))))
     (send b4 :translate (float-vector (+ 290 290 270 (/ 35 2.0)) (+ 85 (/ 1080 2.0) (/ 50 -2.0) -2430) (+ 225 230 230 220 (/ 1065 2.0))))
     (send b5 :translate (float-vector (+ 290 290 270 1205 (/ 35 2.0)) (+ 85 -100 (/ 1080 2.0) (/ 50 2.0)) (+ 225 230 230 220 (/ 1065 2.0))))
     (send b6 :translate (float-vector (+ 290 290 270 1205 (/ 35 2.0)) (+ 85 100 (/ 1080 2.0) (/ 50 -2.0) -2430) (+ 225 230 230 220 (/ 1065 2.0))))
     ;;
     (send b7 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 50 2.0)) (+ 225 230 230 220 1065 (/ 40 2.0))))
     (send b8 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 50 2.0)) (+ 225 230 230 220 550 (/ 40 2.0))))
     (send b9 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 50 -2.0) -2430) (+ 225 230 230 220 1065 (/ 40 2.0))))
     (send b10 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 50 -2.0) -2430) (+ 225 230 230 220 550 (/ 40 2.0))))
     (send b11 :translate (float-vector (+ 290 290 270 1205 (/ 50 2.0)) (+ 85 (/ 1080 2.0) (/ 2430 -2.0)) (+ 225 230 230 220 1065 (/ 40 2.0))))
     (send b12 :translate (float-vector (+ 290 290 270 1205 (/ 50 2.0)) (+ 85 (/ 1080 2.0) (/ 2430 -2.0)) (+ 225 230 230 220 550 (/ 40 2.0))))
     ;;
     (send b13 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 10 -2.0) -2430) (+ 225 230 230 220 (/ 40 2.0))))
     (send b14 :translate (float-vector (+ 290 290 270 (/ 1205 2.0)) (+ 85 (/ 1080 2.0) (/ 10 -2.0) 0) (+ 225 230 230 220 (/ 40 2.0))))
     (list b1 b2 b3 b4 b5 b6
           b7 b8 b9 b10 b11 b12
           b13 b14)
     ))
  (:get-all-stair-top-faces
   ()
   stair-top-faces)
  )

(defclass drc-hose-wall
  :super single-link-model
  :slots ()
  )

(defmethod drc-hose-wall
  (:init
   ()
   (let ((blist (send self :make-hose-wall-bodies)))
     (dolist (b (cdr blist)) (send (car blist) :assoc b))
     (send-super :init :name "drc-hose-wall" :bodies blist)))
  (:make-hose-consent
   ()
   (let ((b1 (make-cube 75 65 165))
         (b2 (make-cylinder-shell (/ 56 2.0) (/ 48 2.0) 25.0))
         (b3 (make-cylinder-shell (/ 56 2.0) (/ 48 2.0) 25.0))
         (blist))
     (send b1 :translate (float-vector (/ 75 -2.0) 0 0))
     (send b2 :rotate pi/2 :y)
     (send b2 :translate (float-vector (+ -75 -25) 0 (+ (/ 165 2.0) -40)) :world)
     (send b3 :translate (float-vector (/ 75 -2.0) 0 (+ (/ 165 -2.0) -25.0)) :world)
     (let ((blist (list b1 b2 b3)))
       (dolist (b (cdr blist)) (send (car blist) :assoc b))
       blist
       )))
  (:make-hose-wall-bodies
   ()
   (let ((b0 (make-cube 10 1210 2430))
         (blist1 (send self :make-hose-consent))
         (blist2 (send self :make-hose-consent))
         (blist))
     (send b0 :set-color #F(0.8 0.5 0))
     (send b0 :translate (float-vector (/ 10 2.0) 0 (/ 2430 2.0)))
     (send (car blist1) :translate (float-vector 0 200 (+ (/ 165 2.0) 1120)))
     (send (car blist2) :translate (float-vector 0 -200 (+ (/ 165 2.0) 1120)))
     (send b0 :assoc (car blist1))
     (send b0 :assoc (car blist2))
     (append (list b0) blist1 blist2)))
  )

(defclass drc-hose-plug
  :super single-link-model
  :slots ()
  )

(defmethod drc-hose-plug
  (:init
   ()
   (let ((blist (send self :make-hose-plug-bodies)))
     (dolist (b (cdr blist)) (send (car blist) :assoc b))
     (send-super :init :name "drc-hose-plug" :bodies blist)))
  (:make-hose-plug-bodies
   ()
   (let ((b1 (make-cylinder (/ 45 2.0) 85.0))
         (b2 (make-cube 20 75 10)))
     (send b1 :rotate pi/2 :y)
     (send b2 :translate (float-vector (/ 20 2.0) 0 0))
     (send b1 :assoc b2)
     (send-all (list b1 b2) :set-color :gray20)
     (list b1 b2)
     ))
  )

(defclass drc-drill-wall
  :super single-link-model
  :slots ()
  )

(defmethod drc-drill-wall
  (:init
   ()
   (let ((blist (send self :make-drill-wall-bodies)))
     (dolist (b (cdr blist)) (send (car blist) :assoc b))
     (send-super :init :name "drc-drill-wall" :bodies blist)))
  (:make-drill-wall-bodies
   ()
   (let ((b1 (make-cube 10 1210 2430))
         (b2 (make-cube 10 1210 2430))
         (b2- (make-cube 20 810 810))
         (b3 (make-cube 250 440 10)))
     (send b2- :translate (float-vector 0 0 (+ 720 (/ 810 2.0) (/ 2430 -2.0))))
     (setq b2 (body- b2 b2-))
     (send b1 :translate (float-vector (/ 10 2.0) 0 (/ 2430 2.0)) :world)
     (send b2 :translate (float-vector (/ 10 2.0) 0 (/ 2430 2.0)) :world)
     (send b3 :translate (float-vector (/ 250 -2.0) 0 (+ (/ 10 2.0) 1020)))
     (send b1 :assoc b3)
     (send b1 :rotate (deg2rad 45) :z)
     (send b2 :rotate (deg2rad -45) :z)
     (send b1 :translate (float-vector 0 (* 1210 0.5) 0))
     (send b2 :translate (float-vector 0 (* 1210 -0.5) 0))
     (send-all (list b1 b2 b3) :set-color #f(0.8 0.5 0))
     (list b1 b2 b3)
     ))
  )

(defclass drc-terrain
  :super cascaded-link
  :slots (block-dimensions block-bodies ground-bodies block-angle)
  )

(defmethod drc-terrain
  (:init
   (&rest args &key (name "drc-terrain") ((:block-dimensions bd) (list 390 195 140)) (add-groud-p nil)
          ((:block-angle ba) 15.0) ;; [deg]
    &allow-other-keys)
   (prog1
       (send-super* :init :name name args)
     (setq block-dimensions bd)
     (setq block-angle ba)
     (let* ((blist (send self :make-drc-terrain-block-bodies))
            (l))
       (case
        add-groud-p
        (:test-field
         (let ((b0 (make-cube 2780 3300 10))
               (b1 (make-cube 400 3300 30))
               (b2 (make-cube 400 3300 30)))
           (send b0 :translate (float-vector 1200 1050 -5))
           (send b0 :set-color :gray)
           (send b1 :translate (float-vector -380 1050 15))
           (send b1 :set-color :darkgoldenrod)
           (send b2 :translate (float-vector 2800 1050 15))
           (send b2 :set-color :darkgoldenrod)
           (send (car blist) :assoc b0)
           (send (car blist) :assoc b1)
           (send (car blist) :assoc b2)
           (setq ground-bodies (list b0 b1 b2))
           (setq blist (append blist (list b0 b1 b2)))
           (send (car blist) :translate (float-vector 0 0 -30))
           ))
        (t
         (let ((b (make-cube 3700 3300 10)))
           (send b :translate (float-vector 1200 1050 -5))
           (send b :set-color :gray)
           (setq ground-bodies (list b))
           (send (car blist) :assoc b)
           (setq blist (append blist (list b))))))
       (setq l (instance bodyset-link :init (make-cascoords)
                         :name :root-link
                         :bodies blist))
       (send self :assoc l)
       (setq links (list l))
       (setq joint-list (list))
       (send self :init-ending)
       self)))
  (:make-drc-block-one
   ()
   (let* ((block-x (elt block-dimensions 0))
          (block-y (elt block-dimensions 1))
          (block-z (elt block-dimensions 2))
          (block-hole-x 170)
          (block-hole-y 200)
          (block-hole-z 85)
          (block-body (make-cube block-x block-y block-z))
          (block-hole1 (make-cube block-hole-x block-hole-y block-hole-z))
          (block-hole2 (make-cube block-hole-x block-hole-y block-hole-z))
          )
     (send block-hole1 :translate (float-vector (/ (+ block-hole-x (/ (- block-x (* block-hole-x 2)) 3.0)) 2.0) 0 0) :local)
     (send block-hole2 :translate (float-vector (- (/ (+ block-hole-x (/ (- block-x (* block-hole-x 2)) 3.0)) 2.0)) 0 0) :local)
     (setq block-body (body- block-body block-hole1))
     (setq block-body (body- block-body block-hole2))
     (send block-body :set-color :gray)
     block-body
     )
   )
  (:make-drc-block-set-one
   ()
   (let* ((block-x (elt block-dimensions 0))
          (block-y (elt block-dimensions 1))
          (block-z (elt block-dimensions 2))
          (block1 (send self :make-drc-block-one))
          (block2 (send self :make-drc-block-one))
          (base-x 90)
          (base-y block-x)
          (base-z 90)
          (base (make-cube base-x base-y base-z))
          )
     (send block1 :translate (float-vector 0 (/ block-y 2.0) (+ (/ block-z 2.0) (* (/ block-x 2.0) (sin (deg2rad block-angle))))) :local)
     (send block2 :translate (float-vector 0 (- (/ block-y 2.0)) (+ (/ block-z 2.0) (* (/ block-x 2.0) (sin (deg2rad block-angle))))) :local)
     (send base :translate (float-vector (- (- (/ block-x 2.0) (/ base-x 2.0))) 0 (/ base-x 2.0)) :local)
     (send block1 :rotate (deg2rad block-angle) :y :local)
     (send block2 :rotate (deg2rad block-angle) :y :local)
     (send base :set-color :darkgoldenrod)
     (send block1 :assoc block2)
     (send block1 :assoc base)
     (instance bodyset :init (make-cascoords) :bodies (list block1 block2 base))
     )
   )
  (:calc-block-set-region
   ()
   (let* ((b0 (send self :make-drc-block-set-one)) ;; dummy
          (b1 (send (make-bounding-box (flatten (send-all (send b0 :bodies) :vertices))) :body)))
     ;; average of x and y
     (/ (+ (- (elt (find-extream (send b1 :vertices) #'(lambda (x) (elt x 0)) #'>) 0)
              (elt (find-extream (send b1 :vertices) #'(lambda (x) (elt x 0)) #'<) 0))
           (- (elt (find-extream (send b1 :vertices) #'(lambda (x) (elt x 1)) #'>) 1)
              (elt (find-extream (send b1 :vertices) #'(lambda (x) (elt x 1)) #'<) 1)))
        2.0)
     ))
  (:make-drc-terrain-block-bodies
   ()
   (let* ((orientation-map (list (list 1 2 3 0 1 2)
                                 (list 2 3 0 1 2 3)
                                 (list 3 0 1 2 3 0)
                                 (list 0 1 2 3 0 1)
                                 (list 1 2 3 0 1 2)
                                 (list 2 3 0 1 2 3)
                                 (list 3 0 1 2 3 0)))
          (height-map (list (list 0 0 0 0 0 0)
                            (list 0 0 1 1 0 0)
                            (list 0 0 1 1 0 0)
                            (list 0 1 1 1 1 0)
                            (list 1 2 1 1 2 1)
                            (list 1 1 1 1 1 1)
                            (list 0 0 0 0 0 0))))
     (let* ((region (send self :calc-block-set-region))
            (block-set-region-x region)
            (block-set-region-y region))
       (dotimes (i (length orientation-map))
         (dotimes (j (length (elt orientation-map i)))
           (let* ((block-set (send self :make-drc-block-set-one)))
             (send block-set :rotate (* (elt (elt orientation-map i) j) pi/2) :z :world)
             (send block-set :translate (float-vector (* i block-set-region-x) (* j block-set-region-y) (* (elt (elt height-map i) j) (elt block-dimensions 2))) :world)
             (push block-set block-bodies)))))
     (setq block-bodies (flatten (send-all block-bodies :bodies)))
     (dolist (b (cdr block-bodies)) (send (car block-bodies) :assoc b))
     block-bodies)
   )
  (:block-bodies () block-bodies)
  (:get-terrain-top-face-from-block-idx
   (block-idx)
   (list (elt (send (elt block-bodies (* 3 block-idx)) :faces) 6)
         (elt (send (elt block-bodies (+ (* 3 block-idx) 1)) :faces) 6))
   )
  (:get-all-terrain-top-faces
   ()
   (let ((ret -1))
     (flatten (append (mapcar #'(lambda (x) (send self :get-terrain-top-face-from-block-idx (incf ret))) (make-list (/ (length block-bodies) 3)))
                      (mapcar #'(lambda (gb) (find-if #'(lambda (x) (memq :top (send x :id))) gb)) (send-all ground-bodies :faces))))))
  )


(defclass drc-surprise-task-shower
  :super cascaded-link
  :slots (bar1-joint bar2-joint)
  )

(defmethod drc-surprise-task-shower
  (:init
   ()
   (prog1
       (send-super :init :name "drc-surprise-task-shower")
     ;; 1. make links links and assoc all links
     (let ((l0 (send self :make-root-link))
           (l1 (send self :make-bar1-link))
           (l2 (send self :make-bar2-link)))
       (send l1 :translate (float-vector (+ -465 190) 600 (+ 1650 190 545)))
       (send l2 :translate (float-vector -465 600 (+ 1650 190 545)))
       ;; 2. assoc links
       ;;    Root link should be associated with "self".
       (send self :assoc l0)
       (send l0 :assoc l1)
       (send l1 :assoc l2)
       ;; 3. make all joints
       ;;    Before making joints, you should :assoc all links.
       (setq bar1-joint (instance rotational-joint :init
                                  :parent-link l0 :child-link l1
                                  :axis :y
                                  :name :bar1-joint
                                  :min -90 :max 90))
       (setq bar2-joint (instance rotational-joint :init
                                  :parent-link l1 :child-link l2
                                  :axis :y
                                  :name :bar2-joint
                                  :min -90 :max 90))
       ;; 4. define slots for robot class
       ;;    links and joint-list for cascaded-link.
       (setq links (list l0 l1 l2))
       (setq joint-list (list bar1-joint bar2-joint))
       ;; 5. call :init-ending after defining links and joint-list and return "self"
       (send self :init-ending)
       self)))
  ;; Methods to define robot links
  (:make-root-link
   ()
   (let* ((b0 (make-cube 10 1330 1590))
          (b1 (make-cube (+ 465 150) 10 10))
          (b2-height (- (+ 1650 190 545) 1590))
          (b2 (make-cube 10 10 b2-height))
          (b3 (make-torus (list (float-vector 0 0 0) (float-vector 0 100 -100) (float-vector 0 0 -100)))))
     (send b0 :translate (float-vector (/ 10 2.0) 0 (/ 1800 2.0)))
     (send b1 :translate (float-vector (/ (+ 465 150) -2.0) 600 (+ 1650 190 545)))
     (send b2 :translate (float-vector (/ 10 -2.0) 600 (+ 1650 190 545 (/ b2-height -2.0))))
     (send b3 :translate (float-vector (+ -465 -150) 600 (+ 1650 190 545)))
     (send b0 :assoc b1)
     (send b0 :assoc b2)
     (send b0 :assoc b3)
     (send b0 :set-color #f(0.8 0.5 0))
     (send b1 :set-color :gray40)
     (send b2 :set-color :gray60)
     (send b3 :set-color :red)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0 b1 b2 b3))
     ))
  (:make-bar1-link
   ()
   (let ((b0 (make-cube 190 10 10)))
     (send b0 :translate (float-vector (/ 190 -2.0) -10 0))
     (send b0 :set-color :gray70)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0))
     ))
  (:make-bar2-link
   ()
   (let ((b0 (make-cube 10 10 545))
         (b1 (make-prism (list (float-vector 0 0 0) (float-vector 100 190 0) (float-vector -100 190 0)) 10)))
     (send b0 :translate (float-vector 0 0 (/ 545 -2.0)))
     (send b1 :rotate -pi/2 :x)
     (send b1 :translate (float-vector 0 0 -545) :world)
     (send b0 :assoc b1)
     (send b0 :set-color :gray60)
     (send b1 :set-color :gray50)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0 b1))
     ))
  )

(defclass drc-surprise-task-button
  :super cascaded-link
  :slots (hinge-joint)
  )

(defmethod drc-surprise-task-button
  (:init
   ()
   (prog1
       (send-super :init :name "drc-surprise-task-button")
     ;; 1. make links links and assoc all links
     (let ((l0 (send self :make-root-link))
           (l1 (send self :make-door-link)))
       (send l1 :translate (float-vector -130 (/ 200 2.0) 1330))
       ;; 2. assoc links
       ;;    Root link should be associated with "self".
       (send self :assoc l0)
       (send l0 :assoc l1)
       ;; 3. make all joints
       ;;    Before making joints, you should :assoc all links.
       (setq hinge-joint (instance rotational-joint :init
                                  :parent-link l0 :child-link l1
                                  :axis :z
                                  :name :hinge-joint
                                  :min -90 :max 90))
       ;; 4. define slots for robot class
       ;;    links and joint-list for cascaded-link.
       (setq links (list l0 l1))
       (setq joint-list (list hinge-joint))
       ;; 5. call :init-ending after defining links and joint-list and return "self"
       (send self :init-ending)
       self)))
  ;; Methods to define robot links
  (:make-root-link
   ()
   (let ((b0 (make-cube 10 1330 1590))
         (b1 (make-cube 130 200 250))
         (b1- (make-cube 50 190 240))
         (b2 (make-cylinder 30 40)))
     (send b1- :translate (float-vector (+ (/ 130 -2.0) (/ 50 2.0) -5) 0 0))
     (send b1- :worldcoords)
     (setq b1 (body- b1 b1-))
     (send b0 :translate (float-vector (/ 10 2.0) 0 (/ 1800 2.0)))
     (send b1 :translate (float-vector (/ 130 -2.0) 0 (+ 1330 (/ 250 2.0))))
     (send b2 :translate (float-vector -105 0 (+ 1330 (/ 250 2.0))) :world)
     (send b2 :rotate -pi/2 :y)
     (send b0 :set-color #f(0.8 0.5 0))
     (send b1 :set-color :gray60)
     (send b2 :set-color :red)
     (send b0 :assoc b1)
     (send b0 :assoc b2)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0 b1 b2))
     ))
  (:make-door-link
   ()
   (let ((b0 (make-cube 20 200 250)))
     (send b0 :translate (float-vector (/ 20 -2.0) (/ 200 -2.0) (/ 250 2)))
     (send b0 :set-color :gray60)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0))
     ))
  )

(defclass drc-surprise-task-lever
  :super cascaded-link
  :slots (lever-joint)
  )

(defmethod drc-surprise-task-lever
  (:init
   ()
   (prog1
       (send-super :init :name "drc-surprise-task-lever")
     ;; 1. make links links and assoc all links
     (let ((l0 (send self :make-root-link))
           (l1 (send self :make-lever-link)))
       (send l1 :translate (float-vector 0 (/ 405 -2.0) (+ 910 455)))
       ;; 2. assoc links
       ;;    Root link should be associated with "self".
       (send self :assoc l0)
       (send l0 :assoc l1)
       ;; 3. make all joints
       ;;    Before making joints, you should :assoc all links.
       (setq lever-joint (instance rotational-joint :init
                                   :parent-link l0 :child-link l1
                                   :axis :y
                                   :name :lever-joint
                                   :min -90 :max 90))
       ;; 4. define slots for robot class
       ;;    links and joint-list for cascaded-link.
       (setq links (list l0 l1))
       (setq joint-list (list lever-joint))
       ;; 5. call :init-ending after defining links and joint-list and return "self"
       (send self :init-ending)
       self)))
  ;; Methods to define robot links
  (:make-root-link
   ()
   (let ((b0 (make-cube 10 1330 1590))
         (b1 (make-cube 150 405 720)))
     (send b0 :translate (float-vector (/ 10 2.0) 0 (/ 1800 2.0)))
     (send b1 :translate (float-vector (/ 150 -2.0) 0 (+ 910 (/ 720 2.0))))
     (send b1 :set-color :gray50)
     (send b0 :set-color #f(0.8 0.5 0))
     (send b0 :assoc b1)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0 b1))
     ))
  (:make-lever-link
   ()
   (let ((b0 (make-cube 230 10 30))
         (b1 (make-cylinder (/ 58 2.0) 10)))
     (send b0 :translate (float-vector (/ 230 -2.0) (/ 10 -2.0) 0))
     (send b1 :translate (float-vector -230 0 0))
     (send b1 :rotate pi/2 :x)
     (send b0 :set-color :gray60)
     (send b1 :set-color :red)
     (send b0 :assoc b1)
     (instance bodyset-link :init (make-cascoords) :bodies (list b0 b1))
     ))
  )

(defclass drc-surprise-task-rope
  :super single-link-model
  :slots ()
  )

(defmethod drc-surprise-task-rope
  (:init
   ()
   (let ((blist (send self :make-surprise-task-rope-bodies)))
     (dolist (b (cdr blist)) (send (car blist) :assoc b))
     (send-super :init :name "drc-surprise-task-rope" :bodies blist)))
  (:make-surprise-task-rope-bodies
   ()
   (let ((b0 (make-cube 10 1330 1590))
         (b1 (make-cube 50 100 50))
         (b2 (make-cube 10 1040 10)))
     (send b0 :translate (float-vector (/ 10 2.0) 0 (/ 1800 2.0)))
     (send b1 :translate (float-vector (/ 50 -2.0) -480 1435))
     (send b2 :translate (float-vector (/ 10 -2.0) 80 1435))
     (send b0 :set-color #f(0.8 0.5 0))
     (send b1 :set-color :blue)
     (send b2 :set-color :red)
     (send b0 :assoc b1)
     (send b0 :assoc b2)
     (list b0 b1 b2)))
  )

;;;;;;;;;;;;;;;;
;; Model generation functions
;;;;;;;;;;;;;;;;
(defun make-drc-stair (&key (add-groud-p))
  (setq *stair* (instance drc-stair :init :add-groud-p add-groud-p))
  )

(defun make-drc-door ()
  "Door on drc"
  (null-output (load "package://drc_task_common/euslisp/test-drc-door-task.l"))
  (setq *door* (instance param-door :init 900
                         60 102 20
                         (float-vector 0 (+ 900 -60 -24) 845)
                         (float-vector -60 (+ 900 -60 -24) 845)
                         :handle-l/r :left))
  )

(defun make-drc-hose-wall ()
  (setq *hose-wall* (instance drc-hose-wall :init))
  )

(defun make-drc-hose-plug ()
  (setq *hose-plug* (instance drc-hose-plug :init))
  )

(defun make-drc-drill-wall ()
  (setq *drill-wall* (instance drc-drill-wall :init))
  )

(defun make-drc-terrain (&key (add-groud-p))
  (setq *terrain* (instance drc-terrain :init :add-groud-p add-groud-p))
  )

(defun make-drc-terrain-japanese-block-ver (&key (add-groud-p))
  (setq *terrain* (instance drc-terrain :init :block-dimensions (list 390 190 150) :add-groud-p add-groud-p :name "drc-terrain-japanese-block-ver"))
  )

(defun make-drc-terrain-japanese-block-ver-sagami (&key (add-groud-p))
  (setq *terrain* (instance drc-terrain :init :block-dimensions (list 390 190 150) :add-groud-p add-groud-p :block-angle 12.5 :name "drc-terrain-japanese-block-ver-sagami"))
  )

(defun make-drc-terrain-test-field-ver (&key (add-groud-p :test-field))
  (setq *terrain* (instance drc-terrain :init :block-dimensions (list 395 195 190) :add-groud-p add-groud-p :block-angle 15 :name "drc-terrain-test-field-ver"))
  )

(defun make-drc-valve ()
  (setq *drc-valve* (instance valve :init :radius (/ 260 2.0) :thick (/ (- 260 205) 2.0) :ring-color :red))
  )

(defun make-drc-surprise-task-shower ()
  (setq *surprise-task-shower* (instance drc-surprise-task-shower :init))
  )

(defun make-drc-surprise-task-button ()
  (setq *surprise-task-button* (instance drc-surprise-task-button :init))
  )

(defun make-drc-surprise-task-lever ()
  (setq *surprise-task-lever* (instance drc-surprise-task-lever :init))
  )

(defun make-drc-surprise-task-rope ()
  (setq *surprise-task-rope* (instance drc-surprise-task-rope :init))
  )

(defun make-drc-surprise-task-box ()
  (make-drc-surprise-task-shower)
  (make-drc-surprise-task-button)
  (make-drc-surprise-task-lever)
  (make-drc-surprise-task-rope)
  (send *surprise-task-shower* :translate (float-vector (/ 1330 -2.0) 0 0))
  (send *surprise-task-lever* :translate (float-vector (/ 1330 2.0) 0 0))
  (send *surprise-task-lever* :rotate pi :z)
  (send *surprise-task-button* :translate (float-vector 0 (/ 1330 -2.0) 0))
  (send *surprise-task-button* :rotate pi/2 :z)
  (send *surprise-task-rope* :translate (float-vector 0 (/ 1330 2.0) 0))
  (send *surprise-task-rope* :rotate -pi/2 :z)
  (list *surprise-task-shower* *surprise-task-button* *surprise-task-lever* *surprise-task-rope*)
  )

;; TODO : This should be scene
(defun make-drc-testbed-models ()
  ;; Generate models
  (make-drc-door)
  (make-drc-hose-plug)
  (make-drc-hose-wall)
  (make-drc-stair)
  (make-drc-drill-wall)
  (make-drc-terrain)
  (make-drc-valve)
  (let ((sb (make-drc-surprise-task-box)))
    (send-all sb :translate (float-vector 12000 0 0) :world))
  ;; Align models
  (send *stair* :translate (float-vector 8000 0 0) :world)
  (send *door* :translate (float-vector 0 0 0) :world)
  (send *hose-wall* :rotate (deg2rad -45) :z)
  (send *hose-wall* :translate (float-vector 4000 -3000 0) :world)
  (send *drill-wall* :rotate -pi/2 :z)
  (send *drill-wall* :translate (float-vector 2000 -3500 0) :world)
  (send *terrain* :translate (float-vector 5000 -1000 0) :world)
  (send *drc-valve* :rotate -pi/2 :y)
  (send *drc-valve* :rotate (deg2rad -135) :x)
  (send *drc-valve* :translate (float-vector 2000 -3500 1130) :world)
  (send *drc-valve* :translate (float-vector 0 (/ 1210 -2.0) (+ -110 -400)))
  ;; Didplay
  (objects (list *hose-plug* *hose-wall* *stair* *door* *drill-wall* *terrain* *drc-valve*
                 *surprise-task-shower* *surprise-task-button* *surprise-task-lever* *surprise-task-rope*))
  )
