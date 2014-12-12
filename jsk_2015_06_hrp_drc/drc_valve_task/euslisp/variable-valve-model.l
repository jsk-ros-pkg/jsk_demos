
(defclass variable-valve
  :super cascaded-link
  :slots (handle valve-joint)
  )

(defmethod variable-valve
  (:init
   (&rest args
          &key (thick 20) (radius 115)
          (ring-color :gray50) (base-color :royalblue)
          (type :round))
   (send-super* :init args)
   (setq handle nil)
   ;; 1. make links links and assoc all links
   (let ((rl (send self :make-root-link
                   :base-color base-color))
         (vl (send self :make-valve-link
                   :thick thick :radius radius :ring-color ring-color :type type)))
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
   (&key base-color)
   (let (base-cylinder
         (r 90)
         (L 240)
         (H 100)
         (thick 30)
         axis-cylinder
         rl)
     (setq axis-cylinder (make-cylinder 20 H))
     (setq rl (instance bodyset-link :init (make-cascoords)
                        :bodies (list axis-cylinder)
                        :name :crank-root-link))
     (send axis-cylinder :translate (float-vector 0 0 (- H)) :world)  ;;vavle's hight (H)
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
                 (cross-bar1 (make-cube pipe-radius (* ring-radius 2) pipe-radius ))
                 (cross-bar2 (make-cube (* ring-radius 2) pipe-radius pipe-radius )))
            (send ring :assoc cross-bar1)
            (send ring :assoc cross-bar2)
            (let ((valve-link (instance bodyset-link :init (make-cascoords)
                                        :bodies (list ring cross-bar1 cross-bar2) :name :valve-handle-link)))
              ;;
              (push (make-cascoords :coords 
                                    (send
                                     (send 
                                      (send (send ring :copy-worldcoords) :translate (float-vector 0 radius 0))
                                      :rotate pi/2 :y :local)
                                     :rotate pi :x :local)
                                    :name :valve-handle) handle)
              (send valve-link :assoc (car handle))
              (dolist (l (send valve-link :bodies))
                (send l :set-color ring-color))
              valve-link)))
         ((equal type :bar)
          (let* ((segments 16)
                 (bar-thick 13.5)
                 (bar-length 390)
                 (bar-root-thick 23.5)
                 (bar-root-length 100)
                 (bar (make-cylinder bar-thick bar-length :segments segments))
                 (bar-root (make-cylinder bar-root-thick bar-root-length :segments segments)))
            (send bar-root :assoc bar)
            (send bar-root :rotate pi/2 :x :world)
            (send bar-root :translate (float-vector 0 (/ bar-root-length 2.0) 0) :world)
            (let ((valve-link (instance bodyset-link :init (make-cascoords)
                                        :bodies (list bar-root bar) :name :valve-handle-link))
                  (valve-handle
                   (make-cascoords :coords
                                   (send
                                    (send bar-root :copy-worldcoords)
                                    :translate (float-vector 0 0 0))
                                   :name :valve-handle)))
              (push valve-handle handle)
              (send valve-link :assoc (car handle))
              (dolist (l (send valve-link :bodies))
                (send l :set-color ring-color))
              valve-link))))
   )
  (:handle (&rest args) (forward-message-to-all handle args))
  (:handle-valve-handle () (car handle))
  (:valve-joint (&rest args) (forward-message-to valve-joint args))
  )
