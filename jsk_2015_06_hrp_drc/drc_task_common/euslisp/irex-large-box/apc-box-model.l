(defclass apc-box :super cascaded-link :slots (parts))

(defmethod apc-box
  (:init (&key (x-size 335.0) (y-size 605.0) (z-size 600.0))
         (send-super :init)
         (let* (box0 link0)
           ;; make one box named box0
           (setq box0 (make-cube x-size y-size z-size :name :box0))
           (send box0 :set-color (float-vector 0 1 1))
           ;; place one box to ground
           (send box0 :translate (float-vector 0 0 (/ z-size 2)) :world)
           ;; set parts list to assoc other box
           (setq parts (list box0))
           ;; make one link named link0
           (setq link0 (instance bodyset-link :init (make-cascoords) :bodies parts))
           ;; assoc root link to self
           (send self :assoc link0)
           ;; sustitute links to the "links" slots in cascaded-link
           (setq links (list link0))
           ;; end of init
           (send self :init-ending)
           self))
  ;; return the cube of given name
  (:parts (name)
          (find name parts :test #'equal :key #'(lambda (x) (send x :name)))
          )
  ;; return the coords to grasp top-front-edge with right hand
  (:top-front-right-grasp-edge (&optional (edge-to-right 80))
                               "return the coords to grasp top-front-edge with right hand"
                               (let* ((x-length (x-of-cube (send self :parts :box0)))
                                      (y-length (y-of-cube (send self :parts :box0)))
                                      (z-length (z-of-cube (send self :parts :box0)))
                                      target-coords)
                                 (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                                           :translate (float-vector (- (/ x-length 2)) (- edge-to-right (/ y-length 2)) (/ z-length 2))))
                                 (send target-coords :rotate (deg2rad -90) :x :world)
                                 ))
  ;; return the coords to grasp top-front-edge with left hand
  (:top-front-left-grasp-edge (&optional (edge-to-left 80))
                              "return the coords to grasp top-front-edge with left hand"
                              (let* ((x-length (x-of-cube (send self :parts :box0)))
                                     (y-length (y-of-cube (send self :parts :box0)))
                                     (z-length (z-of-cube (send self :parts :box0)))
                                     target-coords)
                                (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                                          :translate (float-vector (- (/ x-length 2)) (- (/ y-length 2) edge-to-left) (/ z-length 2))))
                                (send target-coords :rotate (deg2rad 90) :x :world)
                                ))
  ;; return the coords to grasp side-front-edge with right hand
  (:side-front-right-grasp-edge (&optional (top-to-under 80))
                     "return the coords to grasp side-front-edge with right hand"
                     (let* ((x-length (x-of-cube (send self :parts :box0)))
                            (y-length (y-of-cube (send self :parts :box0)))
                            (z-length (z-of-cube (send self :parts :box0)))
                            target-coords)
                       (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                                 :translate (float-vector (- (/ x-length 2)) (- (/ y-length 2)) (- (/ z-length 2) top-to-under))))
                       ))
  ;; return the coords to grasp side-front-edge with left hand
  (:side-front-left-grasp-edge (&optional (top-to-under 80))
                    "return the coords to grasp side-front-edge with left hand"
                    (let* ((x-length (x-of-cube (send self :parts :box0)))
                           (y-length (y-of-cube (send self :parts :box0)))
                           (z-length (z-of-cube (send self :parts :box0)))
                           target-coords)
                      (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                                :translate (float-vector (- (/ x-length 2)) (/ y-length 2) (- (/ z-length 2) top-to-under))))
                      ))
  )

(defun make-apc-box ()
  (setq *apc-box* (instance apc-box :init))
  )
