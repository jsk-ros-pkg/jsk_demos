(defclass wooden-box :super cascaded-link :slots (parts))

(defmethod wooden-box
  (:init (&key (x-size 598.0) (y-size 480.0) (z-size 710.0)
               (x-label 198.0) (y-label 250.0) (z-label 5.0) (front-to-label 302.0))
    (send-super :init)
    (let* (box0 link0)
      ;; make one box named box0 and marker named sift-label
      (setq box0 (make-cube x-size y-size z-size :name :box0))
      (send box0 :set-color (float-vector 0 1 1))
      (gl::transparent box0 0.8)
      (setq sift-label (make-cube x-label y-label z-label :name :sift-label))
      (send sift-label :set-color (float-vector 1 0 0))
      ;; place one box to ground and sift-label on box
      (send box0 :translate (float-vector 0 0 (/ z-size 2)) :world)
      (send sift-label :translate (float-vector (- front-to-label (/ x-size 2))
                                                0.0
                                                (+ z-size (/ z-label 2))) :world)
      ;; set parts list to assoc others
      (send sift-label :assoc box0)
      (setq parts (list sift-label box0))
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
  (:top-front-right-grasp-edge (&optional (edge-to-right 100.0))
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
  (:top-front-left-grasp-edge (&optional (edge-to-left 100.0))
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
  (:side-front-right-grasp-edge (&optional (top-to-under 80.0))
    "return the coords to grasp side-front-edge with right hand"
    (let* ((x-length (x-of-cube (send self :parts :box0)))
           (y-length (y-of-cube (send self :parts :box0)))
           (z-length (z-of-cube (send self :parts :box0)))
           target-coords)
      (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                :translate (float-vector (- (/ x-length 2)) (- (/ y-length 2)) (- (/ z-length 2) top-to-under))))
      ))
  ;; return the coords to grasp side-front-edge with left hand
  (:side-front-left-grasp-edge (&optional (top-to-under 80.0))
    "return the coords to grasp side-front-edge with left hand"
    (let* ((x-length (x-of-cube (send self :parts :box0)))
           (y-length (y-of-cube (send self :parts :box0)))
           (z-length (z-of-cube (send self :parts :box0)))
           target-coords)
      (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                :translate (float-vector (- (/ x-length 2)) (/ y-length 2) (- (/ z-length 2) top-to-under))))
      ))
  ;; return the coords of box-attached label for sift recog
  (:sift-label-coords (&optional (front-to-label 302.0))
    "return the coords of box-attached label for sift recog"
    (let* ((x-length (x-of-cube (send self :parts :box0)))
           (z-length (z-of-cube (send self :parts :box0)))
           target-coords)
      (setq target-coords (send (send (send (send self :parts :box0) :worldcoords) :copy-worldcoords)
                                :translate (float-vector (- front-to-label (/ x-length 2)) 0 (/ z-length 2))))
      ))
  )

(defun make-wooden-box ()
  (setq *wooden-box* (instance wooden-box :init))
  )
