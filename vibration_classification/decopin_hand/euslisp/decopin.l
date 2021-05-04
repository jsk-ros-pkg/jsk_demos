(defclass decopin
  :super cascaded-link
  :slots (reset-pose-)
  )

(defmethod decopin
  (:init
   (&aux
    (joint-names
     (list "finger1" "finger2" "finger3" "rotational_joint"))
    )
   (send-super :init :name "decopin")
   (dotimes (i 5)
     (push
      (instance bodyset-link :init
                (make-cascoords)
                :bodies (list (make-cube 1 1 1)) :name (format nil "link~a" i) :weight 1.0)
      links)
     )
   (dotimes (i 4)
     (push
      (instance rotational-joint :init
                :parent-link (elt links i)
                :child-link (elt links (1+ i))
                :name (elt joint-names i)
                :axis :z :min -1e10 :max 1e10)
      joint-list)
     )
   (send self :init-ending)
   self
   )
  (:finger1
   ()
   (find-if #'(lambda (j) (equal (send j :name) "finger1")) joint-list)
   )
  (:finger2
   ()
   (find-if #'(lambda (j) (equal (send j :name) "finger2")) joint-list)
   )
  (:finger3
   ()
   (find-if #'(lambda (j) (equal (send j :name) "finger3")) joint-list)
   )
  (:rotational_joint
   ()
   (find-if #'(lambda (j) (equal (send j :name) "rotational_joint")) joint-list)
   )
  ;; functions below are pre-defined pose of the hand.
  ;; only :reset-pose is needed to be overwrite when the wire is re-rolled. (others are relative)
  (:set-reset-pose (av)
    (setq reset-pose- av))
  (:reset-pose ()
    (if (eq reset-pose- nil)
      (progn (format t "call (set-reset-pose)~%")
             (return-from :reset-pose))
      (send self :angle-vector reset-pose-)))
  (:open ()
    (send self :reset-pose)
    (send self :angle-vector
          (v+ (send self :angle-vector) #f(0 80 -80 80))))
  (:pinch ()
    (send self :reset-pose)
    (send self :angle-vector
          (v+ (send self :angle-vector) #f(0 0 -110 110))))
  (:pinch-left ()
    (send self :reset-pose)
    (send self :angle-vector
          (v+ (send self :angle-vector) #f(0 0 -120 100))))
  (:pinch-right ()
    (send self :reset-pose)
    (send self :angle-vector
          (v+ (send self :angle-vector) #f(0 0 -100 120))))
  )
