(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_maps/src/eng2-scene.l")

(defun init ()
  (fetch-init)
  ;;(send *ri* :angle-vector (send *fetch* :reset-pose))
;;  (send *ri* :angle-vector #f(23.102 57.2229 -47.2485 -75.0006 87.4081 -147.508 20.6889 -154.0 -4.25995 13.6359))
  ;;(send *ri* :wait-interpolation)
  (objects (list *fetch*))
  (if (not (boundp '*eng2-scene*))
      (setq *eng2-scene* (make-eng2-scene)))
  (setq *7f* (car (member-if #'(lambda (x) (string= (send x :name) "/eng2/7f")) (send *eng2-scene* :floors))))
  (objects (list *fetch* *7f*))
  (send *fetch* :move-to (send (send *7f* :spot "/eng2/7f/room73B2-table-side1") :copy-worldcoords))
  (send *irtviewer* :look-all (list *fetch*))
  (ros::set-param "/box_number/red" 0.0)  
  (ros::set-param "/box_number/green" 0.0)  
  (ros::set-param "/box_number/blue" 0.0)  
)
-
(defun reset-pose nil
    (send *fetch* :angle-vector #f(220.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)


(in-package "GL")
(defmethod glviewsurface
  (:string
   (x y str &optional (fid x:font-courb24)) ;;x::font-helvetica-bold-12))                                                               
   (send self :makecurrent)
   (glMatrixMode GL_PROJECTION)
   (glPushMatrix)
   (send self :2d-mode)
   (unless (eq (get self :glxusexfont) fid)
     (setf (get self :glxusexfont) fid)
     (glxUseXfont fid 32 96 (+ 1000 32)))
   (glRasterPos2i (round x) (- (send self :height) (round y)))
   (glListBase 1000)
   (glCallLists (length str) GL_UNSIGNED_BYTE str)
   (send self :3d-mode)
   (glMatrixMode GL_PROJECTION)
   (glPopMatrix)
   (glMatrixMode GL_MODELVIEW)
   ))
(in-package "USER")

(defun show-spots ()
    (print "hit return to stop...")
    (do-until-key
     (dolist ( s (send  *eng2-scene* :spots))
       (let (p)
	 (setq p (send *irtviewer* :viewer :point-to-screen (send  s :worldpos)))
	 (send *irtviewer* :viewer :viewsurface :string (elt p 0) (elt p 1) (send s :name) x::font-courb12)
	 (send *irtviewer* :viewer :viewsurface :glflush)
	 (x::window-main-one))
       )
     )
  )



;;default view to watch robot
(defun dv()
    (send *irtviewer* :look-all *fetch*)
)
