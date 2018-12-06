(setq look-down-pose #f(386.15 75.6359 80.1929 -11.4118 98.5702 0.060285 95.1322 -0.060071 -0.895055 42.9474));;見下ろすポーズ

(defun goto-kitchenside nil
  (send *ri* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
			       :transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))))
  (send *fetch* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
			       :transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))) :world)
  (send *ri* :angle-vector look-down-pose 5000)
  (send *fetch* :angle-vector look-down-pose)
  (send *irtviewer* :look-all *fetch*)
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
