(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(load "package://jsk_maps/src/eng2-scene.l")
(setq s (make-eng2-scene))
(objects (list s *fetch*))
(print (mapcar #'(lambda (x) (send x :name)) (send s :spots)))


;;テーブルの横に帰る関数
(defun go-home ()
  (setq table_side (send (send s :spot "/eng2/7f/room73B2-table-side1")
                         :copy-worldcoords))
  (send *fetch* :move-to table_side :world)
  (send *ri* :move-to table_side)
  (send *ri* :wait-interpolation)
  ;; move camera to fetch
  (send *irtviewer* :look-all *fetch*)
  )

;;給湯室へ移動する関数
(defun move3 ()
  (setq kyuutou (send (send (send
                     s :spot "/eng2/7f/room73A3-front1") :copy-worldcoords)
              :transform (make-coords :pos #f(-2050 -13200 0)
                                      :rpy (float-vector 1.570 0 0))))
  (send *ri* :move-to kyuutou)
  (send *fetch* :move-to kyuutou :world)
  (send *irtviewer* :look-all *fetch*)
)

;;右腕のみで逆運動学を行う関数
(defun motion-arm (x y z)
   (send *fetch* :rarm :inverse-kinematics
         (send (send *fetch* :copy-worldcoords)
               :transform (make-coords :pos
                                       (float-vector x y z)))
         :revert-if-fail t)
   (send *ri* :angle-vector (send *fetch* :angle-vector))
   (send *irtviewer* :look-all *fetch*)
   )

;;腕を元に戻す関数(関数化する意味ある？)
(defun reset-pose ()
  (send *fetch* :angle-vector
        #f(20.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0))
  (send *ri* :angle-vector
        #f(20.0 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.0 0.0))
  (send *ri* :wait-interpolation)
  )


;; 給湯室へ行って腕を伸ばす
(go-home)
(move3)
(send *irtviewer* :look-all *fetch*)


(motion-arm 900 200 400)
(send *irtviewer* :look-all *fetch*)

;;何かをつかむ
(send *fetch* :start-grasp)
(send *fetch* :stop-grasp)

;;腕をしまって帰る
(reset-pose)
(go-home)


(in-package "GL")
(defmethod glviewsurface
  (:string
   (x y str &optional (fid x:font-courb24))
   ;;x::font-helvetica-bold-12))
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
(print "hit return to stop...")
(do-until-key
 (dolist (s (send s :spots))
   (let (p)
     (send *irtviewer* :draw-string (send s :worldcoords) (send s :name))
     (setq p (send *irtviewer* :viewer :point-to-screen (send s :worldpos)))
     (send *irtviewer* :viewer :viewsurface :string
           (elt p 0) (elt p 1) (send s :name) x::font-courb12)
     (objects (list s *fetch*))
     (send *irtviewer* :viewer :viewsurface :glflush)
     (x::window-main-one))
