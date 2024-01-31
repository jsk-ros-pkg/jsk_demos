(load "package://spotkinovaeus/spotkinova.l")

(unless (boundp '*spotkinova*)
  (spotkinova :type :gen3_lite_gen3_lite_2f)
  (objects (list *spotkinova*)))

(setq *move-target*
      (list (send *spotkinova* :head :end-coords)
            (send *spotkinova* :larm :end-coords)
            (send *spotkinova* :rarm :end-coords)
            (send *spotkinova* :lleg :end-coords)
            (send *spotkinova* :rleg :end-coords)))
(setq *move-coords*
      (list (send (send (send *spotkinova* :head :end-coords) :copy-worldcoords) :translate #f(200 0 150) :world)
            (send (send (send *spotkinova* :larm :end-coords) :copy-worldcoords) :translate #f(0 0 0) :world)
            (send (send (send *spotkinova* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 0) :world)
            (send (send (send *spotkinova* :lleg :end-coords) :copy-worldcoords) :translate #f(0 0 0) :world)
            (send (send (send *spotkinova* :rleg :end-coords) :copy-worldcoords) :translate #f(0 0 0) :world)))

(defun reset-pose ()
  ;; Reset position and pose
  (send *spotkinova* :move-to (make-coords) :world)
  (send *spotkinova* :reset-pose))

(defun ik-pose ()
  (reset-pose)
  ;; Sample IK pose
  (send *spotkinova*
        :fullbody-inverse-kinematics *move-coords*
        :move-target *move-target*
        :link-list (mapcar #'(lambda (limb) (send *spotkinova* :link-list (send limb :parent))) *move-target*)
        :min (float-vector -500 -500 -500 -20 -20 -10)
        :max (float-vector 500 500 500 -20 -20 -10)
        :rotation-axis (list t nil nil nil nil)
        :target-centroid-pos nil))

(defun visualize ()
  ;; Visualize target coords and origin coords
  (send *irtviewer* :viewer :viewing :look #f(35 6280 727) #f(257 -83 58) #f(0 0 1))
  (send *irtviewer* :draw-objects :flush t)
  (mapcar #'(lambda (x) (send x :draw-on :flush t :width 3 :size 200 :color #f(0 1 0)))
          *move-coords*)
  (send (make-coords) :draw-on :flush t :width 3 :size 200 :color #f(1 0 0))
  ;; Print robot position and pose
  (format t "Robot worldcoords: ~A~%" (send *spotkinova* :worldcoords)))
