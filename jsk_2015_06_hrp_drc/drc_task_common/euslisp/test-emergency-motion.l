(load "package://drc_task_common/euslisp/robot-util.l")
(warn "(generate-emergency-motion-for-jaxon)~%")
(defun generate-emergency-motion-for-jaxon (&key (initial-pose nil))
  (if (not (boundp '*robot*))
      (progn
        (load (robot-file "jaxon"))
        (setq *robot* (jaxon))))
  ;; initial pose
  (when initial-pose
    (y-or-n-p ";; initial pose? (reset-pose)~%")
    (progn
      (send *robot* :reset-pose)
      (send *robot* :fix-leg-to-coords (make-coords)))
    (send *irtviewer* :draw-objects)
    (read-line))
  ;; squat pose
  (y-or-n-p ";; squat pose?~%")
  (progn
    (send *robot* :legs :move-end-pos #f(0 0 500))
    (send *robot* :torso-waist-p :joint-angle 30)
    (dolist (l (list :rarm :larm))
      (send *robot* l :inverse-kinematics (send (make-coords :pos (send (send (send *robot* l :end-coords) :copy-worldcoords) :worldpos)) :rotate pi/2 :y)))
    (send *robot* :inverse-kinematics
          (list (send (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 150 0 500) :local) :rotate (/ 0 6) :y :world)
                (send (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :translate (float-vector 150 0 500) :local) :rotate (/ 0 6) :y :world))
          :move-target (list (send *robot* :rarm :end-coords)
                             (send *robot* :larm :end-coords))
          :link-list (list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
                           (send *robot* :link-list (send *robot* :larm :end-coords :parent))))
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg :larm :rarm)
          :root-link-virtual-joint-weight #f(0.1 0.1 0.8 0.1 0.5 0.5)
          :target-centroid-pos (v+ (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos)) (float-vector 20 0 0)))
    (send *irtviewer* :draw-objects)
    (read-line))
  ;; move centroid forward
  (y-or-n-p ";; centroid forward?~%")
  (progn
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg :larm :rarm) :target-centroid-pos (float-vector 30 0 0))
    (read-line))
  ;; four leg mode
  (y-or-n-p ";; four leg mode?~%")
  (let* ((z-pos (send-all (send *robot* :arms :end-coords) :worldpos))
         (tc
          (list
           (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (+ (* -1 (elt (car z-pos) 2)) 15)) :world)
           (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (+ (* -1 (elt (cadr z-pos) 2)) 15)) :world)
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (dolist (l (list :rleg :lleg))
      (send *robot* l :crotch-p :joint-angle -90))
    (send *robot* :fix-leg-to-coords (make-coords))
    (with-move-target-link-list
     (mt ll *robot* '(:rarm :larm :rleg :lleg))
     (send *robot*
           :fullbody-inverse-kinematics tc
           :move-target mt
           :link-list ll
           :max (float-vector +500 +200 +200 +10 +20 +10)
           :min (float-vector -500 -200 -200 -10 -20 -10)
           :root-link-virtual-joint-weight #f(0.8 0.1 0.8 0.1 0.8 0.5)
           :target-centroid-pos (v+ (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos)) (float-vector 30 0 0))
           :debug-view :no-message))
    (send *irtviewer* :draw-objects)
    (read-line))
  ;; move centroid
  (y-or-n-p ";; move centroid?~%")
  (progn
    (send *robot* :move-centroid-on-foot :both '(:rarm :larm :rleg :lleg)
          :root-link-virtual-joint-weight #f(0.8 0.1 0.8 0.1 0.8 0.5)
          :max (float-vector +800 +600 +800 +20 +20 +10)
          :min (float-vector -500 -600 -200 -20 -20 -10)
          :target-centroid-pos (v+ (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
                                   (float-vector 150 0 0))
          :centroid-thre 20
          :debug-view :no-message
          ;; :stop 400
          )
    (send *irtviewer* :draw-objects)
    (read-line))
  )
