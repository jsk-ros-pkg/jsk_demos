(load "package://jsk_2025_10_semi/xlerobot-utils.l")
(if (not (boundp '*xlerobot*)) (xlerobot))
(send *xlerobot* :reset-pose)
(setq *e* (send *xlerobot* :larm :end-coords :copy-worldcoords))
(objects (list *xlerobot*))

(send *xlerobot* :larm :angle-vector #f(0 0 0 0 0 0))
(objects (list *xlerobot*))

(send *xlerobot* :larm :shoulder-y :joint-angle -10)
(send *xlerobot* :larm :shoulder-p :joint-angle 30)
(send *xlerobot* :larm :elbow-p :joint-angle 60)
(send *xlerobot* :larm :wrist-p :joint-angle -30)
(send *xlerobot* :larm :wrist-r :joint-angle 90)
(send *xlerobot* :larm :gripper :joint-angle 45)
(send *xlerobot* :head :neck-p :joint-angle 10)
(send *xlerobot* :head :neck-y :joint-angle 30)

(dotimes (i 50)
  (send *xlerobot* :larm :move-end-pos #f(0 0 2))
  (send *irtviewer* :draw-objects))
(dotimes (i 50)
  (send *xlerobot* :larm :move-end-pos #f(-2 0 0))
  (send *irtviewer* :draw-objects))

(setq count 0)
(setq gain 50)
(do-until-key
  (setq c (send *e* :copy-worldcoords))
  (send c :translate (float-vector (* gain (cos count)) (* gain (sin count)) 0))
  (incf count 0.1)
  ;; do ik
  (send *xlerobot* :larm :inverse-kinematics c :rotation-axis :z)
  
  ;; refresh
  ;;(send c :draw-on :flush t :size 100)
  (send *irtviewer* :draw-objects)
  (x::window-main-one))
