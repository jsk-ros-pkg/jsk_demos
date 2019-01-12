(defun pr2-pick-tray-pose (&key (reset-pose) (torso-height 50))
  (when reset-pose
    (send *pr2* :reset-pose))
  (send *pr2* :torso_lift_joint :joint-angle torso-height)

  ;; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)

  ;; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)

  ;; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)

  (send *pr2* :head :neck-p :joint-angle 50)
  (send *pr2* :angle-vector)
  )

(defun pr2-grasp-can-pose (arm &key (torso))
  (unless torso (setq torso (send *pr2* :torso :waist-z :joint-angle)))
  (case arm
    (:rarm (send *pr2* :larm :angle-vector (get-side-pose :larm)))
    (:larm (send *pr2* :rarm :angle-vector (get-side-pose :rarm))))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (when torso
    (send *pr2* :torso :waist-z :joint-angle torso))
  (case arm
    (:larm (send *pr2* :larm :angle-vector (float-vector -29.4658 33.1466 -9.09702 -120.928 -152.556 -82.9388 -173.681)))
    (:rarm (send *pr2* :rarm :angle-vector (float-vector 31.3839 25.5029 23.0531 -118.916 160.305 -84.1469 160.058))))
  (send *pr2* :head :look-at (send (send *pr2* arm :end-coords) :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector))

(provide :pr2-pose)
