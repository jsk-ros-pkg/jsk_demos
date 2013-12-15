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

(provide :pr2-pose)