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

(defun pr2-pick-can-pose (&key (reset-pose t))
  (when reset-pose
    (send *pr2* :reset-pose))
  (send *pr2* :angle-vector (float-vector 20.1638 39.1793 0.197193 60.086 -109.497 43.8944 -35.3766 217.942 -60.0097 73.9652 -70.0727 -119.982 -20.0107 -30.0023 180.001 -6.4373 43.608))
  )

(provide :pr2-pose)
