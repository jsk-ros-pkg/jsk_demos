(defun pr2-pick-tray-pose (&key (reset-pose))
  ;; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)

  ;; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)

  ;; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)

  (send *pr2* :head :neck-p :joint-angle 50)
  )
