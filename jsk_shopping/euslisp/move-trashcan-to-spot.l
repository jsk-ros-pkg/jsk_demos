(load "package://jsk_spot_startup/euslisp/navigation-utils.l")


(defun move-to-spot-front ()
  (send *ri* :go-pos-unsafe 0 0 -90))
