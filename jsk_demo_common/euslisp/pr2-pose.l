(defun pr2-pick-tray-pose (&key (torso-height 50) (arm :arms))
  (when (memq arm '(:arms :rarm))
    (send *pr2* :rarm :angle-vector
          (float-vector -75.0013 -20.0023 -69.9992 -120.306 -20.0041 -29.6546 179.999)))
  (when (memq arm '(:arms :larm))
    (send *pr2* :larm :angle-vector
          (float-vector 75.0006 -20.0 69.9995 -119.999 20.0009 -29.9976 180.001)))
  (send *pr2* :torso :waist-z :joint-angle torso-height)
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *pr2* :head :neck-y :joint-angle 0)
  (send *pr2* :angle-vector))

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
