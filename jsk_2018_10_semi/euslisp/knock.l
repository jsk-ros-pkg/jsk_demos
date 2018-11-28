(defun knock nil

  (send *fetch* :torso :waist-z :joint-angle 200)
  (setq av0 (send *fetch* :angle-vector))

  (send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 0 700))) :debug-view t)
  (setq av1 (send *fetch* :angle-vector))

  (send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(620 0 700))) :debug-view t)
  (setq av2 (send *fetch* :angle-vector))

  (send *ri* :angle-vector-sequence (list av0 av1 av2 av1 av2 av1) (list 5000 3000 3000 3000 3000 3000))

  (send *ri* :wait-interpolation)
  ;(send *fetch* :reset-pose)

  ;(send *ri* :angle-vector (send *fetch* :angle-vector)) 
  )

(defun neck-reset nil
  (send *fetch* :head :neck-y :joint-angle 0)
  (send *fetch* :head :neck-p :joint-angle 0)
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)
  )

;;(send *fetch* :torso :waist-z :joint-angle 0)
;;(send *fetch* :rarm :shoulder-y :joint-angle -90.0)
;;(send *fetch* :rarm :shoulder-p :joint-angle -20.0)
;;(send *fetch* :rarm :shoulder-r :joint-angle 0.0)
;;(send *fetch* :rarm :elbow-p :joint-angle 50.0)
;;(send *fetch* :rarm :elbow-r :joint-angle -60.0)
;;(send *fetch* :rarm :wrist-p :joint-angle 10.0)
;;(send *fetch* :rarm :wrist-r :joint-angle 0.0)
;;(send *fetch* :head :neck-y :joint-angle 0.0)
;;(send *fetch* :head :neck-p :joint-angle 0.0)

;(send *ri* :angle-vector (send *fetch* :angle-vector))
