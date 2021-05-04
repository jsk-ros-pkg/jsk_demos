;; /hit_volume_threshold: 0.09

;; NOTE move fetch's hand vertical to mokume

(send *ri* :angle-vector (send *fetch* :angle-vector #f(48.096 15.0535 -30.9713 -94.04 32.7146 -284.882 117.751 247.523 0.305897 15.7652)) 10000)
(send *ri* :wait-interpolation)

;; horizontal
(dotimes (i 3)
  (send *ri* :angle-vector (send *fetch* :rarm :move-end-pos #f(0 50 0) :world :use-torso nil) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *fetch* :rarm :move-end-pos #f(0 -50 0) :world :use-torso nil) 3000)
  (send *ri* :wait-interpolation)
  (unix:usleep 1)
  )

;; vertical
(dotimes (i 3)
  (send *ri* :angle-vector (send *fetch* :rarm :move-end-pos #f(-50 0 -10) :world :use-torso nil) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *fetch* :rarm :move-end-pos #f(50 0 10) :world :use-torso nil) 3000)
  (send *ri* :wait-interpolation)
  (unix:usleep 1)
  )
