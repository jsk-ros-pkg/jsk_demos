;; fetchに挨拶をさせるデモ。ロードして(wave-hands)を実行すれば出来る。


(defun first-pose()
  (send *fetch* :angle-vector #f(0.0 -60.0 10.0 0.0 -110.0 -90.0 0.0 0.0 0.0 0.0)))

(defun wave-hands()
  (progn
    (send *fetch* :init-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (send *fetch* :angle-vector #f(0.0 -60.0 10.0 0.0 -110.0 -90.0 0.0 0.0 0.0 0.0))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :elbow-p :joint-angle -60)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :elbow-p :joint-angle -120)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
    (send *ri* :speak-jp "こんにちは"))
