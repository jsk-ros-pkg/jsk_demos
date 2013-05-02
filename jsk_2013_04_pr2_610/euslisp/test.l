#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(pr2)
(setq *ri* (instance pr2-interface :init))

(pr2-reset-pose)
(setq pose1  #f(49.9542 47.7996 25.9635 50.0418 -121.521 1.54531 -41.8865 -180.0 -60.0012 74.0017 -70.0009 -120.0 -19.9994 -29.9995 -180.0 20.1722 54.5159))
(setq pose2 #f(49.9491 26.7269 -13.1397 32.8162 -37.719 73.9471 -61.6853 -140.619 -60.0014 73.9984 -69.999 -120.0 -20.0002 -29.9998 -180.0 18.5228 15.5653))

(defun p1 ()
  (send *ri* :angle-vector pose1)
  (send *ri* :wait-interpolation))

(defun p2 ()
  (send *ri* :angle-vector pose2) ;; ここで何回転かする
  (send *ri* :wait-interpolation))

(p1)
(unix:sleep 3)
(p2)
(unix:sleep 10)

(pr2-reset-pose)
(send *ri* :angle-vector-sequence (list pose1 pose2) (list 3000 3000))
(send *ri* :wait-interpolation)