#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(setq av (send *ri* :state :angle-vector))
;; initial pose
;; (setq av #f(50.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0))
(send *pr2* :angle-vector av)
(objects (list *pr2*))

(format t "save file name: ")
(finish-output)
(setq filename (read-line))

;; 保存パスを指定
(setq save-path (concatenate string "angle_vector/" filename ".l"))

;; ファイルに保存
(setq fp (open save-path :direction :output))
(format fp "(setq ~A '~s)~%" filename av)
(close fp)

(format t "Saved current angle-vector to ~A~%" save-path)
(format t "angle-vector:~%~s~%" av)
