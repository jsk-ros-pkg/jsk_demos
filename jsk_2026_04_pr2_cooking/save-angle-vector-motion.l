#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(format t "save file name: ")
(finish-output)
(setq filename (read-line))
(setq save-path (concatenate string "angle_vector/" filename ".l"))

(setq av-list nil)

(setq fp (open save-path :direction :output))

(format fp "(setq ~A '(" filename)
(finish-output fp)

(do-until-key
  (setq av (copy-object (send *ri* :state :potentio-vector)))
  (format fp "~s~%" av)
  (finish-output fp)
  (unix::usleep 10000))

(format fp "))~%")
(finish-output fp)
(close fp)

(format t "Saved to ~A~%" save-path)

