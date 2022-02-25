#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)
(send *ri* :speak-jp "7 3A3" :wait t)

(defun speak ()
  (unix::sleep 10)
  (send *ri* :speak-jp "73A3に向かってください" :wait t)
  )
