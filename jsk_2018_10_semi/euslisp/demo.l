#!/usr/bin/env roseus

;;
(load "init.l")
(load "kawamura-pose-functions.l")
(load "kawamura-move-functions.l")
;; load demo codes, all init code is written in init.l and (init)
;; (load "say-hello.l")

;; call functions
(init)

;;kawamura -->
(goto-tableside)
(give-hand)
(release)
;;wait 3 sec
(unix:sleep 3)
(grab)
;; return arm to default position
(dpos) 
(goto-73A2)
;; <--


;; (say-hello)

