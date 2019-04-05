#!/usr/bin/env roseus
;; find_object.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(require :speak "package://pr2eus/speak.l")

(defun main ()
  (speak-jp "すみません、わかりません。"))

(run-exit (main))
