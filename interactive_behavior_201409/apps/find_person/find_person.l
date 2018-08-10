#!/usr/bin/env roseus
;; find_person.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :speak "package://pr2eus/speak.l")

(defun main ()
  (let* ((person (get-arg :person))
         (place (person->place person)))
    (if place
        (speak-jp (format nil "~Aにいます。" place))
        (speak-jp "すみません、わかりません。"))))

(run-exit (main))