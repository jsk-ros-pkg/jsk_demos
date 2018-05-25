#!/usr/bin/env roseus
;; find_person.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :speak "package://pr2eus/speak.l")

(defun main ()
  (let* ((person (cdr (assoc "person" *param* :test #'string=)))
         (place (person->place person)))
    (if place
        (speak-jp (format nil "~Aにいます。" place))
        (speak-jp "すみません、わかりません。"))))

(if (main) (exit 0) (exit 1))
