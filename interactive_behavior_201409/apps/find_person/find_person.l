#!/usr/bin/env roseus
;; find_person.l

(require :speak "package://pr2eus/speak.l")

(ros::roseus "run_find_person")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)
;; *param*: ((person . okada))

(let (place)
  (case (intern (string-upcase (cdr (assoc "person" *param* :test #'string=))))
    ('asano (setq place "73B1"))
    ('otsubo (setq place "73B2"))
    ('furushchev (setq place "73A3"))
    ('okada (setq place "73A2"))
    ('inaba (setq place "73A1"))
    (t t))

  (if place
    (speak-jp (format nil "~Aにいます。" place))
    (speak-jp "すみません、わかりません。")))


(exit)
