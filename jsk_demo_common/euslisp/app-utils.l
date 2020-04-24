#!/usr/bin/env roseus
;; app-utils.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :app-utils)

(defun get-arg (key)
  (unless (stringp key)
    (setq key (string-downcase key)))
  (let ((val (cdr (assoc key (ros::get-param "/action/parameters") :test #'string=))))
    (if (or (null val) (null-string-p val)) nil (read-from-string val))))

(defmacro run-exit (&rest form)
  `(if (find-if #'(lambda (s) (substringp "__log:=" s)) lisp::*eustop-argument*)
     (if (progn ,@form) (exit 0) (exit 1))
     (warn ";; run ~A~%" '(progn ,@form))))
