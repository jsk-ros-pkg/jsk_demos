#!/usr/bin/env roseus

;; dummy files
(defun move-to (&rest args)
  (warning-message 1 "move-to ~A" args)
  (read-char))

