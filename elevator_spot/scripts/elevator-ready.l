#!/usr/bin/env roseus
(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

(setq *elevator-ready* t)

(defun get-on()
  ;; Expected the Spot stops in 1.5m front of the elvator.
  (check-elevator-open "front")
  ;; change to crawl mode
  (send *ri* :set-locomotion-mode 4)
  (send *ri* :go-pos 3.0 0 0)
  ;; change to auto mode
  (send *ri* :set-locomotion 1))

(defun get-off()
  ;; Expected the Spot is in the elevator.
  (check-elevator-open "back")
  ;; Going back?
  (send *ri* :set-locomotion-mode 4)
  (send *ri* :go-pos -3.0 0 0)
  (send *ri* :set-locomotion 1))

(defun check-target-floor-arrived(target-floor)
  ;; Feature: using imu or camera and detect the current floor
  )
