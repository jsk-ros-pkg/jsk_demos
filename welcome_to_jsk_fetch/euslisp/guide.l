#!/usr/bin/env roseus

(load "package://welcome_to_jsk_fetch/euslisp/utils.l")

(defun main ()
  ;; Init fetch-interface
  (fetch-init)

  ;; Reset-pose
  (send *ri* :angle-vector (send *fetch* :reset-pose))
  (send *ri* :wait-interpolation)

  ;; Register place names to julius
  (register-spots-to-julius)

  ;; Wait for person by face recognition and then greeting
  (greeting)

  ;; Ask destination
  (send *ri* :speak-jp "どこにいきますか？" :wait t)
  (unix:sleep 2) ;; Wait 2 seconds not to recognize fetch's own voice at next function

  ;; Go to spot from speech
  (go-to-spot-from-speech)
  )
