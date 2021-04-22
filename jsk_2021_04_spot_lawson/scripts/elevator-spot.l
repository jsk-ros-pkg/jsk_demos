#!/usr/bin/env roseus
(setq *elevator-spot* t)
(load "package://spoteus/spot-interface.l")

(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(unless (boundp '*detection*) (load "package://elevator_spot/scripts/detection.l"))
(unless (boundp '*speech-recognition-service*) (load "package://elevator_spot/scripts/speech-recognition-service.l"))

(defun elevator-spot(start-floor goal-floor)
  ;; Using front camera to get in the elevator
  ;; change locomotion mode, see
  ;; https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=locomotion_hint#locomotionhint
  (send *ri* :set-locomotion-mode 4)
  (check-elevator-open "front" :th 500)
  (send *ri* :go-pos 3.0 0 0)
  (unix:sleep 5)
  ;; check elevator arrive
  ;; (wait-for-julius-trigger "すすんで")
  (check-elevator-open "back")
  ;; nazodance sinakya
  (send *ri* :go-pos -4.0 0 0)
  (send *ri* :set-locomotion-mode 1))